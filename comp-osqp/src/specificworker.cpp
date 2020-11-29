/*
 *    Copyright (C) 2020 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>
#include <algorithm>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    try
    {
        RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
        std::string innermodel_path = par.value;
        innerModel = std::make_shared<InnerModel>(innermodel_path);

    }
    catch(const std::exception &e) { qFatal("Error reading config params"); }
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;

    //grid
    Grid<>::Dimensions dim;  //default values
    grid.initialize(&scene, dim);

    //view
    graphicsView->setScene(&scene);
    graphicsView->setMinimumSize(400,400);
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    scene.setSceneRect(dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT);
    graphicsView->scale(1, -1);
    graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
    graphicsView->show();
    connect(&scene, &MyScene::new_target, this, [this](QGraphicsSceneMouseEvent *e)
                    {
                        qInfo() << "Lambda SLOT: " << e->scenePos();
                        target_buffer.put(Eigen::Vector2f ( e->scenePos().x() , e->scenePos().y()));
                        xGraph->data()->clear(); yGraph->data()->clear();
                        atTarget = false;
                    });

    //Draw
    custom_plot.setParent(signal_frame);
    custom_plot.xAxis->setLabel("time");
    custom_plot.yAxis->setLabel("velocity");
    custom_plot.xAxis->setRange(0, 200);
    custom_plot.yAxis->setRange(-1500, 1500);
    xGraph = custom_plot.addGraph();
    xGraph->setPen(QColor("blue"));
    yGraph = custom_plot.addGraph();
    yGraph->setPen(QColor("red"));
    custom_plot.resize(signal_frame->size());
    custom_plot.show();

    //robot
    QPolygonF poly2;
    float size = ROBOT_LENGTH / 2.f;
    poly2 << QPoint(-size, -size)
          << QPoint(-size, size)
          << QPoint(-size / 3, size * 1.6)
          << QPoint(size / 3, size * 1.6)
          << QPoint(size, size)
          << QPoint(size, -size);
    QBrush brush;
    brush.setColor(QColor("DarkRed"));
    brush.setStyle(Qt::SolidPattern);
    robot_polygon = scene.addPolygon(poly2, QPen(QColor("DarkRed")), brush);
    robot_polygon->setZValue(5);
    robot_polygon->setPos(0,0);

    init_optmizer();

    this->Period = period;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);
}

void SpecificWorker::compute()
{
    static Eigen::Vector2f target;
    auto bState = read_base();
    //auto laser_poly = read_laser();
    //fill_grid(laser_poly);
    //auto [x,z,alpha] = state_change(bState, 0.1);  //secs
    //qInfo() << bState.x << bState.z << bState.alpha << "--" << x << z << alpha;

    // check for new target
    if(auto t = target_buffer.try_get(); t.has_value())
    {
        xRef << t.value().x(), t.value().y();
        cast_MPC_to_QP_gradient(Q, xRef, horizon, gradient);
        if (!solver.updateGradient(gradient)) return ;
    }
    if( not atTarget)
    {

        x0 << bState.x, bState.z;
        double err = get_error_norm(x0, xRef);
        //std::cout << __FUNCTION__ << " ------------- Initial state " << x0 << std::endl;
        if (err < 50)
        {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            std::cout << "FINISH" << std::endl;
            atTarget = true;
            return;
        }

        // update QP problem
        if (!solver.updateHessianMatrix(hessian)) return {};
        if (!solver.updateGradient(gradient)) return {};
        if (!solver.updateLinearConstraintsMatrix(linearMatrix)) return {};
        // update the constraint bound
        update_constraint_vectors(x0, lowerBound, upperBound);
        if (!solver.updateBounds(lowerBound, upperBound)) return;

        // solve the QP problem
        if (!solver.solve()) { qInfo() << "Out solve "; return;};
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(2 * (horizon + 1), 0, 2, 1);

        // execute control
        qInfo() << __FUNCTION__ << " Control: " << ctr.x() << ctr.y() << " Dist: " << err;
        auto nose = innerModel->transform("base", QVec::vec3(QPSolution(2, 0), 0., QPSolution(3, 0)), "world");
        float angle = atan2(nose.x(), nose.z());

        // convert mm/sg into radians. Should be un omniroboPyrep
        ctr = ctr / ViriatoBase_WheelRadius;
        auto ll = (ViriatoBase_DistAxes + ViriatoBase_AxesLength) / (2.f*1000.f);
        auto crt_angle = std::clamp(angle, -1.f, 1.f);
        omnirobot_proxy->setSpeedBase((float)ctr.x(), (float)ctr.y(), 0);

//        auto control = (xRef - x0);
//        qInfo() << xRef.x() << xRef.y() << x0.x() << x0.y() << control.x() << control.y();
//        omnirobot_proxy->setSpeedBase(control.x()/ViriatoBase_WheelRadius, control.y()/ViriatoBase_WheelRadius, 0);

        // draw
        std::vector<QPointF> path;
        for(int i=0; i<horizon*2; i+=2)
            path.emplace_back(QPointF(QPSolution(i, 0), QPSolution(i+1, 0)));
        draw_path(path);
        xGraph->addData(cont, QPSolution(0, 0));
        yGraph->addData(cont, QPSolution(1,0));
        cont++;
        custom_plot.replot();



    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
std::optional<Eigen::Matrix<double, 2, 1>> SpecificWorker::init_optmizer()
{

    solver.clearSolverVariables();
    // set MPC problem quantities
    set_dynamics_matrices(A, B);
    set_inequality_constraints(xMax, xMin, uMax, uMin);
    set_weight_matrices(Q, R);

    // cast the MPC problem as QP problem
    cast_MPC_to_QP_hessian(Q, R, horizon, hessian);
    cast_MPC_to_QP_gradient(Q, xRef, horizon, gradient);
    cast_MPC_to_QP_constraint_matrix(A, B, horizon, linearMatrix);
    cast_MPC_to_QP_constraint_vectors(xMax, xMin, uMax, uMin, x0, horizon, lowerBound, upperBound);

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(2 * (horizon + 1) + 2 * horizon);
    solver.data()->setNumberOfConstraints(2 * 2 * (horizon + 1) + 2 * horizon);
    if (!solver.data()->setHessianMatrix(hessian)) return {};
    if (!solver.data()->setGradient(gradient)) return {};
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return {};
    if (!solver.data()->setLowerBound(lowerBound)) return {};
    if (!solver.data()->setUpperBound(upperBound)) return {};

    // instantiate the solver
    if (!solver.initSolver()) return {};
    return x0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::compute_jacobians(AMatrix &A, BMatrix &B, double vx, double vy, double vr, double deltaT)
{
    double alfa = vr * deltaT;
    A <<    1., 0.,  (-vx * sin(alfa) - vy*cos(alfa))*deltaT,
            0., 1.,  (vx * cos(alfa) - vy * sin(alfa))*deltaT;

    B <<    cos(alfa),  -sin(alfa),   0.,
            sin(alfa),  cos(alfa),    0.,
            0.,          0.,          1.;
}
void SpecificWorker::set_dynamics_matrices(AMatrix &A, BMatrix &B)
{
    A <<    1., 0.,
            0., 1.;

    B <<    1., 0.,
            0., 1.;
}
void SpecificWorker::set_inequality_constraints(StateConstraintsMatrix &xMax,
                                                StateConstraintsMatrix &xMin,
                                                ControlConstraintsMatrix &uMax,
                                                ControlConstraintsMatrix &uMin)
{
    xMax << 2500,
            2500;
    xMin << -2500,
            -2500;

    double u0 = 0;  // control at current linearization point
    uMax << 600 - u0,
            600 - u0;
    uMin << -600 - u0,
            -600 - u0;
}
void SpecificWorker::set_weight_matrices(QMatrix &Q, RMatrix &R)
{
    Q.diagonal() << 1., 1.;
    R.diagonal() << 1, 1;
}
void SpecificWorker::cast_MPC_to_QP_hessian(const QMatrix &Q, const RMatrix &R, int horizon, Eigen::SparseMatrix<double> &hessianMatrix)
{
    // room to stack horizon + 1 states and horizon controls.
    hessianMatrix.resize(state_dim * (horizon + 1) + control_dim * horizon, state_dim * (horizon + 1) + control_dim * horizon);

    // populate hessian matrix
    for (int i = 0; i < state_dim * (horizon + 1) + control_dim * horizon; i++)
    {
        if (i < state_dim * (horizon + 1))  // the state part
        {
            float value = Q.diagonal()[i % state_dim]; // 0, 1 ... state_dim over Q
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
        else
        {
            float value = R.diagonal()[i % control_dim]; // 0, 1 ... control_dim over R
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
    }
}
void SpecificWorker::cast_MPC_to_QP_gradient(const QMatrix &Q, const StateSpaceMatrix &xRef, std::uint32_t horizon, Eigen::VectorXd &gradient)
{
    Eigen::Matrix<double, state_dim, 1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(state_dim*(horizon+1) +  control_dim*horizon, 1);
    for(int i = 0; i<state_dim*(horizon+1); i++)
    {
        int posQ = i % state_dim;
        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}
void SpecificWorker::cast_MPC_to_QP_constraint_matrix(const AMatrix &dynamicMatrix, const BMatrix &controlMatrix, std::uint32_t horizon, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(state_dim*(horizon+1)  + state_dim*(horizon+1) + control_dim*horizon, state_dim*(horizon+1) + control_dim*horizon);

    // populate linear constraint matrix
    for(int i = 0; i<state_dim*(horizon+1); i++)
        constraintMatrix.insert(i,i) = -1;

    for(int i = 0; i < horizon; i++)
        for(int j = 0; j<state_dim; j++)
            for(int k = 0; k<state_dim; k++)
            {
                float value = dynamicMatrix(j,k);
                if(value != 0)
                    constraintMatrix.insert(state_dim * (i+1) + j, state_dim * i + k) = value;
            }

    for(int i = 0; i < horizon; i++)
        for(int j = 0; j < state_dim; j++)
            for(int k = 0; k < control_dim; k++)
            {
                float value = controlMatrix(j,k);
                if(value != 0)
                    constraintMatrix.insert(state_dim*(i+1)+j, control_dim*i+k+state_dim*(horizon + 1)) = value;
            }

    for(int i = 0; i<state_dim*(horizon+1) + control_dim*horizon; i++)
        constraintMatrix.insert(i+(horizon+1)*state_dim, i) = 1;
}

void SpecificWorker::cast_MPC_to_QP_constraint_vectors(const StateConstraintsMatrix &xMax, const StateConstraintsMatrix &xMin,
                                                       const ControlConstraintsMatrix &uMax, const ControlConstraintsMatrix &uMin,
                                                       const StateSpaceMatrix &x0, std::uint32_t horizon, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(state_dim*(horizon+1) +  control_dim*horizon, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(state_dim*(horizon+1) +  control_dim*horizon, 1);
    for(int i=0; i<horizon+1; i++)
    {
        lowerInequality.block(state_dim*i,0,state_dim,1) = xMin;
        upperInequality.block(state_dim*i,0,state_dim,1) = xMax;
    }
    for(int i=0; i<horizon; i++)
    {
        lowerInequality.block(control_dim*i + state_dim*(horizon + 1), 0, control_dim, 1) = uMin;
        upperInequality.block(control_dim*i + state_dim*(horizon + 1), 0, control_dim, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(state_dim*(horizon+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,state_dim,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*state_dim*(horizon+1) +  control_dim*horizon,1 );
    lowerBound << lowerEquality, lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*state_dim*(horizon+1) +  control_dim*horizon,1 );
    upperBound << upperEquality, upperInequality;
}

void SpecificWorker::update_constraint_vectors(const StateSpaceMatrix &x0, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    lowerBound.block(0,0,state_dim,1) = -x0;
    upperBound.block(0,0,state_dim,1) = -x0;
}

double SpecificWorker::get_error_norm(const StateSpaceMatrix &x, const StateSpaceMatrix &xRef)
{
    // evaluate the error
    Eigen::Matrix<double, state_dim, 1> error = x - xRef;
    return error.norm();
}


RoboCompGenericBase::TBaseState SpecificWorker::read_base()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        omnirobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        robot_polygon->setRotation(qRadiansToDegrees(bState.alpha));
        robot_polygon->setPos(bState.x, bState.z);
    }
    catch(const Ice::Exception &e)
    { std::cout << "Error reading from Camera" << e << std::endl;}
    return bState;
}

QPolygonF SpecificWorker::read_laser()
{
    QPolygonF laser_poly;
    try
    {
        auto ldata = laser_proxy->getLaserData();
        laser_poly = draw_laser(ldata);
    }
    catch(const Ice::Exception &e)
    { std::cout << "Error reading from Camera" << e << std::endl;}
    return laser_poly;
}

std::tuple<float,float,float> SpecificWorker::state_change( const RoboCompGenericBase::TBaseState &bState, float delta_t)
{
    float rot = bState.rotV;
    float adv = bState.advVz;

    if (fabs(rot) > 0.05)
    {
        float r = adv / rot; //radio
        float x = r - r * cos(rot * delta_t);
        float z = r * sin(rot * delta_t);
        QVec res =  innerModel->transform6D("world", QVec::vec6(x, 0, z, 0, rot * delta_t, 0), "base");
        return std::make_tuple(res.x(), res.z(), res.ry());
    }
    else
    {
        QVec res = innerModel->transform6D("world", QVec::vec6(0, 0,adv * delta_t, 0, 0, 0), "base");
        return std::make_tuple(res.x(), res.z(), res.ry());
    }
}

void SpecificWorker::fill_grid(const QPolygonF &laser_poly)
{
    for(auto &[k, v] : grid)
        if(laser_poly.containsPoint(QPointF(k.x, k.z), Qt::OddEvenFill))
            v.free = true;
        else
            v.free = false;
    grid.draw(&scene);
}

QPolygonF SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
{
    if (laser_polygon != nullptr)
        scene.removeItem(laser_polygon);

    QPolygonF poly;
    for( auto &l : ldata)
        poly << robot_polygon->mapToScene(QPointF(l.dist/1000. * sin(l.angle), l.dist/1000. * cos(l.angle)));

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = scene.addPolygon(poly, QPen(color), QBrush(color));
    laser_polygon->setZValue(3);
    return poly;
}

void SpecificWorker::draw_path( const std::vector<QPointF> &path)
{
    for(auto p : path_paint)
        scene.removeItem(p);
    path_paint.clear();
    for(auto &p : path)
        path_paint.push_back(scene.addEllipse(p.x()-25, p.y()-25, 50 , 50, QPen(path_color), QBrush(QColor(path_color))));
}

///////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////
