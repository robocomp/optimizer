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
    readSettings();
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
    init_drawing(dim);

    // optimizer
    init_optmizer();

    this->Period = 100;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);
}

void SpecificWorker::compute()
{
    static Eigen::Vector2f target;
    auto bState = read_base();
    static int cont=0;
    //auto laser_poly = read_laser();
    //fill_grid(laser_poly);
    //auto [x,z,alpha] = state_change(bState, 0.1);  //secs
    //qInfo() << bState.x << bState.z << bState.alpha << "--" << x << z << alpha;

    // check for new target
    if(auto t = target_buffer.try_get(); t.has_value())
    {
        float tr_x = t.value().x()-bState.x; float tr_y = t.value().y() - bState.z;
        ref_ang = -atan2(tr_x, tr_y);   // signo menos para tener ángulos respecto a Y CCW
        xRef << t.value().x(), t.value().y(), ref_ang;

        solver.clearSolverVariables();
        cast_MPC_to_QP_gradient(Q, xRef, horizon, gradient);
        if (!solver.updateGradient(gradient)) return ;

        // draw
        if(target_draw) scene.removeItem(target_draw);
        target_draw = scene.addEllipse(t.value().x()-50, t.value().y()-50, 100, 100, QPen(QColor("green")), QBrush(QColor("green")));
        auto ex = t.value().x() + 350*sin(-ref_ang); auto ey  =  t.value().y() + 350*cos(-ref_ang);  //OJO signos porque el ang está respecto a Y CCW
        auto line = scene.addLine(t.value().x(), t.value().y(), ex, ey, QPen(QBrush(QColor("green")), 20));
        line->setParentItem(target_draw);
        auto ball = scene.addEllipse(ex-25, ey-25, 50, 50, QPen(QColor("green")), QBrush(QColor("green")));
        ball->setParentItem(target_draw);
        cont = 0;
        xGraph->data()->clear();wGraph->data()->clear();yGraph->data()->clear();exGraph->data()->clear();ewGraph->data()->clear();
        custom_plot.replot();
    }
    if( not atTarget)
    {
        x0 << bState.x, bState.z, bState.alpha;
        double pos_error = sqrt(pow(x0.x() - xRef.x(),2) + pow(x0.y() - xRef.y(),2));
        double rot_error = sqrt(pow(x0.z() - xRef.z(),2));
        if (pos_error < 40 and rot_error < 0.1)
        {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            std::cout << "FINISH" << std::endl;
            atTarget = true;
            return;
        }

        // update QP problem
        compute_jacobians(A, B, bState.advVx*this->Period/1000, bState.advVz/this->Period/1000, bState.alpha );
        cast_MPC_to_QP_constraint_matrix(A, B, horizon, linearMatrix);
        if (!solver.updateLinearConstraintsMatrix(linearMatrix)) qWarning() << "SHIT";
        update_constraint_vectors(x0, lowerBound, upperBound);
        if (!solver.updateBounds(lowerBound, upperBound)) qWarning() << "SHIT";

        // solve the QP problem
        if (!solver.solve()) { qInfo() << "Out solve "; return;};
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(state_dim * (horizon + 1), 0, control_dim, 1) * 2.4;
        //ctr = QPSolution.block(state_dim * (horizon + 1) + control_dim * 2, 0, control_dim, 1) * 2.4;

        // execute control
        omnirobot_proxy->setSpeedBase((float)ctr.x(), (float)ctr.y(), (float)ctr.z());

        // draw
        qInfo() << __FUNCTION__ << "  Control: " << ctr.x() << ctr.y() << ctr.z();
        qInfo() << "\t" << " Pose: " << bState.x << bState.z << bState.alpha ;
        qInfo() << "\t" << " Target: " << xRef.x() << xRef.y() << xRef.z() ;
        qInfo() << "\t" << " Error: " << pos_error << rot_error;
        qInfo() << "----------------------------------------------------";
        std::vector<std::tuple<float, float, float>> path;
        for(std::uint32_t i=0; i<horizon*state_dim; i+=state_dim)
            path.emplace_back(std::make_tuple(QPSolution(i, 0), QPSolution(i+1, 0), QPSolution(i+2, 0)));
        draw_path(path);
        for(int i=0; i<horizon*control_dim; i+=3)
        {
            StateSpaceVector s = QPSolution.block(i, 0, state_dim , 1);
            ControlSpaceVector c = QPSolution.block(state_dim * (horizon + 1) + i, 0, control_dim , 1);
            qInfo() << "------ " << (float)s.z() << (float)c.z() << ref_ang;
        }
        xGraph->addData(cont, ctr.x());
        yGraph->addData(cont, ctr.y());
        wGraph->addData(cont, ctr.z()*300);  // visual scale
        exGraph->addData(cont, pos_error);
        ewGraph->addData(cont, rot_error*300);  // visual scale
        cont++;
        custom_plot.replot();
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::init_optmizer()
{
    // set MPC problem quantities
    //set_dynamics_matrices(A, B);
    compute_jacobians(A, B, 0., 0., 0.);
    set_inequality_constraints(xMax, xMin, uMax, uMin, Eigen::Vector3d::Zero());
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
    solver.data()->setNumberOfVariables(state_dim * (horizon + 1) + control_dim * horizon);
    solver.data()->setNumberOfConstraints(2 * state_dim * (horizon + 1) + control_dim * horizon);
    if (!solver.data()->setHessianMatrix(hessian)) qWarning() << "SHIT";
    if (!solver.data()->setGradient(gradient))qWarning() << "SHIT";
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) qWarning() << "SHIT";
    if (!solver.data()->setLowerBound(lowerBound)) qWarning() << "SHIT";
    if (!solver.data()->setUpperBound(upperBound)) qWarning() << "SHIT";

    // instantiate the solver
    if (!solver.initSolver()) qWarning() << "SHIT";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::compute_jacobians(AMatrix &A, BMatrix &B, double u_x, double u_y, double alfa)
{
//    A <<    1., 0.,  -u_x * sin(alfa) - u_y * cos(alfa),
//            0., 1.,   u_x * cos(alfa) - u_y * sin(alfa),
//            0., 0.,   1. ;

    A <<    1., 0.,  1,
            0., 1.,  1,
            0., 0.,  1 ;

    B <<    cos(alfa),  -sin(alfa),   0.,
            sin(alfa),  cos(alfa),    0.,
            0.,          0.,          this->Period/100;   //PARAMETRO CRITICO

}

void SpecificWorker::set_inequality_constraints(StateConstraintsMatrix &xMax,
                                                StateConstraintsMatrix &xMin,
                                                ControlConstraintsMatrix &uMax,
                                                ControlConstraintsMatrix &uMin,
                                                const ControlConstraintsMatrix &uzero)
{
    xMax << 2500,
            2500,
            OsqpEigen::INFTY;
    xMin << -2500,
            -2500,
            -OsqpEigen::INFTY;

    uMax << 600,
            600,
            1;
    uMin << -600,
            0,
            -1;
    uMax -= uzero;
    uMin -= uzero;
}
void SpecificWorker::set_weight_matrices(QMatrix &Q, RMatrix &R)
{
    //Q.diagonal() << 1, 1, 1;
    //R.diagonal() << 1, 1, 1.;
    Q.diagonal() << 1, 1, 1000;
    R.diagonal() << 1, 1, 1000;
}
void SpecificWorker::cast_MPC_to_QP_hessian(const QMatrix &Q, const RMatrix &R, int horizon, Eigen::SparseMatrix<double> &hessianMatrix)
{
    // room to stack horizon + 1 states and horizon controls.
    hessianMatrix.resize(state_dim * (horizon + 1) + control_dim * horizon, state_dim * (horizon + 1) + control_dim * horizon);

    // populate hessian matrix
    for (std::uint32_t i = 0; i < state_dim * (horizon + 1) + control_dim * horizon; i++)
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
void SpecificWorker::cast_MPC_to_QP_gradient(const QMatrix &Q, const StateSpaceVector &xRef, std::uint32_t horizon, Eigen::VectorXd &gradient)
{

    Eigen::Matrix<double, state_dim, 1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(state_dim*(horizon+1) +  control_dim*horizon, 1);
    for(std::uint32_t i = 0; i<state_dim*(horizon+1); i++)
    {
        int posQ = i % state_dim;
        gradient(i,0) = Qx_ref(posQ,0);
    }
}
void SpecificWorker::cast_MPC_to_QP_constraint_matrix(const AMatrix &dynamicMatrix, const BMatrix &controlMatrix, std::uint32_t horizon, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(state_dim*(horizon+1)  + state_dim*(horizon+1) + control_dim*horizon, state_dim*(horizon+1) + control_dim*horizon);

    // populate linear constraint matrix
    for(std::uint32_t i= 0; i<state_dim*(horizon+1); i++)
        constraintMatrix.insert(i,i) = -1;

    for(std::uint32_t i = 0; i < horizon; i++)
        for(std::uint32_t j = 0; j<state_dim; j++)
            for(std::uint32_t k = 0; k<state_dim; k++)
            {
                float value = dynamicMatrix(j,k);
                if(value != 0)
                    constraintMatrix.insert(state_dim * (i+1) + j, state_dim * i + k) = value;
            }

    for(std::uint32_t i = 0; i < horizon; i++)
        for(std::uint32_t j = 0; j < state_dim; j++)
            for(std::uint32_t k = 0; k < control_dim; k++)
            {
                float value = controlMatrix(j,k);
                if(value != 0)
                    constraintMatrix.insert(state_dim*(i+1)+j, control_dim*i+k+state_dim*(horizon + 1)) = value;
            }

    for(std::uint32_t i = 0; i<state_dim*(horizon+1) + control_dim*horizon; i++)
        constraintMatrix.insert(i+(horizon+1)*state_dim, i) = 1;
}
void SpecificWorker::cast_MPC_to_QP_constraint_vectors(const StateConstraintsMatrix &xMax, const StateConstraintsMatrix &xMin,
                                                       const ControlConstraintsMatrix &uMax, const ControlConstraintsMatrix &uMin,
                                                       const StateSpaceVector &x0, std::uint32_t horizon, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(state_dim*(horizon+1) +  control_dim*horizon, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(state_dim*(horizon+1) +  control_dim*horizon, 1);
    for(std::uint32_t i=0; i<horizon+1; i++)
    {
        lowerInequality.block(state_dim*i,0,state_dim,1) = xMin;
        upperInequality.block(state_dim*i,0,state_dim,1) = xMax;
    }
    for(std::uint32_t i=0; i<horizon; i++)
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

void SpecificWorker::update_constraint_vectors(const StateSpaceVector &x0, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    lowerBound.block(0,0,state_dim,1) = -x0;
    upperBound.block(0,0,state_dim,1) = -x0;
}
double SpecificWorker::get_error_norm(const StateSpaceVector &x, const StateSpaceVector &xRef)
{
    // evaluate the error
    Eigen::Matrix<double, state_dim, 1> error = x - xRef;
    return error.transpose() * R * error;
}


RoboCompGenericBase::TBaseState SpecificWorker::read_base()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        omnirobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, -bState.alpha, 0);
        robot_polygon->setRotation(qRadiansToDegrees(bState.alpha));
        robot_polygon->setPos(bState.x, bState.z);

//        float deltaT = this->Period / 1000.f;
//        float w = qDegreesToRadians(robot_polygon->rotation());
//        QPointF p= robot_polygon->pos();
//        Eigen::Matrix<float, state_dim, control_dim> forward_model;
//        forward_model <<    cos(w),  -sin(w),               0.,
//                            sin(w),  cos(w),                0.,
//                            0.,          0.,                -this->Period / 1000.f;.;
//        Eigen::Vector3f new_pos = Eigen::Vector3f(p.x(), p.y(), w) + forward_model * Eigen::Vector3f(jside, jadv, jrot) * deltaT;
//        robot_polygon->setRotation(qRadiansToDegrees(new_pos.z()));
//        robot_polygon->setPos(new_pos.x(), new_pos.y());

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

void SpecificWorker::draw_path( const std::vector<std::tuple<float, float, float>> &path)
{
    static std::vector<QGraphicsLineItem *> path_paint;
    static QString arrow_color = "#0000FF";
    static QString ball_color = "#FF00FF";
    static const float len = 300;

    for(auto p : path_paint)
        scene.removeItem(p);
    path_paint.clear();
    auto [antx, anty, anta] = path.front();
    for(auto &[x, y, ang] : path)
    {
        auto ex = x + len*sin(-ang); auto ey  =  y + len*cos(-ang); //OJO pintamos con - porque ang viene de Y CCW
//        auto ex = x + len*sin(-ref_ang); auto ey  =  y + len*cos(-ref_ang);  //OJO pintamos con - porque ang viene de Y CCW
        auto l = scene.addLine(x, y, ex, ey, QPen(QBrush(QColor(arrow_color)), 20));
        path_paint.push_back(l);
        auto e = scene.addEllipse(ex - 25, ey - 25, 50, 50, QPen(ball_color), QBrush(QColor(ball_color)));
        e->setParentItem(l);
        auto ll = scene.addLine(antx, anty, x, y, QPen(QBrush(QColor(arrow_color)), 10));
        ll->setParentItem(l);
        antx = x; anty = y;
    }

}

void SpecificWorker::init_drawing( Grid<>::Dimensions dim)
{

    graphicsView->setScene(&scene);
    graphicsView->setMinimumSize(600,600);
    scene.setSceneRect(dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT);
    graphicsView->scale(1, -1);
    graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
    graphicsView->show();
    connect(&scene, &MyScene::new_target, this, [this](QGraphicsSceneMouseEvent *e)
    {
        qInfo() << "Lambda SLOT: " << e->scenePos();
        target_buffer.put(Eigen::Vector2f ( e->scenePos().x() , e->scenePos().y()));
        atTarget = false;
    });

    //Draw
    custom_plot.setParent(signal_frame);
    custom_plot.xAxis->setLabel("time");
    custom_plot.yAxis->setLabel("vx-blue vy-red vw-green dist-magenta ew-black");
    custom_plot.xAxis->setRange(0, 200);
    custom_plot.yAxis->setRange(-1500, 1500);
    xGraph = custom_plot.addGraph();
    xGraph->setPen(QColor("blue"));
    yGraph = custom_plot.addGraph();
    yGraph->setPen(QColor("red"));
    wGraph = custom_plot.addGraph();
    wGraph->setPen(QColor("green"));
    exGraph = custom_plot.addGraph();
    exGraph->setPen(QColor("magenta"));
    ewGraph = custom_plot.addGraph();
    ewGraph->setPen(QColor("black"));
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
    auto bcolor = QColor("DarkRed");
    bcolor.setAlpha(80);
    brush.setColor(bcolor);
    brush.setStyle(Qt::SolidPattern);
    robot_polygon = scene.addPolygon(poly2, QPen(QColor("DarkRed")), brush);
    robot_polygon->setZValue(5);
    try
    {
        RoboCompGenericBase::TBaseState bState;
        omnirobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(qRadiansToDegrees(bState.alpha));
        robot_polygon->setPos(bState.x, bState.z);
    }
    catch(const Ice::Exception &e){};
}
///////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::JoystickAdapter_sendData (RoboCompJoystickAdapter::TData data)
{
    for(auto &x : data.axes)
    {
        if(x.name == "advance"){
            if(fabs(x.value) > 0.1) jadv = x.value; else jadv = 0;}
        if(x.name == "rotate"){
            if(fabs(x.value) > 0.1) jrot = x.value; else jrot = 0;}
        if(x.name == "side"){
            if(fabs(x.value) > 0.1) jside = x.value; else jside = 0;}
    }
//    jadv = (jadv / ViriatoBase_WheelRadius);
//    jside = (jside / ViriatoBase_WheelRadius);
//    jrot = (jrot * ViriatoBase_Rotation_Factor);
      jadv *= 2.4;
      jside *= 2.4;
      jrot *= 2.4;

}
