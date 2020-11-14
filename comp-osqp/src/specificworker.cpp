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

    // target
    target = QPointF(0, 2200);

    // path
    for(auto i : iter::range(0, 2200, 100))
        path.emplace_back(QPointF((100 - qrand()%200), i));
    draw_path();

    if( auto res = init_optmizer(); res.has_value())
    {
        std::cout << res.value() << std::endl;
    }
    else
        std::cout << "ERROR" << std::endl;


	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    auto bState = read_base();
    auto laser_poly = read_laser();
    fill_grid(laser_poly);
    auto [x,z,alpha] = state_change(bState, 0.1);  //secs
    //qInfo() << bState.x << bState.z << bState.alpha << "--" << x << z << alpha;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
std::optional<Eigen::Matrix<double, 4, 1>> SpecificWorker::init_optmizer()
{
    // set the preview window
    int mpcWindow = 20;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 2> B;

    // allocate the constraints vector
    Eigen::Matrix<double, 4, 1> xMax;
    Eigen::Matrix<double, 4, 1> xMin;
    Eigen::Matrix<double, 2, 1> uMax;
    Eigen::Matrix<double, 2, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 4> Q;
    Eigen::DiagonalMatrix<double, 2> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 4, 1> x0;
    Eigen::Matrix<double, 4, 1> xRef;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // set the initial and the desired states
    x0 << 0, 0, 0, 0;
    xRef << 0, 2200, 0, 0;

    // set MPC problem quantities
    setDynamicsMatrices(A, B);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian);
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
    castMPCToQPConstraintMatrix(A, B, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(4 * (mpcWindow + 1) + 2 * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 4 * (mpcWindow + 1) +  2 * mpcWindow);
    if(!solver.data()->setHessianMatrix(hessian)) return {};
    if(!solver.data()->setGradient(gradient)) return {};
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return {};
    if(!solver.data()->setLowerBound(lowerBound)) return {};
    if(!solver.data()->setUpperBound(upperBound)) return {};

    // instantiate the solver
    if(!solver.initSolver()) return {};

    // controller input and QPSolution vector
    Eigen::Vector2d ctr;
    Eigen::VectorXd QPSolution;

    // number of iteration steps
    int numberOfSteps = 50;

    for (int i = 0; i < numberOfSteps; i++)
    {

        // solve the QP problem
        if(!solver.solve()) return {};

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(4 * (mpcWindow + 1), 0, 4, 1);

        // save data into file
        //auto x0Data = x0.data();

        // propagate the model
        x0 = A * x0 + B * ctr;

        // update the constraint bound
        updateConstraintVectors(x0, lowerBound, upperBound);
        if(!solver.updateBounds(lowerBound, upperBound)) return {};
    }
    std::cout << x0 << std::endl;
    return x0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::setDynamicsMatrices(Eigen::Matrix<double, 4, 4> &A, Eigen::Matrix<double, 4, 2> &B)
{
    A <<    1., 0., 1., 0.,
            0., 1., 0., 1.,
            0., 0., 1., 0.,
            0., 0., 0., 1.;

    B <<    0., 0.,
            0., 0.,
            1., 0.,
            0., 1.;
}
void SpecificWorker::setInequalityConstraints(Eigen::Matrix<double, 4, 1> &xMax, Eigen::Matrix<double, 4, 1> &xMin,
                                              Eigen::Matrix<double, 2, 1> &uMax, Eigen::Matrix<double, 2, 1> &uMin)
{
    xMax << 2500,
            2500,
            1.,
            1.;
    xMin << -2500,
            -2500,
            1.,
            1.;
    uMax << 1000,
            1;
    uMin << -1000,
            -1;
}
void SpecificWorker::setWeightMatrices(Eigen::DiagonalMatrix<double, 4> &Q, Eigen::DiagonalMatrix<double, 2> &R)
{
    Q.diagonal() << 0, 0, 1., 1.;
    R.diagonal() << 0.1, 0.1;
}
void SpecificWorker::castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 4> &Q, const Eigen::DiagonalMatrix<double, 2> &R, int mpcWindow,
                                        Eigen::SparseMatrix<double> &hessianMatrix)
{
    hessianMatrix.resize(4 * (mpcWindow + 1) + 2 * mpcWindow, 4 * (mpcWindow + 1) + 2 * mpcWindow);

    //populate hessian matrix
    for (int i = 0; i < 4 * (mpcWindow + 1) + 2 * mpcWindow; i++)
    {
        if (i < 4 * (mpcWindow + 1))
        {
            int posQ = i % 4;
            float value = Q.diagonal()[posQ];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        } else
        {
            int posR = i % 2;
            float value = R.diagonal()[posR];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
    }
}
void SpecificWorker::castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 4> &Q, const Eigen::Matrix<double, 4, 1> &xRef, int mpcWindow,
                                         Eigen::VectorXd &gradient)
{
    Eigen::Matrix<double,4,1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(4*(mpcWindow+1) +  2*mpcWindow, 1);
    for(int i = 0; i<4*(mpcWindow+1); i++)
    {
        int posQ=i%4;
        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}
void SpecificWorker::castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 4, 4> &dynamicMatrix, const Eigen::Matrix<double, 4, 2> &controlMatrix,
                                                 int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(4*(mpcWindow+1)  + 4*(mpcWindow+1) + 2 * mpcWindow, 4*(mpcWindow+1) + 2 * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<4*(mpcWindow+1); i++)
    {
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<4; j++)
            for(int k = 0; k<4; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(4 * (i+1) + j, 4 * i + k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 4; j++)
            for(int k = 0; k < 2; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(4*(i+1)+j, 2*i+k+4*(mpcWindow + 1)) = value;
                }
            }

    for(int i = 0; i<4*(mpcWindow+1) + 2*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*4,i) = 1;
    }
}

void SpecificWorker::castMPCToQPConstraintVectors(const Eigen::Matrix<double, 4, 1> &xMax, const Eigen::Matrix<double, 4, 1> &xMin,
                                  const Eigen::Matrix<double, 2, 1> &uMax, const Eigen::Matrix<double, 2, 1> &uMin,
                                  const Eigen::Matrix<double, 4, 1> &x0,
                                  int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(4*(mpcWindow+1) +  2 * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(4*(mpcWindow+1) +  2 * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(4*i,0,4,1) = xMin;
        upperInequality.block(4*i,0,4,1) = xMax;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(2 * i + 4 * (mpcWindow + 1), 0, 2, 1) = uMin;
        upperInequality.block(2 * i + 4 * (mpcWindow + 1), 0, 2, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(4*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,4,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*4*(mpcWindow+1) +  2*mpcWindow,1 );
    lowerBound << lowerEquality,
            lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*4*(mpcWindow+1) +  2*mpcWindow,1 );
    upperBound << upperEquality,
            upperInequality;
}


void SpecificWorker::updateConstraintVectors(const Eigen::Matrix<double, 4, 1> &x0,
                                             Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    lowerBound.block(0,0,4,1) = -x0;
    upperBound.block(0,0,4,1) = -x0;
}

double SpecificWorker::getErrorNorm(const Eigen::Matrix<double, 4, 1> &x,
                    const Eigen::Matrix<double, 4, 1> &xRef)
{
    // evaluate the error
    Eigen::Matrix<double, 4, 1> error = x - xRef;
    // return the norm
    return error.norm();
}


RoboCompGenericBase::TBaseState SpecificWorker::read_base()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        robot_polygon->setRotation(qRadiansToDegrees(-bState.alpha));
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

void SpecificWorker::draw_path()
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



