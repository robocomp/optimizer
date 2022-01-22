/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
#include <Eigen/Dense>
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

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
    {
        timer.setSingleShot(true);
        timer.start(Period);
    }

    std::vector<double> target_robot{1, 0};
    std::vector<double> init_robot{0, 0, 0};

    initialize_differential(target_robot, init_robot);
}

void SpecificWorker::compute()
{
    qInfo() << "------------------------";
    RoboCompGenericBase::TBaseState current_pose;
    try
    {
        //currentPose = self.omnirobot_proxy.getBaseState()
        differentialrobot_proxy->getBaseState(current_pose);
        //auto current_tr = Eigen::Vector2d(current_pose.x / 1000, current_pose.z / 1000);
    }
    catch(const Ice::Exception &e){ qInfo() << "Error connecting to base"; std::cout << e.what() << std::endl;}

    // laser
    auto &&[laser_poly_robot, laser_poly_world] = read_laser(current_pose);

}

void SpecificWorker::initialize_differential(const std::vector<double> &target_robot, const std::vector<double> &init_robot)
{
    casadi::Slice all;
    casadi::Opti opti = casadi::Opti();
    const double N = 10;

    // ---- state variables ---------
    casadi::MX state = opti.variable(3, N+1);
    auto pos = state(casadi::Slice(0,2), all);
    auto phi = state(2, all);

            // ---- inputs variables 2 adv and rot---------
    casadi::MX control = opti.variable(2, N);
    auto adv = control(0, all);
    auto rot = control(1, all);

    // initial and final state parameters
    casadi::MX target_oparam = opti.parameter(2);
    casadi::MX initial_oparam = opti.parameter(3);

    // lines
    std::vector<casadi::MX> obs_lines{opti.parameter(3), opti.parameter(3), opti.parameter(3), opti.parameter(3)};

    // target and init
    opti.set_value(target_oparam, target_robot);
    opti.set_value(initial_oparam, init_robot);

    // cost function
    auto sum_dist = opti.parameter();
    opti.set_value(sum_dist, 0.0);
    for(auto k : iter::range(N - 1))
        sum_dist =  casadi::MX::sumsqr(pos(all, k+1) - pos(all, k));

    //.minimize(sum_dist + 0.1*rot.sumsqr(rot) + adv.sumsqr(adv) + sqrt(pow(pos(all,-1) - target_oparam, 2)));
    opti.minimize(sum_dist  + casadi::MX::sumsqr(pos(all,-1) - target_oparam));

    // dynamic constraints for differential robot --------
    // dx/dt = f(x, u)   3 x 2 * 2 x 1 -> 3 x 1
    auto f = [](casadi::MX x, casadi::MX u) {return casadi::MX::vertcat(std::vector<casadi::MX>{ casadi::MX::horzcat(std::vector<casadi::MX>{sin(x(2)), 0.0}),
                                                                                                 casadi::MX::horzcat(std::vector<casadi::MX>{cos(x(2)), 0.0}),
                                                                                                 casadi::MX::horzcat(std::vector<casadi::MX>{0.0, 1.0})});};
    double dt = 1;   // timer interval in secs
    for(const auto k : iter::range(N))  // loop over control intervals
    {
        auto step = casadi::MX::vertcat(std::vector<casadi::MX>{sin(phi(k)) * adv, cos(phi(k)) * rot});
        //auto k1 = f(pos(all ,k), phi(k));
        auto x_next = pos(all, k) + step;
        opti.subject_to( pos(all, k + 1) == x_next);  // close  the gaps
    }

    // control constraints -----------
    opti.subject_to(opti.bounded(-0.5, adv, 0.5));  // control is limited meters
    opti.subject_to(opti.bounded(-1, rot, 1));         // control is limited

    // initial point constraints ------
    opti.subject_to(pos(0, 0) == initial_oparam(0));
    opti.subject_to(pos(1, 0) == initial_oparam(1));
    opti.subject_to(phi(0) == initial_oparam(2));

    // forward velocity constraints -----------
    opti.subject_to(adv >= 0);

}

std::vector<std::tuple<double, double, double>> SpecificWorker::points_to_lines(const std::vector<Eigen::Vector2d> &points_in_robot)
{
    std::vector<std::tuple<double, double, double>> lines;
    for(auto i :  iter::range(points_in_robot.size()))
    {
        auto p1 = points_in_robot[i];
        auto p2 = points_in_robot[(i + 1) % points_in_robot.size()];
        auto norm = (p1 - p2).norm();
        auto A = (p1[1] - p2[1]) / norm;
        auto B = (p2[0] - p1[0]) / norm;
        auto C = -((p1[1] - p2[1]) * p1[0] + (p2[0] - p1[0]) * p1[1]) / norm;
        lines.emplace_back(std::make_tuple(A, B, C));
    }
    return lines;
}

std::tuple<QPolygonF, QPolygonF> SpecificWorker::read_laser(const RoboCompGenericBase::TBaseState &pose)
{
    QPolygonF poly_robot, poly_world;
    return std::make_tuple(poly_robot, poly_world);
}

//void SpecificWorker::initialize_acado()
//{
//
//    // — state variables (acadoVariables.x)—
//    ACADO::DifferentialState x;
//    ACADO::DifferentialState y;
//    ACADO::DifferentialState phi;
//
//    // — control inputs —
//    ACADO::Control adv;
//    ACADO::Control rot;
//
//    // —- differential equations —-
//    ACADO::DifferentialEquation f;
//    f << dot(x) == adv*cos(phi);
//    f << dot(y) == adv*sin(phi);
//    f << dot(phi) == rot;
//
//    // — reference functions (acadoVariables.y) —
//    ACADO::Function rf;
//    ACADO::Function rfN;
//    rf  << x << y << phi;
//    rfN << x << y << phi;
//
//    // — constraints, weighting matrices for the reference functions —
//    // N=number of prediction time steps, Ts=step time interval
//    // Provide defined weighting matrices:
//    ACADO::BMatrix W = ACADO::eye<bool>(rf.getDim());
//    ACADO::BMatrix WN = ACADO::eye<bool>(rfN.getDim());
//
//    // define real-time optimal control problem
//    const double tStart = 0.0;
//    const double tEnd   = 1.0;
//    const double N = 20;
//    ACADO::OCP ocp(tStart, tEnd, N);
//    ocp.subjectTo( f );
//    ocp.subjectTo( -0.5 <= adv <= 0.5 );
//    ocp.subjectTo( -1 <= rot <= 1 );
//    ocp.minimizeLSQ(W, rf);
//    ocp.minimizeLSQEndTerm(WN, rfN);
//
//    // SETTING UP THE (SIMULATED) PROCESS:
//    // -----------------------------------
//    ACADO::OutputFcn identity;
//    ACADO::DynamicSystem dynamicSystem( f, identity );
//
//    ACADO::Process process( dynamicSystem, ACADO::INT_RK45 );
//
//    // SETTING UP THE MPC CONTROLLER:
//    // ------------------------------
//    ACADO::RealTimeAlgorithm alg( ocp, 0.05 );
//    alg.set( ACADO::MAX_NUM_ITERATIONS, 2 );
//
//    ACADO::StaticReferenceTrajectory zeroReference;
//    ACADO::Controller controller( alg, zeroReference );
//
//
//    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
//    // ----------------------------------------------------------
//    ACADO::SimulationEnvironment sim( 0.0,3.0,process,controller );
//
//    ACADO::DVector x0(3);
//    x0(0) = 0.01;
//    x0(1) = 0.0;
//    x0(2) = 0.0;
//
//    if (sim.init( x0 ) != ACADO::SUCCESSFUL_RETURN)
//        exit( EXIT_FAILURE );
//    if (sim.run( ) != ACADO::SUCCESSFUL_RETURN)
//        exit( EXIT_FAILURE );
//
//    // ...AND PLOT THE RESULTS
//    // ----------------------------------------------------------
//    ACADO::VariablesGrid sampledProcessOutput;
//    sim.getSampledProcessOutput( sampledProcessOutput );
//
//    ACADO::VariablesGrid feedbackControl;
//    sim.getFeedbackControl( feedbackControl );
//
//    ACADO::GnuplotWindow window;
//    window.addSubplot( sampledProcessOutput(0), "X" );
//    window.addSubplot( sampledProcessOutput(1), "Y" );
//    window.addSubplot( sampledProcessOutput(2), "Phi" );
//    window.addSubplot( feedbackControl(1),      "adv" );
//    window.addSubplot( feedbackControl(0),      "rot" );
//    window.plot( );
//
//}

/****************************************/
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// this->omnirobot_proxy->correctOdometer(...)
// this->omnirobot_proxy->getBasePose(...)
// this->omnirobot_proxy->getBaseState(...)
// this->omnirobot_proxy->resetOdometer(...)
// this->omnirobot_proxy->setOdometer(...)
// this->omnirobot_proxy->setOdometerPose(...)
// this->omnirobot_proxy->setSpeedBase(...)
// this->omnirobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

