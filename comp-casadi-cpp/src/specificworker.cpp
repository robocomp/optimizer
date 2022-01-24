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
#include <cppitertools/enumerate.hpp>
#include <cppitertools/chunked.hpp>

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
        QRectF dimensions(-5100, -2600, 10200, 5200);
        viewer_robot = new AbstractGraphicViewer(this->graphicsView, dimensions);
        robot_polygon = viewer_robot->add_robot(ROBOT_LENGTH);
        laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
        laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
        this->resize(700,450);

        target_in_world = {0, 2}; //meters
        std::vector<double> init_robot{0, 0, 0};
        NUM_STEPS = 10;

        obs_points = {{4-0.5, 0.5}, {4+0.5, 0.5}, {4+0.5, -0.5}, {4-0.5, -0.5}};
        initialize_differential(NUM_STEPS, e2v(target_in_world), init_robot);

        //timer.setSingleShot(true);
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    //static casadi::OptiSol solution = opti.solve();
    static std::vector<double> previous_values;

    qInfo() << "------------------------";
    RoboCompGenericBase::TBaseState current_pose;
    Eigen::Vector2d current_tr;
    try
    {
        //currentPose = self.omnirobot_proxy.getBaseState()
        differentialrobot_proxy->getBaseState(current_pose);
        current_tr[0] = current_pose.x / 1000; current_tr[1] = current_pose.z / 1000;
        robot_polygon->setRotation(current_pose.alpha * 180 / M_PI);
        robot_polygon->setPos(current_pose.x, current_pose.z);
    }
    catch(const Ice::Exception &e){ qInfo() << "Error connecting to base"; std::cout << e.what() << std::endl;}

    // laser
    auto &&[laser_poly_robot, laser_poly_world] = read_laser(current_tr, current_pose.alpha);
    auto obstacles = compute_laser_partitions(laser_poly_robot);
    draw_partitions(obstacles, QColor("Magenta"));

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    double dist_to_target = (current_tr - target_in_world).norm();
    if( this->active and dist_to_target > 0.15)
    {
        // recompute the length of the optimization path if close to target
        if(dist_to_target < 2)
            initialize_differential(10, e2v(from_world_to_robot(target_in_world, current_tr, current_pose.alpha)),
                                    std::vector<double>{0, 0, 0});
       else // Warm start
           if (not previous_values.empty())
               for (auto i: iter::range(1, NUM_STEPS))
                //    opti.set_initial(state(casadi::Slice(), i), solution.value(state(casadi::Slice(), i)));
                   opti.set_initial(state(casadi::Slice(), i), std::vector<double>{previous_values[3*i], 
                                                                                   previous_values[3*i+1], 
                                                                                   previous_values[3*i+2]});
            //else
                // initialize generating a line segment from 0 to target


        // initial values for state ---
        opti.set_value(initial_oparam, std::vector<double>{0.0, 0.0, 0.0});
        opti.set_value(target_oparam, e2v(from_world_to_robot(target_in_world, current_tr, current_pose.alpha)));

        // obstacles
        // self.compute_free_zones()
        qInfo() << __FUNCTION__ << "in compute";
        std::vector<Eigen::Vector2d> obs_points_in_robotSR(obs_points.size());             // [np.array(2)]*4
        for( auto &&[i, p] : iter::enumerate(obs_points))
            obs_points_in_robotSR[i] = from_world_to_robot(p, current_tr, current_pose.alpha);
        auto lines = points_to_lines(obs_points_in_robotSR);
        for(auto &&[i, l] : iter::enumerate(lines))
            opti.set_value(obs_lines[i], l);

        // solve NLP ------
        try
        {
            auto solution = opti.solve();
            previous_values = std::vector<double>(solution.value(state)); 

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

            // print output -----
            auto advance = std::vector<double>(solution.value(adv)).front() * 1000;
            auto rotation = std::vector<double>(solution.value(rot)).front();
            qInfo() << __FUNCTION__ << "Iterations:" << (int) solution.stats()["iter_count"];
            qInfo() << __FUNCTION__ << "Adv: " << advance << "Rot:" << rotation;

            // move the robot
            //move_robot(advance *3, rotation);

            // draw
            auto path = std::vector<double>(solution.value(pos));
            draw_path(path, Eigen::Vector2d(current_pose.x, current_pose.z), current_pose.alpha);
        }
        catch (...) {}
    }
    else  // at  target
    {
        move_robot(0, 0);
        active = false;
        qInfo() << __FUNCTION__ << "Stopping";
    }
}

void SpecificWorker::initialize_differential(const int N, const std::vector<double> &target_robot, const std::vector<double> &init_robot)
{
    casadi::Slice all;
    opti = casadi::Opti();

    auto s_options = casadi::Dict();
    auto p_options = casadi::Dict();
    s_options["max_iter"] = 10000;
    s_options["print_level"] = 0;
    s_options["acceptable_tol"] = 1e-8;
    s_options["acceptable_obj_change_tol"] = 1e-6;
    opti.solver("ipopt", p_options, s_options);

    // ---- state variables ---------
    state = opti.variable(3, N+1);
    pos = state(casadi::Slice(0,2), all);
    phi = state(2, all);

    // ---- inputs variables 2 adv and rot---------
    control = opti.variable(2, N);
    adv = control(0, all);
    rot = control(1, all);

    // initial and final state parameters
    target_oparam = opti.parameter(2);
    initial_oparam = opti.parameter(3);

    // lines
    obs_lines = {opti.parameter(3), opti.parameter(3), opti.parameter(3), opti.parameter(3)};

    std::vector<Eigen::Vector2d> obs_points_in_robotSR(obs_points.size());             // [np.array(2)]*4
    RoboCompGenericBase::TBaseState current_pose;
    differentialrobot_proxy->getBaseState(current_pose);
    for( auto &&[i, p] : iter::enumerate(obs_points))
        obs_points_in_robotSR[i] = from_world_to_robot(p, Eigen::Vector2d(current_pose.x/1000, current_pose.z/1000), current_pose.alpha);
    auto lines = points_to_lines(obs_points_in_robotSR);
    for(auto &&[i, l] : iter::enumerate(lines))
        opti.set_value(obs_lines[i], l);

    // target and init
    opti.set_value(target_oparam, target_robot);
    opti.set_value(initial_oparam, init_robot);

    // cost function
    auto sum_dist = opti.parameter();
    opti.set_value(sum_dist, 0.0);
    for(auto k : iter::range(N - 1))
        sum_dist =  casadi::MX::sumsqr(pos(all, k+1) - pos(all, k));

    //.minimize(sum_dist + 0.1*rot.sumsqr(rot) + adv.sumsqr(adv) + sqrt(pow(pos(all,-1) - target_oparam, 2)));
    opti.minimize(sum_dist +
                  casadi::MX::sumsqr(pos(all,-1) - target_oparam) +
                  0.1 * casadi::MX::sumsqr(rot) +
                  casadi::MX::sumsqr(adv));

    // dynamic constraints for differential robot: dx/dt = f(x, u)   3 x 2 * 2 x 1 -> 3 x 1
    auto integrate = [](casadi::MX phi, casadi::MX u) { return casadi::MX::mtimes(
                                                casadi::MX::vertcat(std::vector<casadi::MX>{
                                                    casadi::MX::horzcat(std::vector<casadi::MX>{sin(phi), 0.0}),
                                                    casadi::MX::horzcat(std::vector<casadi::MX>{cos(phi), 0.0}),
                                                    casadi::MX::horzcat(std::vector<casadi::MX>{0.0,      1.0})}
                                                ), u);};
    double dt = 0.5;   // timer interval in secs
    for(const auto k : iter::range(N))  // loop over control intervals
    {
        //auto step = casadi::MX::vertcat(std::vector<casadi::MX>{sin(phi(k)) * adv(k), cos(phi(k)) * adv(k), rot(k)});
        auto x_next = state(all, k) + dt * integrate(phi(k), control(all,k));
        opti.subject_to( state(all, k + 1) == x_next);  // close  the gaps
    }

    // control constraints -----------
    opti.subject_to(opti.bounded(-0.5, adv, 0.5));  // control is limited meters
    opti.subject_to(opti.bounded(-1, rot, 1));         // control is limited

    // initial point constraints ------
    opti.subject_to(state(all,0) == initial_oparam);

    // forward velocity constraints -----------
    opti.subject_to(adv >= 0);

    // obstacle constraints
    for(auto i : iter::range(3, N-5))
        for(auto l : obs_lines)
            opti.subject_to(pos(0,i)*l(0) + pos(1,i)*l(1) + l(2) >= 0.5);
}
/////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d SpecificWorker::from_robot_to_world(const Eigen::Vector2d &p, const Eigen::Vector2d &robot_tr, double robot_ang)
{
    Eigen::Matrix2d matrix;
    matrix << cos(robot_ang) , -sin(robot_ang) , sin(robot_ang) , cos(robot_ang);
    return (matrix * p) + robot_tr;
}
Eigen::Vector2d SpecificWorker::from_world_to_robot(const Eigen::Vector2d &p, const Eigen::Vector2d &robot_tr, double robot_ang)
{
    Eigen::Matrix2d matrix;
    matrix << cos(robot_ang) , -sin(robot_ang) , sin(robot_ang) , cos(robot_ang);
    return (matrix.transpose() * (p - robot_tr));
}
std::vector<double> SpecificWorker::e2v(const Eigen::Vector2d &v)
{
    return std::vector<double>{v.x(), v.y()};
}
QPointF SpecificWorker::e2q(const Eigen::Vector2d &v)
{
    return QPointF(v.x(), v.y());
}
void SpecificWorker::move_robot(float adv, float rot, float side)
{
    try
    {
        differentialrobot_proxy->setSpeedBase(adv, rot);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
}
std::vector<std::vector<double>> SpecificWorker::points_to_lines(const std::vector<Eigen::Vector2d> &points_in_robot)
{
    std::vector<std::vector<double>> lines(points_in_robot.size());
    for(auto i :  iter::range(points_in_robot.size()))
    {
        auto p1 = points_in_robot[i];
        auto p2 = points_in_robot[(i + 1) % points_in_robot.size()];
        auto norm = (p1 - p2).norm();
        auto A = (p1[1] - p2[1]) / norm;
        auto B = (p2[0] - p1[0]) / norm;
        auto C = -((p1[1] - p2[1]) * p1[0] + (p2[0] - p1[0]) * p1[1]) / norm;
        qInfo()<<A<<B<<C;
        lines[i] = {A, B, C};
    }
    return lines;
}
std::tuple<QPolygonF, QPolygonF> SpecificWorker::read_laser(const Eigen::Vector2d &robot_tr, double robot_angle)
{
    QPolygonF poly_robot, poly_world;
    const float MAX_RDP_DEVIATION_mm  =  70;
    try
    {
        auto ldata = laser_proxy->getLaserData();

        // Simplify laser contour with Ramer-Douglas-Peucker
        poly_robot = ramer_douglas_peucker(ldata, MAX_RDP_DEVIATION_mm);
        qInfo() << __FUNCTION__ << "laser" << poly_robot.size();

                // compute poly_world
        poly_world.resize(poly_robot.size());
        for (auto &&[i, p] : poly_robot | iter::enumerate)
            poly_world[i] = e2q(from_robot_to_world(Eigen::Vector2d(p.x(), p.y()),  robot_tr, robot_angle));

        //poly_robot.pop_back();
        draw_laser(poly_robot);
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    return std::make_tuple(poly_robot, poly_world);
}
void SpecificWorker::draw_laser(const QPolygonF &poly_world) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer_robot->scene.removeItem(laser_polygon);

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer_robot->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly_world), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}
void SpecificWorker::draw_path(const std::vector<double> &path, const Eigen::Vector2d &tr_world, double rot)
{
    static std::vector<QGraphicsEllipseItem *> path_paint;
    static QString path_color = "DarkBlue";

    for(auto p : path_paint)
        viewer_robot->scene.removeItem(p);
    path_paint.clear();

    uint s = 100;
    std::vector<Eigen::Vector2d> qpath(path.size()/2);
    for(auto &&[i, p] : iter::chunked(path, 2) | iter::enumerate)
    {
        auto pw = from_robot_to_world(Eigen::Vector2d(p[0]*1000, p[1]*1000), tr_world, rot);
        path_paint.push_back(viewer_robot->scene.addEllipse(pw.x()-s/2, pw.y()-s/2, s , s, QPen(path_color), QBrush(QColor(path_color))));
        path_paint.back()->setZValue(30);
    }
}
SpecificWorker::Obstacles SpecificWorker::compute_laser_partitions(QPolygonF  &poly_robot)  //robot coordinates
{
    TPPLPartition partition;
    TPPLPoly poly_part;
    TPPLPolyList parts;

    poly_part.Init(poly_robot.size());
    poly_part.SetHole(false);
    for(auto &&[i, l] : iter::enumerate(poly_robot))
    {
        poly_part[i].x = l.x();
        poly_part[i].y = l.y();
    }
    poly_part.SetOrientation(TPPL_CCW);
    partition.ConvexPartition_HM(&poly_part, &parts);
    //partition.ConvexPartition_OPT(&poly_part, &parts);
    //int r = partition.Triangulate_OPT(&poly_part, &parts);
    //qInfo() << __FUNCTION__ << "Ok: " << r << "Num vertices:" << poly_part.GetNumPoints() << "Num res polys: " << parts.size();

    Obstacles obstacles(parts.size());
    for(auto &&[k, poly_res] : iter::enumerate(parts))
    {
        //color.setRgb(qrand() % 255, qrand() % 255, qrand() % 255);
        auto num_points = poly_res.GetNumPoints();

        // generate QPolygons for drawing
        QPolygonF poly_draw(num_points);
        std::generate(poly_draw.begin(), poly_draw.end(), [poly_res, k=0, robot = robot_polygon]() mutable
        {
            auto &p = poly_res.GetPoint(k++);
            return robot->mapToScene(QPointF(p.x, p.y));  //convert to world coordinates
        });

        // generate vector of <A,B,C> tuples
        Lines line(num_points);
        std::generate(line.begin(), line.end(),[poly_res, k=0, num_points]() mutable
        {
            float x1 = poly_res.GetPoint(k).x;
            float y1 = poly_res.GetPoint(k).y;
            float x2 = poly_res.GetPoint((++k) % num_points).x;
            float y2 = poly_res.GetPoint((k) % num_points).y;
            float norm = sqrt(pow(y1-y2, 2) + pow(x2-x1, 2));
            return std::make_tuple((y1 - y2)/norm, (x2 - x1)/norm, -((y1 - y2)*x1 + (x2 - x1)*y1)/norm);
        });
        obstacles[k] = std::make_tuple(line, poly_draw);
    }
    return obstacles;
}
void SpecificWorker::draw_partitions(const Obstacles &obstacles, const QColor &color, bool print)
{
    static std::vector<QGraphicsPolygonItem *> polys_ptr{};
    for (auto p: polys_ptr)
        viewer_robot->scene.removeItem(p);
    polys_ptr.clear();

    const QColor color_inside("LightBlue");
    const QColor color_outside("LightPink");
    for(auto &obs : obstacles)
    {
        bool inside = true;
        for (auto &[A, B, C] : std::get<Lines>(obs))
            inside = inside and C > 0; // since ABC were computed in the robot's coordinate frame

        if (inside)
            polys_ptr.push_back(viewer_robot->scene.addPolygon(std::get<QPolygonF>(obs), QPen(color_inside, 0), QBrush(color_inside)));
        else
            polys_ptr.push_back(viewer_robot->scene.addPolygon(std::get<QPolygonF>(obs), QPen(color, 0), QBrush(color_outside)));
    }
    if(print)
    {
        qInfo() << "--------- LINES ------------";
        for(auto &[ll, _] : obstacles)
        {
            for (auto &[a, b, c] : ll)
                qInfo() << a << b << c;
            qInfo() << "-----------------------";
        }
    }
}
QPolygonF SpecificWorker::ramer_douglas_peucker(const RoboCompLaser::TLaserData &ldata, double epsilon)
{
    if(ldata.size()<2)
    {
        qWarning() << __FUNCTION__ << "Not enough points to simplify";
        return QPolygonF();
    }
    std::vector<Point> pointList(ldata.size());
    for (auto &&[i, l] : ldata | iter::enumerate)
        pointList[i] = std::make_pair(l.dist * sin(l.angle), l.dist * cos(l.angle));
    std::vector<Point> pointListOut;
    ramer_douglas_peucker_rec(pointList, epsilon, pointListOut);
    QPolygonF poly(pointListOut.size());
    for (auto &&[i, p] : pointListOut | iter::enumerate)
        poly[i] = QPointF(p.first, p.second);
    return poly;
}
void SpecificWorker::ramer_douglas_peucker_rec(const vector<Point> &pointList, double epsilon, std::vector<Point> &out)
{
    // Find the point with the maximum distance from line between start and end
    auto line = Eigen::ParametrizedLine<float, 2>::Through(Eigen::Vector2f(pointList.front().first, pointList.front().second),
                                                           Eigen::Vector2f(pointList.back().first, pointList.back().second));
    auto max = std::max_element(pointList.begin()+1, pointList.end(), [line](auto &a, auto &b)
        { return line.distance(Eigen::Vector2f(a.first, a.second)) < line.distance(Eigen::Vector2f(b.first, b.second));});

    // If max distance is greater than epsilon, recursively simplify
    float dmax =  line.distance(Eigen::Vector2f((*max).first, (*max).second));
    if(dmax > epsilon)
    {
        // Recursive call
        std::vector<Point> recResults1;
        std::vector<Point> recResults2;
        std::vector<Point> firstLine(pointList.begin(), max + 1);
        std::vector<Point> lastLine(max, pointList.end());
        ramer_douglas_peucker_rec(firstLine, epsilon, recResults1);
        ramer_douglas_peucker_rec(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end() - 1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if (out.size() < 2)
        {
            qWarning() << __FUNCTION__ << "Problem assembling output";
            return;
        }
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList.front());
        out.push_back(pointList.back());
    }
}
//void SpecificWorker::spike_filter()
//{
//    // Filter out spikes. If the angle between two line segments is less than to the specified maximum angle
//    std::vector<QPointF> removed;
//    for(auto &&[k, ps] : iter::sliding_window(laser_poly,3) | iter::enumerate)
//        if( MAX_SPIKING_ANGLE_rads > acos(QVector2D::dotProduct( QVector2D(ps[0] - ps[1]).normalized(), QVector2D(ps[2] - ps[1]).normalized())))
//            removed.push_back(ps[1]);
//    for(auto &&r : removed)
//        laser_poly.erase(std::remove_if(laser_poly.begin(), laser_poly.end(), [r](auto &p) { return p == r; }), laser_poly.end());
//}
/////////////////////////////////////////////////////////////////
void SpecificWorker::new_target_slot(QPointF t)
{
    qInfo() << __FUNCTION__ << " Received new target at " << t;
    target.pos = t;
    target.active = true;
}
/***************************************************************/
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