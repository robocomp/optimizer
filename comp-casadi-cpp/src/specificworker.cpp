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
#include <cppitertools/slice.hpp>
#include <cppitertools/chunked.hpp>
#include <cppitertools/sliding_window.hpp>
#include <ranges>

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
        auto [rp, rl] = viewer_robot->add_robot(ROBOT_LENGTH, ROBOT_LENGTH);
        laser_in_robot_polygon = rl;
        robot_polygon = rp;
        this->resize(700,450);

        opti = initialize_differential(consts.num_steps);

        // connect signal from mouse to set target
        connect(viewer_robot, &AbstractGraphicViewer::new_mouse_coordinates, [this](QPointF p) mutable
                            {
                                qInfo() << __FUNCTION__ << " Received new target at " << p;
                                target.set_pos(p);
                                target.set_active(true);
                                if(target.draw != nullptr) viewer_robot->scene.removeItem(target.draw);
                                target.draw = viewer_robot->scene.addEllipse(p.x()-50, p.y()-50, 100, 100, QPen(QColor("magenta")), QBrush(QColor("magenta")));
                                previous_values_of_solution.clear();
                                previous_control_of_solution.clear();
                            });

        //timer.setSingleShot(true);
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    qInfo() << "------------------------";
    //base
    auto [current_pose, current_pose_meters] = read_base();  //OJO CAMBIAR A realsensePoseEstimation

    // Bill
    //read_bill(current_pose);  // sets target at 1m from Bill

    // laser
    auto &&[laser_poly_robot, laser_poly_world, ldata] = read_laser(Eigen::Vector2d(current_pose.x, current_pose.z),
                                                                    current_pose.alpha,
                                                                    false);
    //auto laser_gaussians = fit_gaussians_to_laser(laser_poly_robot, current_pose, false);
    //std::vector<Gaussian> laser_gaussians;

    //auto obstacles = compute_laser_partitions(laser_poly_robot);
    //draw_partitions(obstacles, QColor("blue"));

    // check current target distance
    // double dist_to_target = (current_pose_meters(Eigen::seq(Eigen::fix<0>, Eigen::fix<1>))) - target.to_eigen_meters()).norm(); VERSION 3.4
    double dist_to_target = (Eigen::Vector2d(current_pose_meters.x(), current_pose_meters.y()) - target.to_eigen_meters()).norm();
    if( target.is_active() and dist_to_target > consts.min_dist_to_target)
    {
        //Target s_target = sub_target(target, laser_poly_robot, ldata, Eigen::Vector2d(current_pose.x, current_pose.z), current_pose.alpha);
        Target s_target = target;
        if (auto r = minimize_balls(s_target, current_pose_meters, ldata); r.has_value())
        {
            robot_polygon->setBrush(QColor("Blue"));
            auto [advance, rotation, solution, balls] = r.value();
            try
            {
                // move the robot
                move_robot(advance * gaussian(rotation), rotation);
                qInfo() << __FUNCTION__ << "Adv: " << advance << "Rot:" << rotation;

                // draw
                auto path = std::vector<double>(solution.value(pos));
                draw_path(path, Eigen::Vector2d(current_pose.x, current_pose.z), current_pose.alpha, balls);
            }
            catch (...)
            { std::cout << "No solution found" << std::endl; }
        }
        else // do something to avoid go blind
        {
            move_robot(0, 0);
            robot_polygon->setBrush(QColor("red"));
            previous_values_of_solution.clear();
            previous_control_of_solution.clear();
        }
    }
    else  // at  target
    {
        move_robot(0, 0);
        target.set_active(false);
        qInfo() << __FUNCTION__ << "Stopping";
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
SpecificWorker::Ball SpecificWorker::compute_free_ball(const Eigen::Vector2d &center, const std::vector<Eigen::Vector2d> &lpoints)
{
    // if ball already big enough return
    Eigen::Vector2d min_point = std::ranges::min(lpoints, [c=center](auto a, auto b){ return (a-c).norm() < (b-c).norm();});
    double initial_dist = (center-min_point).norm() - (consts.robot_radius/1000.f);
    //std::cout << __FUNCTION__ << center << " " << initial_dist << std::endl;
    if(initial_dist > consts.max_ball_radius)
         return std::make_tuple(center, 1, Eigen::Vector2d());

    // compute the distance to all laser points for center and center +- dx, center +- dy
    auto grad = [lpoints ](const Eigen::Vector2d &center) {
        auto dx = Eigen::Vector2d(0.1, 0.0);
        auto dy = Eigen::Vector2d(0.0, 0.1);
        auto min_dx_plus = std::ranges::min(lpoints, [c = center + dx](auto a, auto b) { return (a - c).norm() < (b - c).norm(); });
        auto min_dx_minus = std::ranges::min(lpoints, [c = center - dx](auto a, auto b) { return (a - c).norm() < (b - c).norm(); });
        auto min_dy_plus = std::ranges::min(lpoints, [c = center + dy](auto a, auto b) { return (a - c).norm() < (b - c).norm(); });
        auto min_dy_minus = std::ranges::min(lpoints, [c = center - dy](auto a, auto b) { return (a - c).norm() < (b - c).norm(); });
        // compute normalized gradient
        return Eigen::Vector2d((min_dx_plus - (center + dx)).norm() - (min_dx_minus - (center - dx)).norm(),
                               (min_dy_plus - (center + dy)).norm() - (min_dy_minus - (center - dy)).norm()).normalized();
    };

    // move  along gradient until the equality condition  max_dist(step*grad + c) == step + max_dist(c)  breaks
    auto new_center = center;
    float step = 0;
    float current_dist = initial_dist;

    while(fabs(current_dist - (step + initial_dist)) < 0.005)
    {
        step = step + 0.05;
        new_center = new_center + (grad(new_center) * step);
        min_point = std::ranges::min(lpoints, [c=new_center](auto a, auto b){ return (a-c).norm() < (b-c).norm();});
        current_dist = (new_center-min_point).norm() - (consts.robot_radius/1000.f);
    }

    // if redius les than robot_radius DO SOMETHING
//    if(current_dist < consts.robot_radius/1000.f)
//        return std::make_tuple(center, initial_dist, grad(center));

    return std::make_tuple(new_center, current_dist, grad(new_center));
}
std::optional<std::tuple<double, double, casadi::OptiSol, SpecificWorker::Balls>> SpecificWorker::minimize_balls(const Target &my_target,
                                                                                    const Eigen::Vector3d &current_pose_meters,
                                                                                    const RoboCompLaser::TLaserData &ldata)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto opti_local = opti.copy();
    casadi::Slice all;

    // target in robot RS
    auto target_robot = from_world_to_robot(my_target.to_eigen_meters(), Eigen::Vector2d(current_pose_meters.x(), current_pose_meters.y()), current_pose_meters(2));

    // Warm start
    if (previous_values_of_solution.empty())
    {
        previous_values_of_solution.resize(consts.num_steps+1);
        double landa = 1.0 / (target_robot.norm() / consts.num_steps);
        for (auto &&[i, step]: iter::range(0.0, 1.0, landa) | iter::enumerate)
        {
            auto paso = target_robot * step;
            previous_values_of_solution[3 * i] = paso.x();
            previous_values_of_solution[3 * i + 1] = paso.y();
            previous_values_of_solution[3 * i + 2] = 0.0;
        }
    }

    // initialize slack variables
    static std::vector<double> mu_vector(consts.num_steps, 1);
    std::vector<double> slack_init(consts.num_steps, 0.1);
    opti.set_initial(slack_vector, slack_init);

    // add free balls constraints
    std::vector<Eigen::Vector2d> lpoints(ldata.size());
    for(auto &&[i, l] : ldata | iter::enumerate)
        lpoints[i] = Eigen::Vector2d(l.dist*sin(l.angle)/1000.0, l.dist*cos(l.angle)/1000.0);
    Balls balls;
    balls.push_back(Ball{Eigen::Vector2d(0.0,0.0), 0.25, Eigen::Vector2d(0.2, 0.3)});  // first point on robot
    for (auto i: iter::range(0, consts.num_steps))
    {
        opti_local.set_initial(state(all, i), std::vector<double>{previous_values_of_solution[3 * i],
                                                                  previous_values_of_solution[3 * i + 1],
                                                                  previous_values_of_solution[3 * i + 2]});
        auto ball = compute_free_ball(Eigen::Vector2d(previous_values_of_solution[3*i],
                                                      previous_values_of_solution[3*i+1]), lpoints);

        auto &[center, radius, grad] = ball;
        double r = consts.robot_radius/1000.f;
        opti_local.subject_to(casadi::MX::sumsqr(pos(all, i) - e2v(center) + r) < (radius-0.01) + slack_vector(i));
        balls.push_back(ball);
    }

    //qInfo() << "Huma speed: " << target.get_velocity();
    // cost function
    double alfa = 1;
    auto sum_dist_target = opti_local.parameter();
    opti_local.set_value(sum_dist_target, 0.0);
    auto t = e2v(from_world_to_robot(my_target.to_eigen_meters(), Eigen::Vector2d(current_pose_meters.x(), current_pose_meters.y()), current_pose_meters(2)));
    for (auto k: iter::range(0, consts.num_steps+1))
        sum_dist_target += pow(alfa,k) * casadi::MX::sumsqr(pos(all, k) - t);

    // acceleration
//    auto sum_accel = opti_local.parameter();
//    opti_local.set_value(sum_accel, 0.0);
//    for (auto k: iter::range(previous_control_of_solution.size()/2))
//        sum_accel += casadi::MX::sumsqr(control(all, k) -  std::vector<double>{previous_control_of_solution[2*k], previous_control_of_solution[2*k+1]});

    // minimize step size weighting more the furthest poses
    double beta = 1.1;
    auto sum_dist_local = opti_local.parameter();
    opti_local.set_value(sum_dist_local, 0.0);
    for (auto k: iter::range(2, consts.num_steps+1))
        sum_dist_local += pow(beta,k) * casadi::MX::sumsqr(state(all, k-1) - state(all, k));

    // J
    opti_local.minimize( 1.0 * sum_dist_target +              /*casadi::MX::sumsqr(phi(-1)-target_angle_param)*10*/
                         /*0.01 * casadi::MX::sumsqr(rot) +*/
                         /*pow(1, consts.num_steps)**/casadi::MX::sumsqr(pos(all, consts.num_steps) - t) +
                         /*0.6 * sum_dist_local +*/
                        casadi::MX::dot(slack_vector, mu_vector)
                        /*sum_accel*/);

    // solve NLP ------
    try
    {
        auto solution = opti_local.solve();
        std::string retstat = solution.stats().at("return_status");
        if (retstat.compare("Solve_Succeeded") != 0)
        {
            std::cout << "NOT succeeded" << std::endl;
            move_robot(0, 0);  //slow down
            return {};
        }
        previous_values_of_solution = std::vector<double>(solution.value(state));
        previous_control_of_solution = std::vector<double>(solution.value(control));

        // print output -----
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
        auto advance = std::vector<double>(solution.value(adv)).front() * 1000;
        auto rotation = std::vector<double>(solution.value(rot)).front();
        //qInfo() << std::vector<double>(solution.value(rot));
        qInfo() << __FUNCTION__ << "Iterations:" << (int) solution.stats()["iter_count"];
        qInfo() << __FUNCTION__ << "Status:" << QString::fromStdString(solution.stats().at("return_status"));
        return std::make_tuple(advance, rotation, solution, balls);
    }
    catch (...)
    {
        std::cout << "No solution found" << std::endl;
        previous_values_of_solution.clear();
        previous_control_of_solution.clear();
        return {}; }
}
std::optional<std::tuple<double, double, casadi::OptiSol>> SpecificWorker::minimize_free(const Target &my_target,
                                                                                         const QPolygonF &poly_laser_robot,
                                                                                         const std::vector<Gaussian> &laser_gaussians,
                                                                                         const Eigen::Vector3d &current_pose_meters,
                                                                                         const Obstacles &obstacles)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto opti_local = opti.copy();
    casadi::Slice all;

    // target in robot RS
    auto target_robot = from_world_to_robot(my_target.to_eigen_meters(), Eigen::Vector2d(current_pose_meters.x(), current_pose_meters.y()), current_pose_meters(2));

    // Warm start
    if (previous_values_of_solution.empty())
    {
        previous_values_of_solution.resize(consts.num_steps+1);
        double landa = 1.0 / (target_robot.norm() / consts.num_steps);
        for (auto &&[i, step]: iter::range(0.0, 1.0, landa) | iter::enumerate)
        {
            auto paso = target_robot * step;
            previous_values_of_solution[3 * i] = paso.x();
            previous_values_of_solution[3 * i + 1] = paso.y();
            previous_values_of_solution[3 * i + 2] = 0.0;
        }
    }

    // set initial state from guess or previous iteration
    for (auto i: iter::range(1, consts.num_steps))
        opti_local.set_initial(state(all, i), std::vector<double>{previous_values_of_solution[3 * i],
                                                                  previous_values_of_solution[3 * i + 1],
                                                                  previous_values_of_solution[3 * i + 2]});

    // Constraints: Free regions. Select a convex region for each step
    for (auto i : iter::range(2, consts.num_steps))
    {
        for(auto &[line, poly] : obstacles)
            if( poly.containsPoint(QPointF(previous_values_of_solution[3 * i], previous_values_of_solution[3 * i + 1]), Qt::OddEvenFill))
            {
                for(auto &[A,B,C] : line)
                    opti_local.subject_to((pos(0, i)*A + pos(1, i)*B + C) > consts.min_line_dist);
                break;
            }
    }

    // initial values for state ---
    //auto target_angle_param = opti_local.parameter(1);
    //opti_local.set_value(target_angle_param, -M_PI/2.);

    // cost function
    auto t = e2v(target_robot);
    auto sum_dist_target = opti_local.parameter();
    opti_local.set_value(sum_dist_target, 0.0);
    for (auto k: iter::range(0, consts.num_steps+1))
        sum_dist_target += casadi::MX::sumsqr(pos(all, k) - t);

//    for (auto k: iter::range(2, consts.num_steps))
//        sum_dist_target += casadi::MX::sumsqr(pos(all, k-1) - pos(all, k));
    opti_local.minimize( 0.1 * sum_dist_target +                 /*casadi::MX::sumsqr(phi(-1)-target_angle_param)*10*/
                         0.1 * casadi::MX::sumsqr(rot) );
//                         casadi::MX::sumsqr(pos(all, consts.num_steps) - t));


    // solve NLP ------
    try
    {
        auto solution = opti_local.solve();
        std::string retstat = solution.stats().at("return_status");
        if (retstat.compare("Solve_Succeeded") != 0)
        {
            std::cout << "NOT succeeded" << std::endl;
            move_robot(0, 0);  //slow down
            return {};
        }
        previous_values_of_solution = std::vector<double>(solution.value(state));

        // print output -----
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
        auto advance = std::vector<double>(solution.value(adv)).front() * 1000;
        auto rotation = std::vector<double>(solution.value(rot)).front();
        //qInfo() << std::vector<double>(solution.value(rot));
        qInfo() << __FUNCTION__ << "Iterations:" << (int) solution.stats()["iter_count"];
        qInfo() << __FUNCTION__ << "Status:" << QString::fromStdString(solution.stats().at("return_status"));
        return std::make_tuple(advance, rotation, solution);
    }
    catch (...) { std::cout << "No solution found" << std::endl;  return {}; }
}
std::optional<std::tuple<double, double, casadi::OptiSol>> SpecificWorker::minimize(const Target &my_target,
                                                                                    const QPolygonF &poly_laser_robot,
                                                                                    const std::vector<Gaussian> &laser_gaussians,
                                                                                    const Eigen::Vector3d &current_pose_meters)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto opti_local = opti.copy();
    casadi::Slice all;

    // target in robot RS
    auto target_robot = from_world_to_robot(my_target.to_eigen_meters(), Eigen::Vector2d(current_pose_meters.x(), current_pose_meters.y()), current_pose_meters(2));

    // Warm start
    if (previous_values_of_solution.empty())
    {
        previous_values_of_solution.resize(consts.num_steps+1);
        double landa = 1.0 / (target_robot.norm() / consts.num_steps);
        for (auto &&[i, step]: iter::range(0.0, 1.0, landa) | iter::enumerate)
        {
            auto paso = target_robot * step;
            previous_values_of_solution[3 * i] = paso.x();
            previous_values_of_solution[3 * i + 1] = paso.y();
            previous_values_of_solution[3 * i + 2] = 0.0;
        }
    }

    // set initial state from guess or previous iteration
    for (auto i: iter::range(1, consts.num_steps))
        opti_local.set_initial(state(all, i), std::vector<double>{previous_values_of_solution[3 * i],
                                                                  previous_values_of_solution[3 * i + 1],
                                                                  previous_values_of_solution[3 * i + 2]});

    // Warm start
    if (previous_values_of_solution.empty())
    {
        previous_values_of_solution.resize(consts.num_steps+1);
        double landa = 1.0 / (target_robot.norm() / consts.num_steps);
        for (auto &&[i, step]: iter::range(0.0, 1.0, landa) | iter::enumerate)
        {
            auto paso = target_robot * step;
            previous_values_of_solution[3 * i] = paso.x();
            previous_values_of_solution[3 * i + 1] = paso.y();
            previous_values_of_solution[3 * i + 2] = 0.0;
        }
    }

    // set initial state from guess or previous iteration
    for (auto i: iter::range(1, consts.num_steps))
        opti_local.set_initial(state(all, i), std::vector<double>{previous_values_of_solution[3 * i],
                                                                  previous_values_of_solution[3 * i + 1],
                                                                  previous_values_of_solution[3 * i + 2]});

    // initial values for state ---
    //auto target_angle_param = opti_local.parameter(1);
    //opti_local.set_value(target_angle_param, -M_PI/2.);

    // cost function
    auto sum_dist_target = opti_local.parameter();
    opti_local.set_value(sum_dist_target, 0.0);
    auto t = e2v(from_world_to_robot(my_target.to_eigen_meters(), Eigen::Vector2d(current_pose_meters.x(), current_pose_meters.y()), current_pose_meters(2)));
    for (auto k: iter::range(2, consts.num_steps +1))
        sum_dist_target += casadi::MX::sumsqr(pos(all, k) - t);

    // promote equal distances
    auto sum_dist_local = opti_local.parameter();
    opti_local.set_value(sum_dist_local, 0.0);
    for (auto k: iter::range(2, consts.num_steps +1))
        sum_dist_local += casadi::MX::sumsqr(pos(all, k-1) - pos(all, k));

//    for (auto k: iter::range(2, consts.num_steps))
//        sum_dist_target += casadi::MX::sumsqr(pos(all, k-1) - pos(all, k));
    opti_local.minimize( sum_dist_target +                 /*casadi::MX::sumsqr(phi(-1)-target_angle_param)*10*/
                         0.1 * casadi::MX::sumsqr(rot) +
                         0.01 * sum_dist_local);
//                         casadi::MX::sumsqr(pos(all, consts.num_steps) - t));

    // obstacles
    double DW = 0.3;
    //double DL = 0.3;
    // std::vector<std::vector<double>> body_offset = {{0, 0}, {DW, 0}, { -DW, 0}};
    //std::vector<std::vector<double>> body_offset = {{0, 0}, {0, DW}, {0, -DW}};

    // add line between points gaussians
    for (const auto &lg : laser_gaussians)
        for (auto i: iter::range(1, consts.num_steps))
        {
          casadi::MX dist = casadi::MX::sumsqr(pos(all, i) - lg.mu);
          opti_local.subject_to(casadi::MX::exp(-0.5 * casadi::MX::sumsqr(casadi::MX::mtimes(lg.i_sigma, dist))) < consts.gauss_dist);
        }

    // add point gaussians
    double sigma = consts.point_sigma;
    for (const auto &p: poly_laser_robot)
        for (auto i: iter::range(1, consts.num_steps))
            opti_local.subject_to(casadi::MX::exp(-0.5 * casadi::MX::sumsqr(pos(all, i) - std::vector<double>{p.x() / 1000, p.y() / 1000}) / sigma) < consts.point_dist);

    // solve NLP ------
    try
    {
        auto solution = opti_local.solve();
        std::string retstat = solution.stats().at("return_status");
        if (retstat.compare("Solve_Succeeded") != 0)
        {
            std::cout << "NOT succeeded" << std::endl;
            move_robot(0, 0);  //slow down
            return {};
        }
        previous_values_of_solution = std::vector<double>(solution.value(state));

        // print output -----
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
        auto advance = std::vector<double>(solution.value(adv)).front() * 1000;
        auto rotation = std::vector<double>(solution.value(rot)).front();
        //qInfo() << std::vector<double>(solution.value(rot));
        qInfo() << __FUNCTION__ << "Iterations:" << (int) solution.stats()["iter_count"];
        qInfo() << __FUNCTION__ << "Status:" << QString::fromStdString(solution.stats().at("return_status"));
        return std::make_tuple(advance, rotation, solution);
    }
    catch (...) { std::cout << "No solution found" << std::endl;  return {}; }
}
// we define here only the fixed part of the problem
casadi::Opti SpecificWorker::initialize_differential(const int N)
{
    casadi::Slice all;
    auto opti = casadi::Opti();
    auto specific_options = casadi::Dict();
    auto generic_options = casadi::Dict();
    //specific_options["accept_after_max_steps"] = 100;
    specific_options["fixed_variable_treatment"] = "relax_bounds";
    //specific_options["print_time"] = 0;
    //specific_options["max_iter"] = 10000;
    specific_options["print_level"] = 0;
    specific_options["acceptable_tol"] = 1e-8;
    specific_options["acceptable_obj_change_tol"] = 1e-6;
    opti.solver("ipopt", generic_options, specific_options);


    // ---- state variables ---------
    state = opti.variable(3, N+1);
    pos = state(casadi::Slice(0,2), all);
    phi = state(2, all);

    // ---- inputs variables 2 adv and rot---------
    control = opti.variable(2, N);
    adv = control(0, all);
    rot = control(1, all);

    // Gap closing: dynamic constraints for differential robot: dx/dt = f(x, u)   3 x 2 * 2 x 1 -> 3 x 1
    auto integrate = [](casadi::MX x, casadi::MX u) { return casadi::MX::mtimes(
            casadi::MX::vertcat(std::vector<casadi::MX>{
                    casadi::MX::horzcat(std::vector<casadi::MX>{casadi::MX::sin(x(2)), 0.0}),
                    casadi::MX::horzcat(std::vector<casadi::MX>{casadi::MX::cos(x(2)), 0.0}),
                    casadi::MX::horzcat(std::vector<casadi::MX>{0.0,      1.0})}
            ), u);};
    double dt = 0.5;   // timer interval in secs
    for(const auto k : iter::range(N))  // loop over control intervals
    {
        auto k1 = integrate(state(all, k), control(all,k));
        auto k2 = integrate(state(all,k) + (dt/2)* k1 , control(all, k));
        auto k3 = integrate(state(all,k) + (dt/2)*k2, control(all, k));
        auto k4 = integrate(state(all,k) + k3, control(all, k));
        auto x_next = state(all, k) + dt / 6 * (k1 + 2*k2 + 2*k3 + k4);
        //auto x_next = state(all, k) + dt * integrate(state(all,k), control(all,k));
        opti.subject_to( state(all, k + 1) == x_next);  // close  the gaps
    }

    // control constraints -----------
    opti.subject_to(opti.bounded(consts.min_advance_value, adv, consts.max_advance_value));  // control is limited meters
    opti.subject_to(opti.bounded(-consts.max_rotation_value, rot, consts.max_rotation_value));         // control is limited

    // forward velocity constraints -----------
    //opti.subject_to(adv >= 0);

    // initial point constraints ------
//    auto initial_oparam = opti.parameter(3);
//    opti.set_value(initial_oparam, std::vector<double>{0.0, 0.0, 0.0});
    opti.subject_to(state(all, 0) == std::vector<double>{0.0, 0.0, 0.0});

    // slack vector declaration
    slack_vector = opti.variable(consts.num_steps);

    return opti;
}
std::vector<SpecificWorker::Gaussian>
SpecificWorker::fit_gaussians_to_laser(const QPolygonF &poly_laser_robot, const RoboCompGenericBase::TBaseState &bState, bool draw)
{
    static std::vector<QGraphicsItem *> balls;
    static std::vector<QGraphicsItem *> elipses;
    std::vector<Gaussian> gaussians;

    const float log_thr = log(consts.gauss_value_for_point);

    if(not balls.empty())
        for(auto &b : balls) viewer_robot->scene.removeItem(b);
    balls.clear();
    if(not elipses.empty())
        for(auto &b : elipses) viewer_robot->scene.removeItem(b);
    elipses.clear();

    if(true)
    {
        QPen tip_color_pen(QColor("DarkGreen"));
        QBrush tip_color_brush(QColor("DarkGreen"));
        for (const auto &l: poly_laser_robot)
        {
            QPointF pw(robot_polygon->mapToScene(QPointF(l.x(), l.y())));
            balls.push_back(viewer_robot->scene.addEllipse(QRectF(pw.x()-100, pw.y()-100, 200, 200), tip_color_pen, tip_color_brush));
        }
    }

    for(auto &&par : iter::sliding_window(poly_laser_robot, 2))
    {
        const auto p1 = q2e(par[0]/1000.0);
        const auto p2 = q2e(par[1]/1000.0);
        Eigen::Vector2f mean = (p1+p2)/2.0;
        // compute 1D variance
        Eigen::Vector2f dist = p1-mean;
        float sx = -dist.squaredNorm()/(2.0*log_thr);
        float sy;
        if(sx < 0) sy = sx; else sy = sx/10.0;
        Eigen::Matrix2f S;
        S << sx, 0.0, 0.0, sy;
        Eigen::Vector2f dir = p2-p1;
        float ang = atan2(dir.y(), dir.x());
        Eigen::Matrix2f R;
        R << cos(ang), -sin(ang), sin(ang), cos(ang);  // CCW
        // rotate covariance
        auto SR = R * S * R.transpose();

        // draw, compute eigenvectors
        if(draw)
        {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> e_solver(SR);
            auto a_values = e_solver.eigenvalues();
            auto a_vectors = e_solver.eigenvectors();
            float d_angle = atan2(a_vectors.col(1)(1), a_vectors.col(1)(0));  // largest eigenvector
            auto el_ptr = viewer_robot->scene.addEllipse(QRectF(-fabs(a_values(1) * 1000 * 1.5), -fabs(a_values(0) * 1000 * 1.5),
                                                                3 * fabs(a_values(1) * 1000), 3 * fabs(a_values(0) * 1000)), QPen(QColor("blue"), 50));
            QPointF mean_w(robot_polygon->mapToScene(QPointF(mean.x() * 1000, mean.y() * 1000)));
            el_ptr->setPos(mean_w);
            el_ptr->setRotation(qRadiansToDegrees(d_angle) + qRadiansToDegrees(bState.alpha));
            elipses.push_back(el_ptr);
        }

        // build return type
        Eigen::MatrixX2f iS = SR.inverse();
        Gaussian g;
        g.mu = std::vector<double>{mean.x(), mean.y()};
        g.i_sigma = casadi::MX::vertcat({casadi::MX::horzcat({iS(0,0), iS(0,1)}),
                                         casadi::MX::horzcat({iS(1,0), iS(1,1)})});
        gaussians.push_back(g);
    }
    return gaussians;
}
SpecificWorker::Lines SpecificWorker::get_cube_lines(const Eigen::Vector2d &robot_tr, double robot_angle)
{
    auto obs_points = std::vector<Eigen::Vector2d>{Eigen::Vector2d(-500, 500), Eigen::Vector2d(500, 500), Eigen::Vector2d(500, -500), Eigen::Vector2d(-500, -500), Eigen::Vector2d(-500, 500)};
    std::vector<Eigen::Vector2d> obs_points_in_robotSR;

    int ip = 0;
    for( auto &p: obs_points)
    {
        obs_points_in_robotSR.push_back(from_world_to_robot(p, robot_tr, robot_angle));
        std::cout<<obs_points_in_robotSR[ip].x()<<" "<<obs_points_in_robotSR[ip].y()<<std::endl;
        ip++;
    }

    SpecificWorker::Lines obs_lines;
    int nPoints = obs_points.size();
    for(auto i: iter::range(0, nPoints-1))
    {
        auto p1 = obs_points_in_robotSR[i]/1000.;
        auto p2 = obs_points_in_robotSR[(i+1)]/1000.;
        float norm = sqrt(pow((p1.y()-p2.y()), 2) + pow((p1.x()-p2.x()), 2));
        float A = (p2.y() - p1.y())/norm;
        float B = (p1.x() - p2.x())/norm;
        float C = -(A*p1.x() + B*p1.y())/norm;
         std::cout<<A<<" "<<B<<" "<<C<<std::endl;
        obs_lines.push_back({A, B, C});
    }
    return obs_lines;

}
SpecificWorker::Target SpecificWorker::sub_target( const Target &target,
                                                   const QPolygonF &poly,
                                                   const RoboCompLaser::TLaserData &ldata,
                                                   const Eigen::Vector2d &robot_tr_mm,
                                                   double robot_ang
                                                  )
{
    static QGraphicsRectItem *former_draw = nullptr;
    if(former_draw != nullptr)
        viewer_robot->scene.removeItem(former_draw);

    // if target inside laser_polygon return
    auto target_in_robot = from_world_to_robot( target.to_eigen(), robot_tr_mm, robot_ang);
    if(poly.containsPoint(QPointF(target_in_robot.x(), target_in_robot.y()), Qt::WindingFill))
        return target;

    // locate openings
    std::vector<float> derivatives(ldata.size());
    derivatives[0] = 0;
    for (const auto &&[k, l]: iter::sliding_window(ldata, 2) | iter::enumerate)
        derivatives[k + 1] = l[1].dist - l[0].dist;

    // locate peaks
    std::vector<Eigen::Vector2d> peaks;
    for (const auto &&[k, der]: iter::enumerate(derivatives))
        if (fabs(der) > consts.peak_threshold)
        {
            RoboCompLaser::TData l;
            l = ldata.at(k - 1);
            auto one = Eigen::Vector2d(l.dist * sin(l.angle), l.dist * cos(l.angle));
            l = ldata.at(k);
            auto two = Eigen::Vector2d(l.dist * sin(l.angle), l.dist * cos(l.angle));
            peaks.push_back((one + two) / 2.f);
//            Eigen::Vector2d m;
//            if(one.norm() < two.norm()) m = one; else m = two;
//            peaks.push_back(m);
        }

    // if no peaks return
    if(peaks.empty())
        return target;

    // select closest opening to robot
    auto min = std::ranges::min_element(peaks, [target_in_robot](auto a, auto b)
                    { return (target_in_robot-a).norm() < (target_in_robot-b).norm();});
    auto candidate = *min;

    // if too close to subtarget ignore
    if( candidate.norm() < 200)
        return target;

    // if target closer that subtatget ignore
    if(target_in_robot.norm() < candidate.norm())
        return target;

    // return target
    Target t;
    t.set_active(true);
    auto pos = from_robot_to_world(*min, robot_tr_mm, robot_ang);
    t.set_pos(QPointF(pos.x(), pos.y()));
    former_draw = viewer_robot->scene.addRect(t.get_pos().x()-100, t.get_pos().y()-100, 200, 200, QPen(QColor("blue")), QBrush(QColor("blue")));

    return t;
}
//////////////////////////////////////// AUX /////////////////////////////////////////////////
std::optional<Eigen::Vector2d> SpecificWorker::find_inside_target(const Eigen::Vector2d &target_in_robot,
                                                                  const RoboCompLaser::TLaserData &ldata,
                                                                  const QPolygonF &poly)
{
    if(poly.containsPoint(e2q(target_in_robot), Qt::WindingFill))
        return {};
    std::vector<std::tuple<Eigen::Vector2d, double, Eigen::ParametrizedLine<double, 2>>> candidates;
    for(auto &&par : iter::sliding_window(ldata, 2))
    {
        Eigen::Vector2d p1(par[0].dist*sin(par[0].angle), par[0].dist*cos(par[0].angle));
        Eigen::Vector2d p2(par[1].dist*sin(par[1].angle), par[1].dist*cos(par[1].angle));
        double dist = (p1-p2).norm();
        auto line = Eigen::ParametrizedLine<double, 2>::Through(p1, p2);
        auto proj = line.projection(target_in_robot);
        if(((proj-p1).norm() > dist) or ((proj-p2).norm() > dist))
        {
            if((proj-p1).norm()>(proj-p2).norm())
                proj = p2;
            else
                proj = p1;
        }
        candidates.push_back(std::make_tuple(proj, (proj-target_in_robot).norm()/*line.squaredDistance(target_in_robot)*/, Eigen::ParametrizedLine<double, 2>::Through(target_in_robot, proj)));
    }
    qInfo() << __FUNCTION__ << candidates.empty() << candidates.size();
    if(not candidates.empty())
    {
        auto min_it = std::ranges::min_element(candidates, [](auto a, auto b){return std::get<1>(a) < std::get<1>(b);});
        Eigen::Vector2d new_t = std::get<2>(*min_it).pointAt(std::get<double>(*min_it) + 400);
        return new_t;
    }
    else
        return {};
}
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
std::tuple<QPolygonF, QPolygonF, RoboCompLaser::TLaserData> SpecificWorker::read_laser(const Eigen::Vector2d &robot_tr,
                                                                                       double robot_angle,
                                                                                       bool noise)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::normal_distribution<double> normal_dist(0.0, consts.laser_noise_sigma);

    QPolygonF poly_robot, poly_world;
    RoboCompLaser::TLaserData ldata;
    try
    {
        ldata = laser_proxy->getLaserData();

        // add radial noise la ldata
        if(noise)
            for(auto &l : ldata)
                l.dist += normal_dist(mt);

        // get n random angles to apply hard noise on them
        static std::uniform_int_distribution<int> unif_dist(0, ldata.size());
        for(int i: iter::range(consts.num_lidar_affected_rays_by_hard_noise))
            ldata[unif_dist(mt)].dist /= 5;

        // Simplify laser contour with Ramer-Douglas-Peucker
        //poly_robot = ramer_douglas_peucker(ldata, consts.max_RDP_deviation);
        // Build raw polygon
        for(auto &l : ldata)
            poly_robot << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));

            // compute poly_world
        poly_world.resize(poly_robot.size());
        for (auto &&[i, p] : poly_robot | iter::enumerate)
            poly_world[i] = e2q(from_robot_to_world(Eigen::Vector2d(p.x(), p.y()),  robot_tr, robot_angle));

        poly_robot << QPointF(0, 0);
        draw_laser(poly_robot);
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    return std::make_tuple(poly_robot, poly_world, ldata);
}
std::tuple<RoboCompGenericBase::TBaseState, Eigen::Vector3d> SpecificWorker::read_base()
{
    RoboCompGenericBase::TBaseState current_pose;
    Eigen::Vector3d current_pose_meters;
    try
    {
        //currentPose = self.omnirobot_proxy.getBaseState()
        differentialrobot_proxy->getBaseState(current_pose);
        current_pose_meters[0] = current_pose.x / 1000;
        current_pose_meters[1] = current_pose.z / 1000;
        current_pose_meters[2] = current_pose.alpha;
        robot_polygon->setRotation(current_pose.alpha * 180 / M_PI);
        robot_polygon->setPos(current_pose.x, current_pose.z);
    }
    catch(const Ice::Exception &e){ qInfo() << "Error connecting to base"; std::cout << e.what() << std::endl;}
    return std::make_tuple(current_pose, current_pose_meters);
}
bool SpecificWorker::read_bill(const RoboCompGenericBase::TBaseState &bState)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::normal_distribution<double> normal_dist(0.0, consts.target_noise_sigma);

    try
    {
        auto pose = billcoppelia_proxy->getPose();
        QLineF r_to_target(QPointF(pose.x, pose.y), QPointF(bState.x, bState.z));
        auto t = r_to_target.pointAt(700.0 / r_to_target.length());
        target.set_pos(t + QPointF(normal_dist(mt), normal_dist(mt)));              // Adding noise to target
        if(target.draw != nullptr) viewer_robot->scene.removeItem(target.draw);
        target.set_active(true);
        target.draw = viewer_robot->scene.addEllipse(target.get_pos().x()-100, target.get_pos().y()-100, 200, 200, QPen(QColor("magenta")), QBrush(QColor("magenta")));
    }
    catch(const Ice::Exception &e)
    {
        qInfo() << "Error connecting to Bill Coppelia";
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
}
SpecificWorker::Obstacles SpecificWorker::compute_laser_partitions(QPolygonF  &poly_robot)
{
    TPPLPartition partition;
    TPPLPoly poly_part;
    TPPLPolyList parts;

    poly_part.Init(poly_robot.size());
    poly_part.SetHole(false);
    for(auto &&[i, l] : iter::enumerate(poly_robot))
    {
        poly_part[i].x = l.x();   // to meters
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
            float x1 = poly_res.GetPoint(k).x /1000;
            float y1 = poly_res.GetPoint(k).y /1000;
            float x2 = poly_res.GetPoint((++k) % num_points).x /1000;
            float y2 = poly_res.GetPoint((k) % num_points).y /1000;
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
float SpecificWorker::gaussian(float x)
{
    const double xset = consts.xset_gaussian;
    const double yset = consts.yset_gaussian;
    const double s = -xset*xset/log(yset);
    return exp(-x*x/s);
}
void SpecificWorker::new_target_slot(QPointF t)
{
    qInfo() << __FUNCTION__ << " Received new target at " << t;
    target.set_pos(t);
    target.set_active(true);
}
////////////////////////////// DRAW ///////////////////////////////////
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
void SpecificWorker::draw_path(const std::vector<double> &path, const Eigen::Vector2d &tr_world, double my_rot, const Balls &balls)
{
    static std::vector<QGraphicsItem *> path_paint, ball_paint, ball_grads, ball_centers;
    static QString path_color = "Orange";
    static QString ball_color = "LightBlue";

    for(auto p : path_paint)
        viewer_robot->scene.removeItem(p);
    path_paint.clear();
    for(auto p : ball_paint)
        viewer_robot->scene.removeItem(p);
    ball_paint.clear();
    for(auto p : ball_grads)
        viewer_robot->scene.removeItem(p);
    ball_grads.clear();
    for(auto p : ball_centers)
        viewer_robot->scene.removeItem(p);
    ball_centers.clear();

    uint s = 100;
    std::vector<Eigen::Vector2d> qpath(path.size()/2);
    std::vector<Eigen::Vector2d> path_centers;
    for(auto &&[i, p] : iter::chunked(path, 2) | iter::enumerate)
    {
        auto pw = from_robot_to_world(Eigen::Vector2d(p[0]*1000, p[1]*1000), tr_world, my_rot);  // in mm
        path_centers.push_back(pw);
        path_paint.push_back(viewer_robot->scene.addEllipse(pw.x()-s/2, pw.y()-s/2, s , s, QPen(path_color), QBrush(QColor(path_color))));
        path_paint.back()->setZValue(30);
    }
    uint i=0;
    auto balls_temp = balls;
    balls_temp.erase(balls_temp.begin());
    for(auto &[center, r, grad] : balls_temp)
    {
        auto bc = from_robot_to_world(center*1000, tr_world, my_rot);
        auto nr = r * 1000;
        ball_paint.push_back(viewer_robot->scene.addEllipse(bc.x()-nr, bc.y()-nr, nr*2 , nr*2, QPen(QBrush("DarkBlue"),15), QBrush(QColor(ball_color))));
        ball_paint.back()->setZValue(15);
        ball_paint.back()->setOpacity(0.07);

        // grads
        //auto p2 = from_robot_to_world(center*1000 + grad*100, tr_world, my_rot);
        //ball_grads.push_back(viewer_robot->scene.addLine(bc.x(), bc.y(), p2.x(), p2.y(), QPen(QBrush("Magenta"), 20)));
        if(i < path_centers.size())
        {
            ball_grads.push_back(viewer_robot->scene.addLine(path_centers[i].x(), path_centers[i].y(), bc.x(), bc.y(), QPen(QBrush("Magenta"), 20)));
            i++;
        }
        //ball_grads.back()->setZValue(30);

        // centers
        //ball_centers.push_back(viewer_robot->scene.addEllipse(bc.x()-20, bc.y()-20, 40, 40, QPen(QColor("Magenta")), QBrush(QColor("Magenta"))));
        //ball_centers.back()->setZValue(30);
    }

}
void SpecificWorker::draw_target(const Target &target)
{
    static QGraphicsItem *target_rep = nullptr;
    if (target_rep != nullptr)
        viewer_robot->scene.removeItem(target_rep);
    target_rep = viewer_robot->scene.addEllipse(target.get_pos().x()-100./2, target.get_pos().y()-100./2, 100. , 100., QPen("DarkRed"), QBrush(QColor("DarkRed")));
}
//////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
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

//void SpecificWorker::spike_filter_spatial()
//{
//    // Filter out spikes. If the angle between two line segments is less than to the specified maximum angle
//    std::vector<QPointF> removed;
//    for(auto &&[k, ps] : iter::sliding_window(laser_poly,3) | iter::enumerate)
//        if( MAX_SPIKING_ANGLE_rads > acos(QVector2D::dotProduct( QVector2D(ps[0] - ps[1]).normalized(), QVector2D(ps[2] - ps[1]).normalized())))
//            removed.push_back(ps[1]);
//    for(auto &&r : removed)
//        laser_poly.erase(std::remove_if(laser_poly.begin(), laser_poly.end(), [r](auto &p) { return p == r; }), laser_poly.end());
//}


// project target on closest laser perimeter point
//     if(auto t = find_inside_target(from_world_to_robot( target.to_eigen(),
//                                                         Eigen::Vector2d(current_pose.x, current_pose.z), current_pose.alpha),
//                                                         ldata,
//                                                         laser_poly_robot); t.has_value())
//         target.pos = e2q(from_robot_to_world(t.value(), Eigen::Vector2d(current_pose.x, current_pose.z), current_pose.alpha));
//