//
// Created by pbustos on 3/04/22.
//

#include "MPC.h"
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>

namespace mpc
{
    casadi::Opti MPC::initialize_differential(const int N)
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

    std::optional<std::tuple<double, double, casadi::OptiSol, MPC::Balls>> MPC::minimize_balls(const Target &my_target,
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

    ////////////////////// AUX /////////////////////////////////////////////////
    std::vector<double> MPC::e2v(const Eigen::Vector2d &v)
    {
        return std::vector<double>{v.x(), v.y()};
    }
} // mpc