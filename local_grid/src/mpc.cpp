//
// Created by pbustos on 3/04/22.
//

#include "mpc.h"
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/chunked.hpp>
#include <QGraphicsScene>
#include <QPen>
#include <QBrush>

namespace mpc
{
    casadi::Opti MPC::initialize_differential(const int N)
    {
        consts.num_steps = N;
        casadi::Slice all;
        this->opti = casadi::Opti();
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
        // std::cout<<adv.size()<<std::endl;
        // exit(0);
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
        for(const auto k : iter::range(N-1))  // acceleration constraints
        {
            auto v1 = control(0,k);
            auto v2 = control(0,k+1);
            auto acc = (v2-v1)/dt;
            //auto x_next = state(all, k) + dt * integrate(state(all,k), control(all,k));
            opti.subject_to(opti.bounded(-3.19, acc, 3.19)); 
            auto w1 = control(1,k);
            auto w2 = control(1,k+1);
            auto ang_acc = (w2-w1)/dt;
            opti.subject_to(opti.bounded(-0.5, ang_acc, 0.5)); 
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
    };

    MPC::Result MPC::minimize_balls_path(const std::vector<Eigen::Vector2d> &path,
                                         const Eigen::Vector3d &current_pose_meters,
                                         const RoboCompLaser::TLaserData &ldata)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        auto opti_local = this->opti.copy();
        casadi::Slice all;

        // target in robot RS
        auto target_robot = path.at(consts.num_steps-1);
        std::cout<<"2"<<std::endl;

        // Warm start
        if (previous_values_of_solution.empty())
        {
            std::cout<<"3"<<std::endl;

            previous_values_of_solution.resize(consts.num_steps+1);
            std::cout<<"4"<<std::endl;
            double landa = 1.0 / (target_robot.norm() / consts.num_steps);
            for (auto &&[i, step]: iter::range(0.0, 1.0, landa) | iter::enumerate)
            {
                auto paso = target_robot * step;
                previous_values_of_solution[3 * i] = paso.x();
                previous_values_of_solution[3 * i + 1] = paso.y();
                previous_values_of_solution[3 * i + 2] = 0.0;
               
            }
            std::cout<<"5"<<std::endl;
        } 
        


        // initialize slack variables
        std::cout<<"6"<<std::endl;
        
        std::vector<double> mu_vector(consts.num_steps, 1.0);
        std::cout<<"7"<<std::endl;
        std::cout<<"mu_vector size: "<<mu_vector.size()<<std::endl;
        // exit(0);
        std::vector<double> slack_init(consts.num_steps, 0.1);
        std::cout<<"8"<<std::endl;
        opti_local.set_initial(slack_vector, slack_init);
        

        // add free balls constraints
        std::vector<Eigen::Vector2d> lpoints(ldata.size());
        for(auto &&[i, l] : ldata | iter::enumerate)
            lpoints[i] = Eigen::Vector2d(l.dist*sin(l.angle)/1000.0, l.dist*cos(l.angle)/1000.0);
        Balls balls;
        balls.push_back(Ball{Eigen::Vector2d(0.0,0.0), 0.25, Eigen::Vector2d(0.2, 0.3)});  // first point on robot
        for (auto i: iter::range(0u, consts.num_steps))
        {
            // opti_local.set_initial(state(all, i), std::vector<double>{previous_values_of_solution[3 * i],
            //                                                           previous_values_of_solution[3 * i + 1],
            //                                                           previous_values_of_solution[3 * i + 2]});
            auto ball = compute_free_ball(Eigen::Vector2d(previous_values_of_solution[3*i],
                                                          previous_values_of_solution[3*i+1]), lpoints);

            auto &[center, radius, grad] = ball;
            double r = consts.robot_radius/1000.f;
            opti_local.subject_to(casadi::MX::sumsqr(pos(all, i) - e2v(center) + r) < (radius-0.01) + slack_vector(i));
            balls.push_back(ball);
        }

        // cost function
        double alfa = 1.1;
        auto sum_dist_target = opti_local.parameter();
        opti_local.set_value(sum_dist_target, 0.0);
        auto t = e2v(target_robot);
        for (auto k: iter::range(consts.num_steps+1))
            sum_dist_target += pow(alfa,k) * casadi::MX::sumsqr(pos(all, k) - t);

//        // minimize step size weighting more the furthest poses
//        double beta = 1.1;
//        auto sum_dist_local = opti_local.parameter();
//        opti_local.set_value(sum_dist_local, 0.0);
//        for (auto k: iter::range(2, consts.num_steps+1))
//            sum_dist_local += pow(beta,k) * casadi::MX::sumsqr(state(all, k-1) - state(all, k));

        // minimze distance to each element of path
        auto sum_dist_path = opti_local.parameter();
        opti_local.set_value(sum_dist_path, 0.0);
        for (auto k: iter::range(consts.num_steps))
            sum_dist_path += casadi::MX::sumsqr(pos(all, k) - e2v(path[k]));

        // minimze sum of rotations
        auto sum_rot = opti_local.parameter();
        opti_local.set_value(sum_rot, 0.0);
        for (auto k: iter::range(consts.num_steps))
            sum_rot += casadi::MX::sumsqr(rot(k));

        //  J
//        opti_local.minimize( sum_dist_path + 0.1*sum_dist_target + sum_rot +
//                             casadi::MX::sumsqr(pos(all, consts.num_steps) - t) +
//                             casadi::MX::dot(slack_vector, mu_vector));

        opti_local.minimize( sum_dist_path  +
                             casadi::MX::sumsqr(pos(all, consts.num_steps) - t) +
                             casadi::MX::dot(slack_vector, mu_vector));
        // solve NLP ------
        try
        {
            auto solution = opti_local.solve();
            std::string retstat = solution.stats().at("return_status");
            if (retstat.compare("Solve_Succeeded") != 0)
            {
                std::cout << "NOT succeeded" << std::endl;
                //move_robot(0, 0);  //slow down
                return {};
            }
            previous_values_of_solution = std::vector<double>(solution.value(state));
            previous_control_of_solution = std::vector<double>(solution.value(control));

            // print output -----
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
            //auto advance = std::vector<double>(solution.value(adv)).front() * 1000;
            //auto rotation = std::vector<double>(solution.value(rot)).front();
            auto advance = std::vector<double>(solution.value(adv)).at(1) * 1000;
            auto rotation = std::vector<double>(solution.value(rot)).at(1);

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

    MPC::Ball MPC::compute_free_ball(const Eigen::Vector2d &center, const std::vector<Eigen::Vector2d> &lpoints)
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

    MPC::Result MPC::update( float adv_prev, double slack_weight, std::vector<Eigen::Vector2d> near_obstacles, const std::vector<Eigen::Vector2f> &path_robot, QGraphicsPolygonItem *robot_polygon, QGraphicsScene *scene)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // transform path to meters
        std::vector<Eigen::Vector2f> path_robot_meters(path_robot.size());
        for(const auto &[i, p] : path_robot | iter::enumerate)
        {
            path_robot_meters[i] = p / 1000.f;
        }
        
        // target in robot RS in meters
        auto target_robot = path_robot_meters.back();

        auto opti_local = this->opti.copy();
        casadi::Slice all;

        std::vector<double> mu_vector(consts.num_steps, slack_weight);
        std::vector<double> slack_init(consts.num_steps, 0.5);
        opti_local.set_initial(slack_vector, slack_init);

        double r = consts.robot_radius/1000.f;

        Balls balls;
        balls.push_back(Ball{Eigen::Vector2d(0.0,0.0), r, Eigen::Vector2d(0.2, 0.3)});

        // for (auto k: iter::range(near_obstacles.size()))
        // {
        //     for (auto i: iter::range(consts.num_steps)) // obstacle avoidance constraints
        //     {
        //         auto dist = casadi::MX::sqrt( casadi::MX::pow((pos(0,i) - near_obstacles[k][0]/1000.f),2) + casadi::MX::pow((pos(1,i) - near_obstacles[k][1]/1000.f),2) );
        //         opti_local.subject_to(dist >= (.300 + .075) );

        //     }
        // }

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
        // auto acc = (control(0,0)-adv_prev)/0.5;
        // opti_local.subject_to((control(0,0)-adv_prev)/0.5 <= 3.19); 
        // opti_local.subject_to((control(0,0)-adv_prev)/0.5 >= -3.19); 

        Eigen::Vector2d min_point;
        Eigen::Vector2d center;        

        if(near_obstacles.size()>0)
        {
            for (auto i: iter::range(consts.num_steps))
            {
                // opti_local.set_initial(state(all, i), std::vector<double>{previous_values_of_solution[3 * i],
                //                                                           previous_values_of_solution[3 * i + 1],
                //                                                           previous_values_of_solution[3 * i + 2]});

            
                //------------------------------ COMPUTE FREE BALLS ------------------------------
                center = Eigen::Vector2d(previous_values_of_solution[3*i], previous_values_of_solution[3*i+1]); // in meters
                min_point = std::ranges::min(near_obstacles, [c=center](auto a, auto b){ return (a-c).norm() < (b-c).norm();});
                double initial_dist = (center-min_point).norm()- (consts.robot_radius/1000.f);
                if(initial_dist > consts.max_ball_radius) //consts.max_ball_radius)
                {
                    std::cout<<"Initial dist: "<<initial_dist<<std::endl;
                    auto ball = std::make_tuple(center, consts.max_ball_radius, Eigen::Vector2d());
                    auto &[center, radius, gradient] = ball;
                    balls.push_back(ball);
                    opti_local.subject_to(casadi::MX::sqrt(casadi::MX::sumsqr(pos(all, i) - e2v(center) + r)) < (radius-.001)+ slack_vector(i));

                }
                else{
                    auto grad = [near_obstacles ](const Eigen::Vector2d center) {
                        auto dx = Eigen::Vector2d(0.1, 0.0);
                        auto dy = Eigen::Vector2d(0.0, 0.1);
                        auto min_dx_plus = std::ranges::min(near_obstacles, [c = center + dx](auto a, auto b) { return (a - c).norm() < (b - c).norm(); });
                        auto min_dx_minus = std::ranges::min(near_obstacles, [c = center - dx](auto a, auto b) { return (a - c).norm() < (b - c).norm(); });
                        auto min_dy_plus = std::ranges::min(near_obstacles, [c = center + dy](auto a, auto b) { return (a - c).norm() < (b - c).norm(); });
                        auto min_dy_minus = std::ranges::min(near_obstacles, [c = center - dy](auto a, auto b) { return (a - c).norm() < (b - c).norm(); });
                        // compute normalized gradient
                        return Eigen::Vector2d((min_dx_plus - (center + dx)).norm() - (min_dx_minus - (center - dx)).norm(),
                                            (min_dy_plus - (center + dy)).norm() - (min_dy_minus - (center - dy)).norm()).normalized();
                    };
                    auto new_center = center;
                    double step = 0;
                    double current_dist = initial_dist;


                    while(fabs(current_dist - (step + initial_dist)) < 0.005 )
                    {
                        step = step + 0.05;
                        new_center = new_center + (grad(new_center) * step);
                        min_point = std::ranges::min(near_obstacles, [c=new_center](auto a, auto b){ return (a-c).norm() < (b-c).norm();});
                        current_dist = (new_center-min_point).norm()- (consts.robot_radius/1000.f);
                        
                    }
                    // current_dist = std::max(0.32,(current_dist));
                    
                    // std::cout<<"Radius: "<<current_dist<<std::endl;
                    auto ball = std::make_tuple(new_center, current_dist, grad(new_center));
                    auto &[center, radius, gradient] = ball;
                    balls.push_back(ball);
                    opti_local.subject_to(casadi::MX::sqrt(casadi::MX::sumsqr(pos(all, i) - e2v(center) + r)) < (radius-.01)+ slack_vector(i));
                }
            }
        }


        // cost function
        double alfa = 1.001;
        auto sum_dist_target = opti_local.parameter();
        opti_local.set_value(sum_dist_target, 0.0);
        //target
        auto t = e2v(target_robot.cast<double>());

        for (auto k: iter::range(consts.num_steps+1))
            sum_dist_target += pow(alfa,k) * casadi::MX::sumsqr(pos(all, k) - t);

//        // minimize step size weighting more the furthest poses
//        double beta = 1.1;
//        auto sum_dist_local = opti_local.parameter();
//        opti_local.set_value(sum_dist_local, 0.0);
//        for (auto k: iter::range(2, consts.num_steps+1))
//            sum_dist_local += pow(beta,k) * casadi::MX::sumsqr(state(all, k-1) - state(all, k));

        // minimze distance to each element of path
        double beta = 1.5;
        auto sum_dist_path = opti_local.parameter();
        opti_local.set_value(sum_dist_path, 0.0);
        for (auto k: iter::range(consts.num_steps))
            sum_dist_path += pow(beta,k)*casadi::MX::sumsqr(pos(all, k) - e2v(path_robot_meters[k].cast<double>()));

        // minimze sum of rotations
        auto sum_rot = opti_local.parameter();
        opti_local.set_value(sum_rot, 0.0);
        for (auto k: iter::range(consts.num_steps))
            sum_rot += casadi::MX::sumsqr(rot(k));

        // minimze slack sums
        auto sum_slack = opti_local.parameter();
        opti_local.set_value(sum_slack, 0.0);
        for (auto k: iter::range(consts.num_steps))
            sum_slack += casadi::MX::sumsqr(slack_vector(k));

        //  J
    //    opti_local.minimize( sum_dist_path + 0.1*sum_dist_target + sum_rot +
    //                         casadi::MX::sumsqr(pos(all, consts.num_steps) - t) +
    //                         casadi::MX::dot(slack_vector, mu_vector));


        opti_local.minimize( 1*sum_dist_path  + 0.00*sum_dist_target +
                             0.0*casadi::MX::sumsqr(pos(all, consts.num_steps) - t) + 0.0*sum_rot+ slack_weight*sum_slack);
                            //  casadi::MX::dot(slack_vector, mu_vector));
        // opti_local.minimize( sum_dist_path  + 0*sum_dist_target + 0.01*casadi::MX::sumsqr(pos(all, consts.num_steps) - t) + 
        //                     0.001*sum_rot +
        //                      casadi::MX::dot(slack_vector, mu_vector)); // + casadi::MX::sumsqr(control(0,consts.num_steps-1)-0));

        // solve NLP ------
        try
        {
            auto solution = opti_local.solve();
            std::string retstat = solution.stats().at("return_status");
            if (retstat.compare("Solve_Succeeded") != 0)
            {
                std::cout << "NOT succeeded" << std::endl;
                //move_robot(0, 0);  //slow down
                return {};
            }
            previous_values_of_solution = std::vector<double>(solution.value(state));
            previous_control_of_solution = std::vector<double>(solution.value(control));

            // print output -----
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
            //auto advance = std::vector<double>(solution.value(adv)).front() * 1000;
            //auto rotation = std::vector<double>(solution.value(rot)).front();
            auto advance = std::vector<double>(solution.value(adv)).at(1) * 1000;
            auto rotation = std::vector<double>(solution.value(rot)).at(1);

            if(scene != nullptr) draw_path(std::vector<double>(solution.value(state)), robot_polygon, scene);

            qInfo() << __FUNCTION__ << "Iterations:" << (int) solution.stats()["iter_count"];
            qInfo() << __FUNCTION__ << "Status:" << QString::fromStdString(solution.stats().at("return_status"));

            advance = advance * gaussian(rotation);
            return std::make_tuple(advance, rotation, solution, balls);
        }
        catch (...)
        {
            std::cout << "No solution found" << std::endl;
            previous_values_of_solution.clear();
            previous_control_of_solution.clear();
            return {};
        }
    }
    ////////////////////// AUX /////////////////////////////////////////////////
    float MPC::gaussian(float x)
    {
        const double xset = consts.xset_gaussian;
        const double yset = consts.yset_gaussian;
        const double s = -xset*xset/log(yset);
        return exp(-x*x/s);
    }
    std::vector<double> MPC::e2v(const Eigen::Vector2d &v)
    {
        return std::vector<double>{v.x(), v.y()};
    }
    void MPC::draw_path(const std::vector<double> &path_robot_meters, QGraphicsPolygonItem *robot_polygon, QGraphicsScene *scene)
    {
        // draw optimum N points solution
        static std::vector<QGraphicsItem *> path_paint;
        static QString path_color = "orange";

        for(auto p : path_paint)
            scene->removeItem(p);
        path_paint.clear();

        uint s = 100;
        for(auto &&p : path_robot_meters | iter::chunked(3))
        {
            auto pw = robot_polygon->mapToScene(QPointF(p[0]*1000.f, p[1]*1000.f));
            path_paint.push_back(scene->addEllipse(pw.x()-s/2, pw.y()-s/2, s , s, QPen(path_color), QBrush(QColor(path_color))));
            path_paint.back()->setZValue(30);
        }
    }

} // mpc