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

/**
	\brief
	@author authorname
*/

//#pragma push_macro("slots")
//#undef slots
//#include <Python.h>
//#pragma pop_macro("slots")

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


#include <genericworker.h>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <casadi/core/optistack.hpp>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include "polypartition.h"
//#include <template_utilities/template_utilities.h>
#include <Eigen/Eigenvalues>
//#include <unsupported/Eigen/Splines>
#include <grid2d/grid.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker() override;
    bool setParams(RoboCompCommonBehavior::ParameterList params) override;

public slots:

    void compute() override;
    int startup_check();
    void initialize(int period) override;
    void new_target_slot(QPointF);

private:

    struct Constants
    {
        int num_steps = 10;                     // MPC steps ahead
        const float robot_radius = 300;
        double gauss_dist = 0.1;                // minimun distance to a lidar-gaussian as constraint
        double point_dist = 0.2;                // value in lidar-gaussian at lidar points (corners)
        double point_sigma = 0.07;              // variance for point (corners) gaussians
        double gauss_value_for_point = 0.3;     // minimun distance to a corner gaussian as constraint
        float min_dist_to_target = 0.9;         // min distance to target at which the robot stops
        double max_rotation_value = 0.5;        // max rotation constraint in rads/sg
        double max_advance_value = 0.9;           // max advance constraint in m/sg
        double min_advance_value = 0;           // min advance constraint in m/sg
        double xset_gaussian = 0.5;             // gaussian break x set value
        double yset_gaussian = 0.6;             // gaussian break y set value
        double min_line_dist = 0.4;
        float max_RDP_deviation = 70;
        float laser_noise_sigma  = 15;
        float max_ball_radius = 1 ;             // meters
        const float peak_threshold = 500;       // opening detector
        const float target_noise_sigma = 50;
        const int num_lidar_affected_rays_by_hard_noise = 1;
    };
    Constants consts;
    bool startup_check_flag;
    using Ball = std::tuple<Eigen::Vector2d, float, Eigen::Vector2d>;
    using Balls = std::vector<Ball>;

    //robot
    AbstractGraphicViewer *viewer_robot;
    casadi::Opti initialize_differential(const int N);
    void move_robot(float adv, float rot, float side = 0);
    std::vector<double> e2v(const Eigen::Vector2d &v);
    QPointF e2q(const Eigen::Vector2d &v);
    inline Eigen::Vector2f q2e(const QPointF &p) const {return Eigen::Vector2f(p.x(), p.y());};
    Eigen::Vector2d from_robot_to_world(const Eigen::Vector2d &p, const Eigen::Vector2d &robot_tr, double robot_ang);
    Eigen::Vector2d from_world_to_robot(const Eigen::Vector2d &p, const Eigen::Vector2d &robot_tr, double robot_ang);
    void draw_path(const std::vector<double> &path,  const Eigen::Vector2d &tr_world, double my_rot, const Balls &balls);

    // gaussians
    struct Gaussian
    {
        casadi::MX mu;
        casadi::MX i_sigma;
    };
    std::vector<SpecificWorker::Gaussian> fit_gaussians_to_laser(const QPolygonF &poly_laser_robot, const RoboCompGenericBase::TBaseState &bState, bool draw);

    // target
    struct Target
    {
        private:
            bool active = false;
            QPointF pos, pos_ant=QPointF(0,0); //mm
        public:
            void set_active(bool b) { active = b;}
            bool is_active() const { return active;}
            QPointF get_pos() const { return pos; }
            QPointF get_pos_ant() const { return pos_ant; }
            QPointF get_velocity() const {return pos-pos_ant;}
            Eigen::Vector2d get_velocity_meters() const {auto v = get_velocity(); return Eigen::Vector2d(v.x()/1000.0, v.y()/1000.0);}
            void set_pos(const QPointF &p) { pos_ant = pos; pos = p; }
            Eigen::Vector2d to_eigen() const
            { return Eigen::Vector2d(pos.x(), pos.y()); }
            Eigen::Vector2d to_eigen_meters() const
            { return Eigen::Vector2d(pos.x()/1000, pos.y()/1000); }
            QGraphicsEllipseItem *draw = nullptr;
    };
    Target target;

    // convex parrtitions
    using Point = std::pair<float, float>;  //only for RDP, change to QPointF
    using Lines = std::vector<std::tuple<float, float, float>>;
    using Obstacles = std::vector<std::tuple<Lines, QPolygonF>>;
    std::optional<Eigen::Vector2d> find_inside_target(const Eigen::Vector2d &target_in_robot, const RoboCompLaser::TLaserData &ldata, const QPolygonF &poly);
    Obstacles compute_laser_partitions(QPolygonF &laser_poly);
    QPolygonF ramer_douglas_peucker(const RoboCompLaser::TLaserData &ldata, double epsilon);
    void ramer_douglas_peucker_rec(const vector<Point> &pointList, double epsilon, std::vector<Point> &out);
    void draw_partitions(const Obstacles &obstacles, const QColor &color, bool print=false);

    // casadi
    std::vector<double> previous_values_of_solution, previous_control_of_solution;
    casadi::Opti opti;
    casadi::MX state;
    casadi::MX pos;
    casadi::MX phi;
    casadi::MX control;
    casadi::MX adv;
    casadi::MX rot;
    casadi::MX slack_vector;
    std::vector<Eigen::Vector2d> convex_polygon;
    std::optional<std::tuple<double, double, casadi::OptiSol>> minimize(const Target &my_target,
                                                                        const QPolygonF &poly_laser_robot,
                                                                        const vector<Gaussian> &laser_gaussians,
                                                                        const Eigen::Vector3d &current_pose_meters);

    std::optional<std::tuple<double, double, casadi::OptiSol>> minimize_free(const Target &my_target,
                                                                             const QPolygonF &poly_laser_robot,
                                                                             const std::vector<Gaussian> &laser_gaussians,
                                                                             const Eigen::Vector3d &current_pose_meters,
                                                                             const Obstacles &obstacles);

    optional<tuple<double, double, casadi::OptiSol, Balls>> minimize_balls(const Target &my_target,
                                                                           const Eigen::Vector3d &current_pose_meters,
                                                                           const RoboCompLaser::TLaserData &ldata);
    Ball compute_free_ball(const Eigen::Vector2d &center, const std::vector<Eigen::Vector2d> &lpoints);

    //robot
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsEllipseItem *laser_in_robot_polygon;
    void draw_laser(const QPolygonF &poly_robot);
    std::tuple<RoboCompGenericBase::TBaseState, Eigen::Vector3d> read_base();
    std::tuple<QPolygonF, QPolygonF, RoboCompLaser::TLaserData> read_laser(const Eigen::Vector2d &robot_tr, double robot_angle, bool noise=false);
    float gaussian(float x);


    // tests
    Lines get_cube_lines(const Eigen::Vector2d &robot_tr, double robot_angle);

    // Bill
    bool read_bill(const RoboCompGenericBase::TBaseState &bState);

    void draw_target(const Target &target);
    Target sub_target( const Target &target, const QPolygonF &poly,
                       const RoboCompLaser::TLaserData &ldata,
                       const Eigen::Vector2d &robot_tr_mm,
                       double robot_ang);

    // Grid
    Grid grid;

};
#endif
