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
        double gauss_dist = 0.1;                // minimun distance to a lidar-gaussian as constraint
        double point_dist = 0.2;                // value in lidar-gaussian at lidar points (corners)
        double point_sigma = 0.07;              // variance for point (corners) gaussians
        double gauss_value_for_point = 0.3;     // minimun distance to a corner gaussian as constraint
        float min_dist_to_target = 0.1;         // min distance to target at which the robot stops
        double max_rotation_value = 0.5;        // max rotation constraint in rads/sg
        double max_advance_value = 1;           // max advance constraint in m/sg
        double min_advance_value = 0;           // min advance constraint in m/sg
        double xset_gaussian = 0.5;             // gaussian break x set value
        double yset_gaussian = 0.4;             // gaussian break y set value
    };
    Constants consts;
    bool startup_check_flag;
    AbstractGraphicViewer *viewer_robot;
    casadi::Opti initialize_differential(const int N);
    void move_robot(float adv, float rot, float side = 0);
    std::vector<double> e2v(const Eigen::Vector2d &v);
    QPointF e2q(const Eigen::Vector2d &v);
    inline Eigen::Vector2f q2e(const QPointF &p) const {return Eigen::Vector2f(p.x(), p.y());};
    Eigen::Vector2d from_robot_to_world(const Eigen::Vector2d &p, const Eigen::Vector2d &robot_tr, double robot_ang);
    Eigen::Vector2d from_world_to_robot(const Eigen::Vector2d &p, const Eigen::Vector2d &robot_tr, double robot_ang);
    void draw_path(const std::vector<double> &path,  const Eigen::Vector2d &tr_world, double my_rot);

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
        bool active = true;
        QPointF pos; //mm
        Eigen::Vector2d to_eigen() const
        { return Eigen::Vector2d(pos.x(), pos.y()); }
        Eigen::Vector2d to_eigen_meters() const
        { return Eigen::Vector2d(pos.x()/1000, pos.y()/1000); }
        QGraphicsEllipseItem *draw = nullptr;
    };
    Target target;
    std::vector<double> previous_values_of_solution;

    // casadi
    casadi::Opti opti;
    casadi::MX state;
    casadi::MX pos;
    casadi::MX phi;
    casadi::MX control;
    casadi::MX adv;
    casadi::MX rot;
    std::vector<Eigen::Vector2d> convex_polygon;
    std::optional<std::tuple<double, double, casadi::OptiSol>> minimize(const Target &my_target,
                                                                        const QPolygonF &poly_laser_robot,
                                                                        const vector<Gaussian> &laser_gaussians,
                                                                        const Eigen::Vector3d &current_pose_meters);

    //robot
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsEllipseItem *laser_in_robot_polygon;
    void draw_laser(const QPolygonF &poly_robot);
    std::tuple<RoboCompGenericBase::TBaseState, Eigen::Vector3d> read_base();
    std::tuple<QPolygonF, QPolygonF, RoboCompLaser::TLaserData> read_laser(const Eigen::Vector2d &robot_tr, double robot_angle);
    float gaussian(float x);

    // convex parrtitions
    using Point = std::pair<float, float>;  //only for RDP, change to QPointF
    using Lines = std::vector<std::tuple<float, float, float>>;
    using Obstacles = std::vector<std::tuple<Lines, QPolygonF>>;
    std::optional<Eigen::Vector2d> find_inside_target(const Eigen::Vector2d &target_in_robot, const RoboCompLaser::TLaserData &ldata, const QPolygonF &poly);
    Obstacles compute_laser_partitions(QPolygonF &laser_poly);
    QPolygonF ramer_douglas_peucker(const RoboCompLaser::TLaserData &ldata, double epsilon);
    void ramer_douglas_peucker_rec(const vector<Point> &pointList, double epsilon, std::vector<Point> &out);
    void draw_partitions(const Obstacles &obstacles, const QColor &color, bool print=false);

    // tests
    Lines get_cube_lines(const Eigen::Vector2d &robot_tr, double robot_angle);

    // Bill
    bool read_bill(const RoboCompGenericBase::TBaseState &bState);

    void draw_target(const Target &target);
};
#endif
