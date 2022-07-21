/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


#pragma push_macro("slots")
#undef slots
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#pragma pop_macro("slots")

#include <genericworker.h>
#include "/home/robocomp/robocomp/classes/abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <QGraphicsPolygonItem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "/home/robocomp/robocomp/classes/grid2d/grid.h"
#include <fps/fps.h>
#include "mpc.h"
#include "carrot.h"
#include "dynamic_window.h"
#include "qcustomplot/qcustomplot.h"

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);
        void new_target_slot(QPointF);

    private:
        bool startup_check_flag;
        AbstractGraphicViewer *viewer;

        struct Constants
        {
            uint num_steps_mpc = 8;
            const float max_advance_speed = 1500;
            float tile_size = 100;
            const float max_laser_range = 4000;
            float current_rot_speed = 0;
            float current_adv_speed = 0;
            float robot_length = 500;
            const float robot_semi_length = robot_length/2.0;
            const float final_distance_to_target = 700; //mm
            const float max_dist_to_target = 600; //mm
            float lidar_noise_sigma  = 20;
            const int num_lidar_affected_rays_by_hard_noise = 2;
            double xset_gaussian = 0.4;             // gaussian break x set value
            double yset_gaussian = 0.3;             // gaussian break y set value
            const float target_noise_sigma = 50;
            const float prob_prior = 0.5;	        // Prior occupancy probability
            const float prob_occ = 0.9;	            // Probability that cell is occupied with total confidence
            const float prob_free = 0.4;            // Probability that cell is free with total confidence
            const int period_to_check_occluded_path = 400; //ms
        };
        Constants constants;

        //robot
        struct Pose2D
        {
            float ang;
            Eigen::Vector2f pos;
            QPointF toQpointF() const { return QPointF(pos.x(), pos.y());};
            Eigen::Vector3d to_vec3_meters() const { return Eigen::Vector3d(pos.x()/1000.0, pos.y()/1000.0, ang);};
        };
        const int ROBOT_LENGTH = 400;
        QGraphicsPolygonItem *robot_polygon;
        QGraphicsRectItem *laser_in_robot_polygon;
        QPointF last_point;
        std::vector<QGraphicsLineItem *> lines;
        Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p);
        Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p);
        Eigen::Vector2f from_grid_to_world(const Eigen::Vector2f &p);
        Eigen::Vector2f from_world_to_grid(const Eigen::Vector2f &p);
        Eigen::Matrix3f from_grid_to_robot_matrix();
        Eigen::Matrix3f from_robot_to_grid_matrix();
        inline QPointF e2q(const Eigen::Vector2f &p) const {return QPointF(p.x(), p.y());};
        inline Eigen::Vector2f q2e(const QPointF &p) const {return Eigen::Vector2f(p.x(), p.y());};
        Pose2D robot_pose;
        Pose2D read_robot();
        void goto_target_carrot(const std::vector<Eigen::Vector2f> &path_robot);
        void goto_target_mpc(const std::vector<Eigen::Vector2d> &path_robot, const RoboCompLaser::TLaserData &ldata);
        void move_robot(float adv, float rot, float side=0);
        float gaussian(float x);

         // grid
        QRectF dimensions;
        Grid grid;
        Pose2D grid_world_pose;
        void update_map(const RoboCompLaser::TLaserData &ldata);

        // laser
        RoboCompLaser::TLaserData read_laser(bool noise=false);
        void draw_laser(const RoboCompLaser::TLaserData &ldata);

        // camera
        void read_camera();

        // target
        struct Target
        {
            bool active = false;
            void set_pos(const QPointF &p) { pos_ant = pos; pos = p;};
            QPointF get_pos() const { return pos;};
            Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(pos.x(), pos.y());}
            Eigen::Vector3f to_eigen_3() const {return Eigen::Vector3f(pos.x()/1000.f, pos.y()/1000.f, 1.f);}
            float dist_to_target_ant() const {return (to_eigen() - Eigen::Vector2f(pos_ant.x(), pos_ant.y())).norm();};
            bool is_new() { if(is_new_var){ is_new_var = false; return true;} else return false; };
            void set_new(bool v) {is_new_var = v;}
            void draw(QGraphicsScene &scene)
            {
                if(draw_point != nullptr) scene.removeItem(draw_point);
                draw_point = scene.addEllipse(pos.x()-100, pos.y()-100, 200, 200, QPen(QColor("Magenta")), QBrush(QColor("magenta")));
                draw_point->setZValue(300);
            };

            private:
                QPointF pos, pos_ant = QPoint(0.f,0.f);
                QGraphicsEllipseItem *draw_point = nullptr;
                bool is_new_var=true;

        };
        Target target;
        template <typename Func, typename Obj>
        auto quick_bind(Func f, Obj* obj)
        { return [=](auto&&... args) { return (obj->*f)(std::forward<decltype(args)>(args)...); };}

        // path
        void draw_path(const std::vector<Eigen::Vector2f> &path_in_robot);
        void draw_solution_path(const vector<double> &path,  const mpc::MPC::Balls &balls);
        double path_length(const std::vector<Eigen::Vector2f> &path);
        std::vector<Eigen::Vector2f> smooth_spline(const std::vector<Eigen::Vector2f> &path_grid);
        std::vector<Eigen::Vector2f> alternative_path(const std::vector<Eigen::Vector2f> &path_grid);
        std::vector<Eigen::Vector2f> convert_to_robot_coordinates(const std::vector<Eigen::Vector2f> &smoothed_path_grid, const std::vector<Eigen::Vector2f> &path_grid);
        std::vector<Eigen::Vector2f> remove_points_close_to_robot(const std::vector<Eigen::Vector2f> &path_grid);
        void draw_path_smooth(const vector<Eigen::Vector2f> &path_in_robot);

        // mpc
        mpc::MPC mpc;

       // Bill
        bool read_bill(const Pose2D &robot_pose);

        // FPS
        FPSCounter fps;

        // not used
        vector<Eigen::Vector2f> bresenham(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);

        // pathfolloweres
        Carrot carrot;
        Dynamic_Window dwa;

        // pythons
        pybind11::scoped_interpreter guard{}; // start the interpreter and keep it alive
        py::object interpolate_spline, evaluate_spline;
        py::module np;

        // switch
        enum class Control {DWA, MPC, CARROT};
        Control control = Control::MPC;

        // QCUSTOMPLOT
        QCustomPlot custom_plot;
        QCPGraph *distance_to_target_graph, *advance_speed_graph, *rotation_speed_graph;
        void draw_timeseries(float dist, float adv, float rot);

};

#endif
