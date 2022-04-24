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

#include <genericworker.h>
#include "/home/robocomp/robocomp/classes/abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <QGraphicsPolygonItem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "/home/robocomp/robocomp/classes/grid2d/grid.h"
#include "mpc.h"


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
            const float max_advance_speed = 1200;
            float tile_size = 100;
            const float max_laser_range = 4000;
            float current_rot_speed = 0;
            float current_adv_speed = 0;
            float robot_length = 500;
            const float robot_semi_length = robot_length/2.0;
            const float final_distance_to_target = 700; //mm
            const float min_dist_to_target = 100; //mm
            float lidar_noise_sigma  = 15;
            const int num_lidar_affected_rays_by_hard_noise = 0;
            double xset_gaussian = 0.5;             // gaussian break x set value
            double yset_gaussian = 0.7;             // gaussian break y set value
            const float target_noise_sigma = 50;
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
            QGraphicsEllipseItem *draw = nullptr;
            void set_pos(const QPointF &p) { pos_ant = pos; pos = p;};
            QPointF get_pos() const { return pos;};
            Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(pos.x(), pos.y());}
            Eigen::Vector3f to_eigen_3() const {return Eigen::Vector3f(pos.x()/1000.f, pos.y()/1000.f, 1.f);}
            float dist_to_target_ant() const {return (to_eigen() - Eigen::Vector2f(pos_ant.x(), pos_ant.y())).norm();};

            private:
                    QPointF pos, pos_ant = QPoint(0.f,0.f);

        };
        Target target;
        template <typename Func, typename Obj>
        auto quick_bind(Func f, Obj* obj)
        { return [=](auto&&... args) { return (obj->*f)(std::forward<decltype(args)>(args)...); };}

        // path
        void draw_path(const std::vector<Eigen::Vector2f> &path_in_robot);

        // mpc
        mpc::MPC mpc;

        float gaussian(float x);
        void draw_solution_path(const vector<double> &path,  const mpc::MPC::Balls &balls);

        bool read_bill(const Pose2D &robot_pose);
};

#endif
