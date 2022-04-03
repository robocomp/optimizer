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
            const float max_advance_speed = 800;
            float tile_size = 100;
            const float max_laser_range = 4000;
            float current_rot_speed = 0;
            float current_adv_speed = 0;
            float robot_length = 450;
            const float robot_semi_length = robot_length/2.0;
            const float final_distance_to_target = 150; //mm
            const float min_dist_to_target = 100; //mm
        };
        Constants constants;

        //robot
        struct Pose2D
        {
            float ang;
            Eigen::Vector2f pos;
            QPointF toQpointF() const { return QPointF(pos.x(), pos.y());};
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
        void goto_target(const std::vector<Eigen::Vector2f> &path);

        // grid
        QRectF dimensions;
        Grid grid;
        Pose2D grid_world_pose;
        void update_map(const RoboCompLaser::TLaserData &ldata);

        // laser
        void read_laser();
        void draw_laser(const RoboCompLaser::TLaserData &ldata);

        // camera
        void read_camera();

        // target
        struct Target
        {
            bool active = false;
            QPointF pos;
            QGraphicsEllipseItem *draw = nullptr;
            Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(pos.x(), pos.y());}
            Eigen::Vector3f to_eigen_3() const {return Eigen::Vector3f(pos.x(), pos.y(), 1.f);}
        };
        Target target;
        template <typename Func, typename Obj>
        auto quick_bind(Func f, Obj* obj)
        { return [=](auto&&... args) { return (obj->*f)(std::forward<decltype(args)>(args)...); };}

        // path
        void draw_path(const list<QPointF> &path);

};

#endif
