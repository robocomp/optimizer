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



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "dynamic_window.h"
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);
        RoboCompMoveTowards::Command MoveTowards_move(float x, float y, float alpha);

public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    private:
        struct Constants
        {
            const float max_advance_speed = 800;
            const float tile_size = 100;
            const float max_laser_range = 4000;
            float current_rot_speed = 0;
            float current_adv_speed = 0;
            float robot_length = 450;
            const float robot_semi_length = robot_length/2.0;
            const float final_distance_to_target = 150; //mm
        };
        Constants constants;

        QRectF dimensions;
        bool startup_check_flag;
        std::tuple<RoboCompFullPoseEstimation::FullPoseEuler, double, double> read_base();
        QPolygonF read_laser();
        using Point = std::pair<float, float>;  //only for RDP, change to QPointF
        QPolygonF ramer_douglas_peucker(RoboCompLaser::TLaserData &ldata, double epsilon);
        void ramer_douglas_peucker_rec(const vector<Point> &pointList, double epsilon, std::vector<Point> &out);
        Dynamic_Window dwa;

        //robot
        AbstractGraphicViewer *viewer_robot;
        const int ROBOT_LENGTH = 400;
        QGraphicsPolygonItem *robot_polygon;
        QGraphicsRectItem *laser_in_robot_polygon;
        void draw_laser(const QPolygonF &ldata);
        Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p,  const RoboCompFullPoseEstimation::FullPoseEuler &r_state);
        double global_advance, global_rotation;
        Eigen::Vector2f target_in_robot;
        QPolygonF laser_poly;

        // target
        struct Target
        {
            bool active = false;
            QPointF pos;
            Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(pos.x(), pos.y());}
            QGraphicsEllipseItem *draw = nullptr;
        };
        Target target;

        void move_robot(float adv, float rot);

    float gaussian(float x);
};

#endif
