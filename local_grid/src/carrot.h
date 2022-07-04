//
// Created by pbustos on 2/07/22.
//

#ifndef LOCAL_GRID_CARROT_H
#define LOCAL_GRID_CARROT_H

#include <vector>
#include <Eigen/Dense>
#include <QDebug>
#include <QLineF>

class Carrot
{
    public:
        std::tuple<float, float, float> update (const std::vector<Eigen::Vector2f> &path);
        void set_max_advance_speed(float max_advance_speed_) {constants.max_advance_speed = max_advance_speed_;};
        void final_distance_to_target(float final_distance_to_target_) {constants.final_distance_to_target = final_distance_to_target_;};
        void min_distance_to_target(float min_distance_to_target_) {constants.final_distance_to_target = min_distance_to_target_;};

    private:
        struct Constants
        {
            float max_advance_speed = 1200;
            float max_rotation_speed = 1;
            float robot_length = 500;
            float robot_semi_length = robot_length/2.0;
            float final_distance_to_target = 700; //mm
            float min_dist_to_target = 100; //mm
            double xset_gaussian = 0.6;             // gaussian break x set value
            double yset_gaussian = 0.5;             // gaussian break y set value
        };
        Constants constants;

        float gaussian_break(float x);
        inline QPointF e2q(const Eigen::Vector2f &p) const {return QPointF(p.x(), p.y());};
};


#endif //LOCAL_GRID_CARROT_H
