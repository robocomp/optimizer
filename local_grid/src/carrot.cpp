//
// Created by pbustos on 2/07/22.
//

#include "carrot.h"
#include <cppitertools/sliding_window.hpp>
#include <QtCore>

std::tuple<float, float, float> Carrot::update(const std::vector<Eigen::Vector2f> &path_robot)  //path in robot RS
{
    if (not path_robot.empty())
    {
        // get a good target from the path
        // advance through the path until curvature is high, or max_dist.
//        for(auto &&seg : path_robot | iter::sliding_window(3))
//        {
//            QLineF l1(e2q(seg[0]), e2q(seg[1]));
//            QLineF l2(e2q(seg[1]), e2q(seg[2]));
//            auto rad = cos(qDegreesToRadians(l1.angleTo(l2)));
//            qInfo() << __FUNCTION__ << rad;
//        }
//        qInfo() << __FUNCTION__ << "------------------";
        Eigen::Vector2f target_r = path_robot.front();

        float dist = target_r.norm();
        //qInfo() << __FUNCTION__ << path_robot.size() << dist;
        if(dist > constants.min_dist_to_target)
        {
            float beta = atan2(target_r.x(), target_r.y());
            beta = std::clamp(beta, -constants.max_rotation_speed, constants.max_rotation_speed);
            float k2 = 0.8;
            float f1 = std::clamp(dist / 1000, 0.f, 1.f);
            float f2 = gaussian_break(k2*beta);
            //qInfo() << __FUNCTION__ << constants.max_advance_speed*f1*f2 << f1 << f2<< k2*beta;
            return std::make_tuple(constants.max_advance_speed*f1*f2, k2*beta, 0.f);
        }
        else
        {
            qInfo() << __FUNCTION__ << "Robot at target. Stopping";
            return std::make_tuple(0.f, 0.f, 0.f);
        }
    }
    else  //empty path
        return {};
    }

float Carrot::gaussian_break(float x)
{
    const double xset = constants.xset_gaussian;
    const double yset = constants.yset_gaussian;
    const double s = -xset*xset/log(yset);
    return exp(-x*x/s);
}