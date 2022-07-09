//
// Created by pbustos on 2/07/22.
//

#include "carrot.h"
#include <cppitertools/sliding_window.hpp>
#include <QtCore>

std::tuple<float, float, float> Carrot::update(const std::vector<Eigen::Vector2f> &path_robot, /*path in robot RS*/
                                               QGraphicsPolygonItem *robot_polygon,
                                               QGraphicsScene *scene)
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

        // get target
        float dist_along_path = 0.f, euc_dist;
        std::size_t index = 0;
        for(const auto &p : iter::sliding_window(path_robot,2))
        {
            dist_along_path += (p[1]-p[0]).norm();
            euc_dist = (p[1] - path_robot[0]).norm();
            if((dist_along_path > constants.max_target_distance) or (dist_along_path>euc_dist*1.1))
                break;
            index++;
        }
        Eigen::Vector2f target_r = path_robot.at(index);
        if(scene != nullptr) draw_target(target_r, robot_polygon, scene);

        float dist = target_r.norm();
        float beta = atan2(target_r.x(), target_r.y());
        beta = std::clamp(beta, -constants.max_rotation_speed, constants.max_rotation_speed);
        float k2 = 0.8;
        float f1 = std::clamp(dist / 1000, 0.f, 1.f);
        float f2 = gaussian_break(k2*beta);
        return std::make_tuple(constants.max_advance_speed*f1*f2, k2*beta, 0.f);
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
void Carrot::draw_target(const Eigen::Vector2f &target_r, QGraphicsPolygonItem *robot_polygon, QGraphicsScene *scene)
{
    static QGraphicsRectItem *target_draw;
    if(target_draw != nullptr)
        scene->removeItem(target_draw);

    auto target_world = robot_polygon->mapToScene(QPointF(target_r.x(), target_r.y()));
    target_draw = scene->addRect(target_world.x()-100, target_world.y()-100, 200 , 200, QPen(QColor("Orange")), QBrush(QColor("Orange")));
}