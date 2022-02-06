//
// Created by pbustos on 11/11/21.
//

#include "dynamic_window.h"
#include <QtCore>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>

Dynamic_Window::Dynamic_Window()
{
    polygon_robot <<  QPointF(-constants.robot_semi_width, constants.robot_semi_width) <<
                      QPointF(constants.robot_semi_width, constants.robot_semi_width) <<
                      QPointF(constants.robot_semi_width, -constants.robot_semi_width) <<
                      QPointF(-constants.robot_semi_width, -constants.robot_semi_width);
}

Dynamic_Window::Result Dynamic_Window::compute(const Eigen::Vector2f &target_r, const QPolygonF &laser_poly,
                                               double advance, double rot, const Eigen::Vector3f &robot,
                                               QGraphicsScene *scene)
{
    static float previous_turn = 0;

    // compute future positions of the robot
    auto point_list = compute_predictions(advance, rot, laser_poly);

    // compute best value
    auto best_choice = compute_optimus(point_list, target_r, previous_turn);

    if(scene != nullptr)
        draw(robot, point_list, best_choice, scene);

    if (best_choice.has_value())
    {
        auto[x, y, v, w, alpha]= best_choice.value();  // x,y coordinates of best point, v,w velocities to reach that point, alpha robot's angle at that point
        previous_turn = w;
        return best_choice.value();
    }
    else
        return Result{};
}

std::vector<Dynamic_Window::Result> Dynamic_Window::compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly)
{
    std::vector<Result> list_points;
    for (float v = -100; v <= constants.max_advance_speed; v += 50) //advance
        for (float w = -1; w <= 1; w += 0.2) //rotation
        {
            float new_adv = current_adv + v;
            float new_rot = -current_rot + w;
            if (fabs(w) > 0.001)  // avoid division by zero to compute the radius
            {
                float r = new_adv / new_rot; // radio de giro ubicado en el eje x del robot
                float arc_length = new_rot * constants.time_ahead * r;
                for (float t = constants.step_along_arc; t < arc_length; t += constants.step_along_arc)
                {
                    float x = r - r * cos(t / r); float y= r * sin(t / r);  // circle parametric coordinates
                    auto point = std::make_tuple(x, y, new_adv, new_rot, t / r);
                    if(sqrt(x*x + y*y)> constants.robot_semi_width and point_reachable_by_robot(point, laser_poly)) // skip points in the robot
                        list_points.emplace_back(std::move(point));
                }
            }
            else // para evitar la división por cero en el cálculo de r
            {
                for(float t = constants.step_along_arc; t < new_adv * constants.time_ahead; t+=constants.step_along_arc)
                {
                    auto point = std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead);
                    if (t > constants.robot_semi_width and point_reachable_by_robot(point, laser_poly))
                        list_points.emplace_back(std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead));
                }
            }
        }
    return list_points;
}
bool Dynamic_Window::point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly)
{
    auto [x, y, adv, giro, ang] = point;
    float parts = Eigen::Vector2f(x,y).norm()/(constants.robot_semi_width/2);
    float ang_delta = ang / parts;
    float init_ang = 0;

    auto linl = laser_in_robot_polygon->mapFromParent(laser_poly);
    auto lp = laser_in_robot_polygon->mapFromParent(polygon_robot);

    for(const auto &l: iter::range(0.0, 1.0, 1.0/parts))
    {
        init_ang += ang_delta;
        auto temp_robot = QTransform().rotate(init_ang).translate(x, y).map(lp);  // compute incremental rotation
        if (auto res = std::ranges::find_if_not(temp_robot, [linl](const auto &p)
                                        { return linl.containsPoint(p, Qt::OddEvenFill); }); res != std::end(lp))
        {
            //qInfo() << __FUNCTION__ << "No cabe";
            return false;
        }
    }
    return true;
}

//bool Dynamic_Window::point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly)
//{
//    auto [x, y, adv, giro, ang] = point;
//    Eigen::Vector2f robot_r(0.0,0.0);
//    Eigen::Vector2f goal_r(x, y);
//    float parts = goal_r.norm()/(constants.robot_semi_width/6.0);
//    Eigen::Vector2f rside(260, 100);
//    Eigen::Vector2f lside(-260, 100);
//    QPointF p,q,r;
//    for(const auto &l: iter::range(0.0, 1.0, 1.0/parts))
//    {
//        p = to_qpointf(robot_r*(1-l) + goal_r*l);
//        q = to_qpointf((robot_r+rside)*(1-l) + (goal_r+rside)*l);
//        r = to_qpointf((robot_r+lside)*(1-l) + (goal_r+lside)*l);
//        if( not laser_poly.containsPoint(p, Qt::OddEvenFill) or
//            not laser_poly.containsPoint(q, Qt::OddEvenFill) or
//            not laser_poly.containsPoint(r, Qt::OddEvenFill))
//        return false;
//    }
//    return true;
//}

std::optional<Dynamic_Window::Result> Dynamic_Window::compute_optimus(const std::vector<Result> &points, const Eigen::Vector2f &tr,
                                                                      float previous_turn)
{
    const float A=1, B=5;  // CHANGE
    std::vector<std::tuple<float, Result>> values(points.size());
    for(auto &&[k, point] : iter::enumerate(points))
    {
        auto [x, y, adv, giro, ang] = point;
        float dist_to_target = (Eigen::Vector2f(x, y) - tr).norm();
        //float dist_to_previous_turn =  fabs(giro - previous_turn);
        float dist_to_previous_turn =  fabs(giro);
        values[k] = std::make_tuple(A*dist_to_target + B*dist_to_previous_turn, point);
    }
    auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
    if(min != values.end())
        return std::get<Result>(*min);
    else
        return {};
}
Eigen::Vector2f Dynamic_Window::from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot)
{
    Eigen::Matrix2f matrix;
    const float &angle = robot.z();
    matrix << cos(angle) , -sin(angle) , sin(angle) , cos(angle);
    return (matrix * p) + Eigen::Vector2f(robot.x(), robot.y());
}

Eigen::Vector2f Dynamic_Window::from_world_to_robot(const Eigen::Vector2f &p, const Eigen::Vector3f &robot)
{
    Eigen::Matrix2f matrix;
    const float &angle = robot.z();
    matrix << cos(angle) , -sin(angle) , sin(angle) , cos(angle);
    return (matrix.transpose() * (p - Eigen::Vector2f(robot.x(), robot.y())));
}

void Dynamic_Window::draw(const Eigen::Vector3f &robot, const std::vector <Result> &puntos,  const std::optional<Result> &best, QGraphicsScene *scene)
{
    static std::vector<QGraphicsEllipseItem *> arcs_vector;
    // remove current arcs
    for (auto arc: arcs_vector)
        scene->removeItem(arc);
    arcs_vector.clear();

    QColor col("Blue");
    for (auto &[x, y, vx, wx, a] : puntos)
    {
        //QPointF centro = robot_polygon_draw->mapToScene(x, y);
        QPointF centro = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
        auto arc = scene->addEllipse(centro.x(), centro.y(), 50, 50, QPen(col, 10));
        arc->setZValue(30);
        arcs_vector.push_back(arc);
    }

    if(best.has_value())
    {
        auto &[x, y, _, __, ___] = best.value();
        QPointF selected = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
        auto arc = scene->addEllipse(selected.x(), selected.y(), 180, 180, QPen(Qt::black), QBrush(Qt::black));
        arc->setZValue(30);
        arcs_vector.push_back(arc);
    }
}
