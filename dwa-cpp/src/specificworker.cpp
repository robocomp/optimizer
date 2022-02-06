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
#include "specificworker.h"
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
    this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else {
        dimensions = QRectF(-5100, -2600, 10200, 5200);
        viewer_robot = new AbstractGraphicViewer(this->graphicsView, dimensions);
        auto [rp, lp] = viewer_robot->add_robot(constants.robot_width, constants.robot_length, constants.laser_x_offset, constants.laser_y_offset);
        robot_draw_polygon = rp;
        laser_draw_polygon = lp;
        connect(viewer_robot, &AbstractGraphicViewer::new_mouse_coordinates, [this](QPointF t)
                {
                    qInfo() << __FUNCTION__ << " Received new target at " << t;
                    target.pos = t;
                    target.active = true;
                    target.draw = viewer_robot->scene.addEllipse(t.x()-50, t.y()-50, 100, 100, QPen(QColor("magenta")), QBrush(QColor("magenta")));
                });

        // left and right expanded semi-polygons to detect collisions. They have to be transformed into the laser ref. system
        const float ext_robot_semi_width = constants.robot_width/2;
        const float ext_robot_semi_length = constants.robot_length/2;
        left_polygon_robot <<  QPointF(0, -ext_robot_semi_length) <<
                               QPointF(-ext_robot_semi_width, -ext_robot_semi_length) <<
                               QPointF(-ext_robot_semi_width, 0) <<
                               QPointF(-ext_robot_semi_width, ext_robot_semi_length/2) <<
                               QPointF(-ext_robot_semi_width, ext_robot_semi_length) <<
                               QPointF(0, constants.robot_semi_length*1.1);
        right_polygon_robot << QPointF(0,-ext_robot_semi_length) <<
                               QPointF(ext_robot_semi_width, -ext_robot_semi_length) <<
                               QPointF(ext_robot_semi_width, 0) <<
                               QPointF(ext_robot_semi_width, ext_robot_semi_length/2) <<
                               QPointF(ext_robot_semi_width, ext_robot_semi_length) <<
                               QPointF(0, ext_robot_semi_length*1.1);

        // robot polygon to verify that the candidate path is traversable
        polygon_robot <<  QPointF(-constants.robot_semi_width*1.2, constants.robot_semi_length) <<
                          QPointF(constants.robot_semi_width*1.2, constants.robot_semi_length) <<
                          QPointF(constants.robot_semi_width, -constants.robot_semi_length) <<
                          QPointF(-constants.robot_semi_width, -constants.robot_semi_length);

        // QCustomPlot
        custom_plot.setParent(timeseries_frame);
        custom_plot.xAxis->setLabel("time");
        custom_plot.yAxis->setLabel("rot-b adv-g lhit-m stuck-r");
        custom_plot.xAxis->setRange(0, 200);
        custom_plot.yAxis->setRange(-10, 10);
        rot_graph = custom_plot.addGraph();
        rot_graph->setPen(QColor("blue"));
        adv_graph = custom_plot.addGraph();
        adv_graph->setPen(QColor("green"));
        lhit_graph = custom_plot.addGraph();
        lhit_graph->setPen(QColor("magenta"));
        rhit_graph = custom_plot.addGraph();
        rhit_graph->setPen(QColor("magenta"));
        stuck_graph = custom_plot.addGraph();
        stuck_graph->setPen(QColor("red"));
        custom_plot.resize(timeseries_frame->size());
        custom_plot.show();

        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    static float delta_rot = constants.initial_delta_rot;
    bool lhit = false, rhit= false, stuck = false;
    auto [r_state, advance, rotation] = read_base();
    r_state_global = r_state;
    global_advance = advance;
    global_rotation = rotation;

    laser_poly = read_laser();

    // Bill
//    if(auto t = read_bill(); t.has_value())
//        target = t.value();

    if(target.active)
    {
        target_in_robot = from_world_to_robot(target.to_eigen(), r_state);
        auto dist = target_in_robot.norm();
        if( dist > constants.final_distance_to_target)
        {
            Result res;
            if( auto res_o =  control(target_in_robot, laser_poly, advance, rotation, Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                     &viewer_robot->scene); not res_o.has_value())
            {
                qInfo() << __FUNCTION__ << "NO CONTROL";
                stuck = do_if_stuck(0, 0, r_state, lhit, rhit);
                return;
            }
            else
                res = res_o.value();
            auto [_, __, adv, rot, ___] = res;

            float dist_break = std::clamp(dist / constants.final_distance_to_target - 1.0, -1.0, 1.0);
            float adv_n = constants.max_advance_speed * dist_break * gaussian(rot);
            auto linl = laser_draw_polygon->mapFromParent(laser_poly);
            auto lp = laser_draw_polygon->mapFromParent(left_polygon_robot);
            if (auto res = std::ranges::find_if_not(lp, [linl](const auto &p)
                { return linl.containsPoint(p, Qt::WindingFill); }); res != std::end(lp))
            {
                qInfo() << __FUNCTION__ << "---- TOCANDO POR LA IZQUIERDA-----------" << *res;
                rot += delta_rot;
                delta_rot *= 2;
                lhit = true;
            }
            else
            {
                delta_rot = constants.initial_delta_rot;
                lhit = false;
            }
            auto rp = laser_draw_polygon->mapFromParent(right_polygon_robot);
            if (auto res = std::ranges::find_if_not(rp, [linl](const auto &p)
                { return linl.containsPoint(p, Qt::WindingFill); }); res != std::end(rp))
            {
                qInfo() << __FUNCTION__ << "---- TOCANDO POR LA DERECHA-----------" << *res;
                rot -= delta_rot;
                delta_rot *= constants.initial_delta_rot;
                rhit = true;
            }
            else
            {
                delta_rot = rot;
                rhit = false;
            }
            rot = std::clamp(rot, -constants.max_rotation_speed, constants.max_rotation_speed);
            move_robot(adv_n, rot);
            qInfo() << __FUNCTION__ << adv_n << rot;
            //stuck = do_if_stuck(adv_n, rot, r_state, lhit, rhit);
            draw_timeseries(rot*5, adv_n/100, (int)lhit*5, (int)rhit*5, (int)stuck*5);
        }
        else
        {
            move_robot(0, 0);
            target.active = false;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// checks if the robot is moving in a time interval
bool SpecificWorker::do_if_stuck(float adv, float rot, const RoboCompFullPoseEstimation::FullPoseEuler &r_state, bool lhit, bool rhit)
{
    //static bool first_time = true;
    const float SMALL_FLOAT = 0.001;
    const float SMALL_INT = 1.0;
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<float> dist(0.3, 1.0);

    qInfo() << __FUNCTION__ << r_state.vy << r_state.vrz;
    if( fabs(r_state.vy) < SMALL_INT  and fabs(r_state.vrz) < SMALL_FLOAT )
    {
        if(lhit) move_robot(-constants.backward_speed, -dist(mt));
        else if(rhit) move_robot(-constants.backward_speed, dist(mt));
        else move_robot(-constants.backward_speed, 0);
        qInfo() << __FUNCTION__ << "------ STUCK ----------";
        return true;
    }
    return false;
}
std::optional<SpecificWorker::Result> SpecificWorker::control(const Eigen::Vector2f &target_r, const QPolygonF &laser_poly,
                                               double advance, double rot, const Eigen::Vector3f &robot,
                                               QGraphicsScene *scene)
{
    static float previous_turn = 0;

    // compute future positions of the robot
    auto point_list = compute_predictions(advance, rot, laser_poly);

    // compute best value
    auto best_choice = compute_optimus(point_list, target_r);

    if(scene != nullptr)
        draw_dwa(robot, point_list, best_choice, scene);

    if (best_choice.has_value())
    {
        auto[x, y, v, w, alpha]= best_choice.value();  // x,y coordinates of best point, v,w velocities to reach that point, alpha robot's angle at that point
        previous_turn = w;
        return best_choice.value();
    }
    else
        return {};
}
std::vector<SpecificWorker::Result> SpecificWorker::compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly)
{
    std::vector<Result> list_points;
    for (float v = 0; v <= constants.max_advance_speed; v += 50) //advance
        for (float w = -1; w <= 1; w += 0.1) //rotation
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
bool SpecificWorker::point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly)
{
    auto [x, y, adv, giro, ang] = point;
    auto goal_r = Eigen::Vector2f(x,y);
    float parts = goal_r.norm()/(constants.robot_semi_width/3);  //should be a fraction of the arc
    float ang_delta = ang / parts;
    float init_ang = 0;

    auto linl = laser_draw_polygon->mapFromParent(laser_poly);
    auto lp = laser_draw_polygon->mapFromParent(polygon_robot);

    for(const auto &l: iter::range(0.0, 1.0, 1.0/parts))
    {
        init_ang += ang_delta;
        auto p = to_qpointf(goal_r * l);
        auto temp_robot = QTransform().rotate(init_ang).translate(p.x(), p.y()).map(lp);  // compute incremental rotation
        if (auto res = std::ranges::find_if_not(temp_robot, [linl](const auto &p)
            { return linl.containsPoint(p, Qt::OddEvenFill); }); res != std::end(temp_robot))
        {
            return false;
        }
    }
    return true;
}
std::optional<SpecificWorker::Result> SpecificWorker::compute_optimus(const std::vector<Result> &points,
                                                                      const Eigen::Vector2f &tr)
{
    std::vector<std::tuple<float, Result>> values(points.size());
    for(auto &&[k, point] : iter::enumerate(points))
    {
        auto [x, y, adv, giro, ang] = point;
        float dist_to_target = (Eigen::Vector2f(x, y) - tr).norm();
        //float dist_to_previous_turn =  fabs(giro - previous_turn);
        float dist_to_previous_turn =  fabs(giro);
        values[k] = std::make_tuple(constants.A_dist_factor*dist_to_target + constants.B_turn_factor*dist_to_previous_turn, point);
    }
    auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
    if(min != values.end())
        return std::get<Result>(*min);
    else
        return {};
}
void SpecificWorker::draw_dwa(const Eigen::Vector3f &robot, const std::vector <Result> &puntos,
                              const std::optional<Result> &best, QGraphicsScene *scene)
{
    static std::vector<QGraphicsEllipseItem *> arcs_vector;
    // remove current arcs
    for (auto arc: arcs_vector)
        scene->removeItem(arc);
    arcs_vector.clear();

    QColor col("Blue");
    for (auto &[x, y, vx, wx, a] : puntos)
    {
        //QPointF centro = robot_draw_polygon_draw->mapToScene(x, y);
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
///////////////////////////////////////////////////////////////////////////////////////////////////
std::optional<SpecificWorker::Target> SpecificWorker::read_bill()
{
    Target my_target;
    try
    {
        auto pose = billcoppelia_proxy->getPose();
        target.pos = QPointF(pose.x, pose.y);
        viewer_robot->scene.removeItem(target.draw);
        target.active = true;
        target.draw = viewer_robot->scene.addEllipse(target.pos.x()-50, target.pos.y()-50, 100, 100, QPen(QColor("magenta")), QBrush(QColor("magenta")));
    }
    catch(const Ice::Exception &e)
    {
        qInfo() << "Error connecting to Bill Coppelia";
        std::cout << e.what() << std::endl;
        return {};
    }
    return target;
}
std::tuple<RoboCompFullPoseEstimation::FullPoseEuler, double, double> SpecificWorker::read_base()
{
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    float advance, rot;
    try
    {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        advance = -sin(r_state.rz)*r_state.vx + cos(r_state.rz)*r_state.vy;
        rot = r_state.vrz;  // Rotation W
        robot_draw_polygon->setRotation(r_state.rz * 180 / M_PI);
        robot_draw_polygon->setPos(r_state.x, r_state.y);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
    return std::make_tuple(r_state, advance, rot);
}
QPolygonF SpecificWorker::read_laser()
{
    QPolygonF poly_robot;
    RoboCompLaser::TLaserData ldata;
    try
    {
        ldata = laser_proxy->getLaserData();
        float dist_ant = 50;
        for (auto &&l : ldata)
        {
            if (l.dist < 30)
                l.dist = dist_ant;
            else
                dist_ant = l.dist;
            poly_robot << QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle));
        }
        // Simplify laser contour with Ramer-Douglas-Peucker
        //poly_robot = ramer_douglas_peucker(ldata, constants.MAX_RDP_DEVIATION_mm);
        // add robot contour  wrt laser_location
        poly_robot.pop_front();
        poly_robot.push_front(QPointF(constants.robot_semi_width*1.1, -constants.robot_semi_length));
        poly_robot.push_front(QPointF(constants.robot_semi_width*1.1, -constants.robot_length));
        poly_robot.push_front(QPointF(0, -constants.robot_length));
        poly_robot.pop_back();
        poly_robot.push_back(QPointF(-constants.robot_semi_width*1.1, -constants.robot_semi_length));
        poly_robot.push_back(QPointF(-constants.robot_semi_width*1.1, -constants.robot_length));
        poly_robot.push_back(QPointF(0, -constants.robot_length));
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    draw_laser(poly_robot);
    return poly_robot;
}
void SpecificWorker::move_robot(float adv, float rot)
{
    try
    { differentialrobot_proxy->setSpeedBase(adv, rot); }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
}
void SpecificWorker::draw_laser(const QPolygonF &poly) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer_robot->scene.removeItem(laser_polygon);

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer_robot->scene.addPolygon(laser_draw_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}
void SpecificWorker::draw_timeseries(float rot, float adv, int lhit, int rhit, int stuck)
{
    static int cont = 0;
    rot_graph->addData(cont, rot);
    adv_graph->addData(cont, adv);
    lhit_graph->addData(cont, lhit);
    rhit_graph->addData(cont, rhit);
    stuck_graph->addData(cont++, stuck);
    custom_plot.xAxis->setRange(cont, 200, Qt::AlignRight);
    custom_plot.replot();
}
QPolygonF SpecificWorker::ramer_douglas_peucker(RoboCompLaser::TLaserData &ldata, double epsilon)
{
    if(ldata.size()<2)
    {
        qWarning() << __FUNCTION__ << "Not enough points to simplify";
        return QPolygonF();
    }
    std::vector<Point> pointList(ldata.size());
    float dist_ant = 50;
    for (auto &&[i, l] : ldata | iter::enumerate) {
        if (l.dist < 30)
            l.dist = dist_ant;
        else
            dist_ant = l.dist;
        pointList[i] = std::make_pair(l.dist * sin(l.angle), l.dist * cos(l.angle));
    }
    std::vector<Point> pointListOut;
    ramer_douglas_peucker_rec(pointList, epsilon, pointListOut);
    QPolygonF poly(pointListOut.size());
    for (auto &&[i, p] : pointListOut | iter::enumerate)
        poly[i] = QPointF(p.first, p.second);
    return poly;
}
void SpecificWorker::ramer_douglas_peucker_rec(const vector<Point> &pointList, double epsilon, std::vector<Point> &out)
{
    // Find the point with the maximum distance from line between start and end
    auto line = Eigen::ParametrizedLine<float, 2>::Through(Eigen::Vector2f(pointList.front().first, pointList.front().second),
                                                           Eigen::Vector2f(pointList.back().first, pointList.back().second));
    auto max = std::max_element(pointList.begin()+1, pointList.end(), [line](auto &a, auto &b)
    { return line.distance(Eigen::Vector2f(a.first, a.second)) < line.distance(Eigen::Vector2f(b.first, b.second));});

    // If max distance is greater than epsilon, recursively simplify
    float dmax =  line.distance(Eigen::Vector2f((*max).first, (*max).second));
    if(dmax > epsilon)
    {
        // Recursive call
        std::vector<Point> recResults1;
        std::vector<Point> recResults2;
        std::vector<Point> firstLine(pointList.begin(), max + 1);
        std::vector<Point> lastLine(max, pointList.end());
        ramer_douglas_peucker_rec(firstLine, epsilon, recResults1);
        ramer_douglas_peucker_rec(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end() - 1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if (out.size() < 2)
        {
            qWarning() << __FUNCTION__ << "Problem assembling output";
            return;
        }
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList.front());
        out.push_back(pointList.back());
    }
}
Eigen::Vector2f SpecificWorker::from_world_to_robot(const Eigen::Vector2f &p,
                                                    const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    Eigen::Matrix2f matrix;
    matrix << cos(r_state.rz) , -sin(r_state.rz) , sin(r_state.rz) , cos(r_state.rz);
    return (matrix.transpose() * (p - Eigen::Vector2f(r_state.x, r_state.y)));
}
Eigen::Vector2f SpecificWorker::from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot)
{
    Eigen::Matrix2f matrix;
    const float &angle = robot.z();
    matrix << cos(angle) , -sin(angle) , sin(angle) , cos(angle);
    return (matrix * p) + Eigen::Vector2f(robot.x(), robot.y());
}
float SpecificWorker::gaussian(float x)
{
    const double xset = 0.5;
    const double yset = 0.4;
    const double s = -xset*xset/log(yset);
    return exp(-x*x/s);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
RoboCompMoveTowards::Command SpecificWorker::MoveTowards_move(float x, float y, float alpha)
{
    qInfo() << __FUNCTION__ ;
    qInfo() << __FUNCTION__ << " Received new target at " << x << y << alpha;
    static float delta_rot = constants.initial_delta_rot;
    bool lhit, rhit, stuck;
    // Target in robot RS. Check if x,y,alpha are within ranges
    target.pos = QPointF(x, y);
    RoboCompMoveTowards::Command command{0.0, 0.0};
    Result res;

    if( auto res_o =  control(target.to_eigen(), laser_poly, global_advance, global_rotation,
                              Eigen::Vector3f(r_state_global.x, r_state_global.y, r_state_global.rz),
                              &viewer_robot->scene); not res_o.has_value())
        return command;
    else
        res = res_o.value();

    auto [_, __, adv, rot, ___] = res;
    double dist = Eigen::Vector2d(x,y).norm();
    float dist_break = std::clamp(dist / constants.final_distance_to_target - 1.0, -1.0, 1.0);
    float adv_n = constants.max_advance_speed * dist_break * gaussian(rot);
    auto linl = laser_draw_polygon->mapFromParent(laser_poly);
    auto lp = laser_draw_polygon->mapFromParent(left_polygon_robot);
    if (auto res = std::ranges::find_if_not(lp, [linl](const auto &p)
        { return linl.containsPoint(p, Qt::WindingFill); }); res != std::end(lp))
    {
        qInfo() << __FUNCTION__ << "---- TOCANDO POR LA IZQUIERDA-----------" << *res;
        rot += delta_rot;
        delta_rot *= 2;
        lhit = true;
    }
    else
    {
        delta_rot = constants.initial_delta_rot;
        lhit = false;
    }
    auto rp = laser_draw_polygon->mapFromParent(right_polygon_robot);
    if (auto res = std::ranges::find_if_not(rp, [linl](const auto &p)
        { return linl.containsPoint(p, Qt::WindingFill); }); res != std::end(rp))
    {
        qInfo() << __FUNCTION__ << "---- TOCANDO POR LA DERECHA-----------" << *res;
        rot -= delta_rot;
        delta_rot *= constants.initial_delta_rot;
        rhit = true;
    }
    else
    {
        delta_rot = rot;
        rhit = false;
    }
    rot = std::clamp(rot, -constants.max_rotation_speed, constants.max_rotation_speed);
    move_robot(adv_n, rot);
    stuck = do_if_stuck(adv_n, rot, r_state_global, lhit, rhit);
}

