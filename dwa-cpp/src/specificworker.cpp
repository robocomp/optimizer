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
        robot_polygon = viewer_robot->add_robot(ROBOT_LENGTH);
        laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
        laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
        connect(viewer_robot, &AbstractGraphicViewer::new_mouse_coordinates, [this](QPointF t)
                {
                    qInfo() << __FUNCTION__ << " Received new target at " << t;
                    target.pos = t;
                    target.active = true;
                    target.draw = viewer_robot->scene.addEllipse(t.x()-50, t.y()-50, 100, 100, QPen(QColor("magenta")), QBrush(QColor("magenta")));
                });

        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    auto [r_state, advance, rotation] = read_base();
    r_state_global = r_state;
    laser_poly = read_laser();
    global_advance = advance;
    global_rotation = rotation;
    target_in_robot = from_world_to_robot(target.to_eigen(), r_state);
    if(target.active)
    {
        auto dist = target_in_robot.norm();
        if( dist > constants.final_distance_to_target)
        {
            auto[_, __, adv, rot, ___] = dwa.compute(target_in_robot, laser_poly, advance,
                                                     rotation, Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                     &viewer_robot->scene);
            float dist_break = std::clamp(dist / 1000.0, 0.0, 1.0);
            float adv_n = constants.max_advance_speed * dist_break * gaussian(rotation);
            move_robot(adv_n, rot);
        }
        else
        {
            move_robot(0, 0);
            target.active = false;
        }
    }
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
        robot_polygon->setRotation(r_state.rz * 180 / M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
    return std::make_tuple(r_state, advance, rot);
}
QPolygonF SpecificWorker::read_laser()
{
    QPolygonF poly_robot;
    RoboCompLaser::TLaserData ldata;
    const float MAX_RDP_DEVIATION_mm  =  100;
    try
    {
        ldata = laser_proxy->getLaserData();
        // Simplify laser contour with Ramer-Douglas-Peucker
        poly_robot = ramer_douglas_peucker(ldata, MAX_RDP_DEVIATION_mm);
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
    laser_polygon = viewer_robot->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
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
    // Check if x,y,alpha are within ranges
    target.pos = QPointF(x, y);
    if( dimensions.contains(target.pos))
    {
        auto[_, __, adv, rot, ___] = dwa.compute(target_in_robot, laser_poly, global_advance,
                                                 global_rotation,
                                                 Eigen::Vector3f(r_state_global.x, r_state_global.y, r_state_global.rz),
                                                 &viewer_robot->scene);
        float dist_break = std::clamp(target_in_robot.norm() / 1000.0, 0.0, 1.0);
        float adv_n = constants.max_advance_speed * dist_break * gaussian(global_rotation);
        RoboCompMoveTowards::Command command{ adv_n, rot};
        return command;
    }
    else
        qInfo() << __FUNCTION__  << "Target outside boundaries";
}

