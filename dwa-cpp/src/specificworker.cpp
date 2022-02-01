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
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    auto [r_state, advance, rot] = read_base();
    auto laser_poly = read_laser();
//    if(target.active)
//    {
//        suto dist = target.norm();
//        if( dist > MAX_DISTANCE_TO_TARGET)
//            auto[_, __, adv, rot, ___] = dw.compute(tr, laser_poly, advance, rot, nullptr /*&viewer_robot->scene*/);
//    }
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
    return poly_robot;
}
QPolygonF SpecificWorker::ramer_douglas_peucker(const RoboCompLaser::TLaserData &ldata, double epsilon)
{
    if(ldata.size()<2)
    {
        qWarning() << __FUNCTION__ << "Not enough points to simplify";
        return QPolygonF();
    }
    std::vector<Point> pointList(ldata.size());
    for (auto &&[i, l] : ldata | iter::enumerate)
        pointList[i] = std::make_pair(l.dist * sin(l.angle), l.dist * cos(l.angle));
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

