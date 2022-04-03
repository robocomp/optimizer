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
#include "specificworker.h"
#include <cppitertools/range.hpp>
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
    auto left_x = std::stod(params.at("left_x").value);
    auto top_y = std::stod(params.at("top_y").value);
    auto width = std::stod(params.at("width").value);
    auto height = std::stod(params.at("height").value);
    auto tile = std::stod(params.at("tile").value);
    qInfo() << __FUNCTION__ << " Read parameters: " << left_x << top_y << width << height << tile;
    this->dimensions = QRectF(left_x, top_y, width, height);
    constants.tile_size = tile;
    return true;
}
void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    viewer = new AbstractGraphicViewer(this->beta_frame, this->dimensions);
    this->resize(900,450);
    const auto &[rp, re] = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH);
    robot_polygon = rp;
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}
void SpecificWorker::compute()
{
    read_laser();
    robot_pose = read_robot();
    //read_camera();
    if(target.active)
    {
        auto lpath = grid.computePath(e2q(from_world_to_grid(robot_pose.pos)),
                                                   e2q(from_world_to_grid(target.to_eigen())));
        std::vector<Eigen::Vector2f> path;
        auto g2r = from_grid_to_robot_matrix();
        path.resize(lpath.size());
        for (auto &&[i, p]: lpath | iter::enumerate)
            path[i] = (g2r * Eigen::Vector3f(p.x(), p.y(), 1.f)).head(2);
        draw_path(lpath);
        goto_target(path);
    }
    else
        qInfo() << __FUNCTION__ << "IDLE";
}
/////////////////////////////////////////////////////////////////////////

void SpecificWorker::goto_target(const std::vector<Eigen::Vector2f> &path)  //path in robot RS
{
    auto exit = [this]()
                {
                    try
                    { differentialrobot_proxy->setSpeedBase(0, 0); }
                    catch (const Ice::Exception &e)
                    { std::cout << e.what() << std::endl; }
                    target.active = false;
                    grid.clear();
                    qInfo() << __FUNCTION__ << "Target reached";
                };

    if (not path.empty())
    {
        Eigen::Vector2f target_r;
        if(path.size() > 5)
            target_r = path[5];
        else
            target_r = path.back();
        float dist = target_r.norm();
        if(dist > constants.min_dist_to_target)
        {
            float beta = atan2(target_r.x(), target_r.y());
            float k2 = 1.f;
            float f1 = std::clamp(dist / 1000, 0.f, 1.f);
            float f2 = exp(-beta * beta);
            try
            { differentialrobot_proxy->setSpeedBase(1000 * f1 * f2, k2 * beta); }
            catch (const Ice::Exception &e)
            { std::cout << e.what() << std::endl; }
        }
        else exit();
    }
    else
        exit();
}
void SpecificWorker::read_laser()
{
    try
    {
        auto ldata = laser_proxy->getLaserData();
        draw_laser( ldata );
        if(target.active)
            update_map(ldata);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
}
void SpecificWorker::read_camera()
{
    try
    {
        cv::Mat top_img_uncomp;
        QImage top_qimg;
        auto top_img = camerasimple_proxy->getImage();
        if(not top_img.image.empty())
        {
            if (top_img.compressed)
            {
                top_img_uncomp = cv::imdecode(top_img.image, -1);
                top_qimg = QImage(top_img_uncomp.data, top_img.width, top_img.height, QImage::Format_RGB888).scaled(
                        top_camera_label->width(), top_camera_label->height(), Qt::KeepAspectRatioByExpanding);;
            } else
                top_qimg = QImage(&top_img.image[0], top_img.width, top_img.height, QImage::Format_RGB888).scaled(
                        top_camera_label->width(), top_camera_label->height(), Qt::KeepAspectRatioByExpanding);;
            auto pix = QPixmap::fromImage(top_qimg);
            top_camera_label->setPixmap(pix);
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
}
SpecificWorker::Pose2D SpecificWorker::read_robot()
{
    RoboCompFullPoseEstimation::FullPoseEuler bState;
    Pose2D rp;
    try
    {
        bState = fullposeestimation_proxy->getFullPoseEuler();
        rp = {.ang=bState.rz, .pos=Eigen::Vector2f(bState.x, bState.y)};
        //qInfo()  << bState.x << bState.y << bState.rz;
        robot_polygon->setRotation(bState.rz*180/M_PI);
        robot_polygon->setPos(bState.x, bState.y);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    return rp;
}
Eigen::Vector2f SpecificWorker::from_robot_to_world(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(robot_pose.ang) , -sin(robot_pose.ang) , sin(robot_pose.ang) , cos(robot_pose.ang);
    return (matrix * p) + robot_pose.pos;
}
Eigen::Vector2f SpecificWorker::from_world_to_robot(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(robot_pose.ang) , -sin(robot_pose.ang) , sin(robot_pose.ang) , cos(robot_pose.ang);
    return (matrix.transpose() * (p - robot_pose.pos));
}
Eigen::Vector2f SpecificWorker::from_grid_to_world(const Eigen::Vector2f &p)
{
    // build the matrix to transform from grid to world knowing robot and grid pose in world
    Eigen::Matrix2f g2w;
    g2w <<  cos(grid_world_pose.ang), -sin(grid_world_pose.ang),
            sin(grid_world_pose.ang), cos(grid_world_pose.ang);
    return g2w * p + grid_world_pose.pos;
}
Eigen::Vector2f SpecificWorker::from_world_to_grid(const Eigen::Vector2f &p)
{
    // build the matrix to transform from world to local_grid, knowing robot and grid pose in world
    Eigen::Matrix2f w2g;
    w2g <<  cos(grid_world_pose.ang), sin(grid_world_pose.ang),
            -sin(grid_world_pose.ang), cos(grid_world_pose.ang);
    return w2g * (p - grid_world_pose.pos);
}
Eigen::Matrix3f SpecificWorker::from_robot_to_grid_matrix()
{
    // build the matrix to transform from robot to local_grid, knowing robot and grid pose in world
    Eigen::Matrix3f r2w;
    r2w <<  cos(robot_pose.ang), -sin(robot_pose.ang), robot_pose.pos.x(),
            sin(robot_pose.ang) , cos(robot_pose.ang), robot_pose.pos.y(),
            0.f, 0.f, 1.f;
    Eigen::Matrix2f w2g_2d_matrix;
    w2g_2d_matrix <<  cos(grid_world_pose.ang), sin(grid_world_pose.ang),
            -sin(grid_world_pose.ang), cos(grid_world_pose.ang);
    auto tr = w2g_2d_matrix * grid_world_pose.pos;
    Eigen::Matrix3f w2g;
    w2g << cos(grid_world_pose.ang), sin(grid_world_pose.ang), -tr.x(),
            -sin(grid_world_pose.ang), cos(grid_world_pose.ang), -tr.y(),
            0.f, 0.f, 1.f;
    Eigen::Matrix3f r2g = w2g * r2w;  // from r to world and then from world to grid
    return r2g;
}
Eigen::Matrix3f SpecificWorker::from_grid_to_robot_matrix()
{
    return from_robot_to_grid_matrix().inverse();
}
/////////////////////////////////////////////////////////////////////////

void SpecificWorker::draw_path(const std::list<QPointF> &path)
{
    static std::vector<QGraphicsItem *> path_paint;
    static QString path_color = "Green";

    for(auto p : path_paint)
        viewer->scene.removeItem(p);
    path_paint.clear();

    uint s = 100;
    for(auto &&p : path)
    {
        auto pw = from_grid_to_world(Eigen::Vector2f(p.x(),p.y()));  // in mm
        path_paint.push_back(viewer->scene.addEllipse(pw.x()-s/2, pw.y()-s/2, s , s, QPen(path_color), QBrush(QColor(path_color))));
        path_paint.back()->setZValue(30);
    }
}
void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer->scene.removeItem(laser_polygon);

    QPolygonF poly;
    poly << QPointF(0,0);
    for(auto &&l : ldata)
        poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
    poly.pop_back();

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}
void SpecificWorker::new_target_slot(QPointF t)
{
    qInfo() << __FUNCTION__ << " Received new target at " << t;
    target.pos = t;
    target.active = true;
    // create local grid for mission
    Eigen::Vector2f t_r= from_world_to_robot(target.to_eigen());
    float dist_to_robot = t_r.norm();
    //    qInfo() << __FUNCTION__ << dist_to_robot_1 << dist_to_robot << dist_to_robot_2;
    QRectF dim(-2000, -500, 4000, dist_to_robot+1000);
    grid_world_pose = {.ang=-atan2(t_r.x(), t_r.y()) + robot_pose.ang, .pos=robot_pose.pos};
    grid.initialize(dim, constants.tile_size, &viewer->scene, false, std::string(),
                    grid_world_pose.toQpointF(), grid_world_pose.ang);
    qInfo() << __FUNCTION__ << " Initial grid pos:" << grid_world_pose.pos.x() << grid_world_pose.pos.y() << grid_world_pose.ang;

}
void SpecificWorker::update_map(const RoboCompLaser::TLaserData &ldata)
{
    // get the matrix to transform from robot to local_grid
    Eigen::Matrix3f r2g = from_robot_to_grid_matrix();

    for(const auto &l : ldata)
    {
        if(l.dist > constants.robot_semi_length)
        {
            Eigen::Vector3f tip(l.dist*sin(l.angle), l.dist*cos(l.angle), 1.f);
            // transform tip form robot's RS to local_grid RS
            tip = r2g * tip;
            int target_kx = (tip.x() - grid.dim.left()) / grid.TILE_SIZE;
            int target_kz = (tip.y() - grid.dim.top()) / grid.TILE_SIZE;
            int last_kx = std::numeric_limits<int>::min();
            int last_kz = std::numeric_limits<int>::min();

            int num_steps = ceil(l.dist/(constants.tile_size/2.0));
            for(const auto &&step : iter::range(0.0, 1.0-(1.0/num_steps), 1.0/num_steps))
            {
                Eigen::Vector3f p = tip*step;
                int kx = (p.x() - grid.dim.left()) / grid.TILE_SIZE;
                int kz = (p.y() - grid.dim.top()) / grid.TILE_SIZE;
                if(kx != last_kx and kx != target_kx and kz != last_kz and kz != target_kz)
                    grid.add_miss(p.head(2));
                last_kx = kx;
                last_kz = kz;
            }
            if(l.dist <= constants.max_laser_range)
                grid.add_hit(tip.head(2));
            // else
            //     grid.add_miss(from_robot_to_world(tip));
        }
    }
}
////////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompJointMotorSimple you can call this methods:
// this->jointmotorsimple_proxy->getMotorParams(...)
// this->jointmotorsimple_proxy->getMotorState(...)
// this->jointmotorsimple_proxy->setPosition(...)
// this->jointmotorsimple_proxy->setVelocity(...)
// this->jointmotorsimple_proxy->setZeroPos(...)

/**************************************/
// From the RoboCompJointMotorSimple you can use this types:
// RoboCompJointMotorSimple::MotorState
// RoboCompJointMotorSimple::MotorParams
// RoboCompJointMotorSimple::MotorGoalPosition
// RoboCompJointMotorSimple::MotorGoalVelocity

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

//    float dist_to_robot = target.dist_to_robot(std::bind(&SpecificWorker::from_world_to_robot, this, std::placeholders::_1));
//    float dist_to_robot_2 = target.dist_to_robot(quick_bind(&SpecificWorker::from_world_to_robot, this));