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
#include <cppitertools/chunked.hpp>


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

    // MPC
    mpc.initialize_differential(constants.num_steps_mpc);

    // mouse clicking
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}
void SpecificWorker::compute()
{
    static std::vector<Eigen::Vector2d> path_ant;
    auto ldata = read_laser(false);
    robot_pose = read_robot();
    // Bill
    read_bill(robot_pose);  // sets target at 1m from Bill

    //read_camera();
    if(target.active)
    {
        auto path = grid.compute_path(e2q(from_world_to_grid(robot_pose.pos)), e2q(from_world_to_grid(target.to_eigen())));
        auto target_r = from_world_to_robot(target.to_eigen());
        if(target_r.norm() < 100)
        {
            move_robot(0,0);
            return;
        }
        if(path.size() < constants.num_steps_mpc)
        {
            float adv = std::clamp(target_r.norm(), 0.f, 500.f);
            float rot = atan2(target_r.x(), target_r.y());
            move_robot(adv, rot);
        }
        else
        {
            auto g2r = from_grid_to_robot_matrix();
            std::vector<Eigen::Vector2d> path_robot_meters(path.size());
            for (auto &&[i, p]: path | iter::enumerate)
            {
                p = (g2r * Eigen::Vector3f(p.x(), p.y(), 1.f)).head(2);
                path_robot_meters[i] = Eigen::Vector3d(p.x(), p.y(), 1.f).head(2) / 1000.0;  // meters
            }
            draw_path(path);
            //goto_target_carrot(path);
            goto_target_mpc(path_robot_meters, ldata);
        }
    }
    else
        qInfo() << __FUNCTION__ << "IDLE";
}
/////////////////////////////////////////////////////////////////////////
void SpecificWorker::goto_target_mpc(const std::vector<Eigen::Vector2d> &path_robot, const RoboCompLaser::TLaserData &ldata)  //path in robot RS
{
    // lambda para unificar las dos salidas de los if
    auto exit = [this, path_robot]()
    {
            try
            { differentialrobot_proxy->setSpeedBase(0, 0); }
            catch (const Ice::Exception &e)
            { std::cout << e.what() << std::endl; }
            target.active = false;
            qInfo() << __FUNCTION__ << "Target reached";
    };

    if(auto r = mpc.minimize_balls_path(path_robot, robot_pose.to_vec3_meters(), ldata); r.has_value())
    {
        auto [advance, rotation, solution, balls] = r.value();
        try
        {
            // move the robot
            move_robot(advance * gaussian(rotation), rotation);
            qInfo() << __FUNCTION__ << "Adv: " << advance << "Rot:" << rotation;

            // draw
            auto path = std::vector<double>(solution.value(mpc.pos));  //in meters
            draw_solution_path(path, balls);
        }
        catch (...)
        { std::cout << "No solution found" << std::endl; }
    }
    else // do something to avoid go blind
    {
        move_robot(0, 0);
        robot_polygon->setBrush(QColor("red"));
    }
}

void SpecificWorker::goto_target_carrot(const std::vector<Eigen::Vector2f> &path_robot)  //path in robot RS
{
    // lambda para unificar las dos salidas de los if
    auto exit = [this]()
                {
                    try
                    { differentialrobot_proxy->setSpeedBase(0, 0); }
                    catch (const Ice::Exception &e)
                    { std::cout << e.what() << std::endl; }
                    target.active = false;
                    qInfo() << __FUNCTION__ << "Target reached";
                };

    if (not path_robot.empty())
    {
        Eigen::Vector2f target_r;
        if(path_robot.size() > 5)
            target_r = path_robot[5];
        else
            target_r = path_robot.back();
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
    {
        exit();
        return;
    }
}
RoboCompLaser::TLaserData SpecificWorker::read_laser(bool noise)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::normal_distribution<double> normal_dist(0.0, constants.lidar_noise_sigma);
    RoboCompLaser::TLaserData ldata;
    try
    {
        ldata = laser_proxy->getLaserData();

        // add radial noise la ldata
        if(noise)
            for(auto &l : ldata)
                l.dist += normal_dist(mt);

        // get n random angles to apply hard noise on them
        static std::uniform_int_distribution<int> unif_dist(0, ldata.size());
        static std::uniform_int_distribution<int> accept_dist(0, 10);
        static auto generator = std::bind(unif_dist, mt);
        std::vector<int> samples(constants.num_lidar_affected_rays_by_hard_noise, unif_dist(mt));
        std::generate_n(samples.begin(), constants.num_lidar_affected_rays_by_hard_noise, generator);
        for(auto &s: samples)
            if(accept_dist(mt) < 3)
                ldata[s].dist /= 3;

        draw_laser( ldata );
        if(target.active)
            update_map(ldata);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    return ldata;
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
bool SpecificWorker::read_bill(const Pose2D &robot_pose)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::normal_distribution<double> normal_dist(0.0, constants.target_noise_sigma);

    try
    {
        auto pose = billcoppelia_proxy->getPose();
        QLineF r_to_target(QPointF(pose.x, pose.y), QPointF(robot_pose.pos.x(), robot_pose.pos.y()));
        auto t = r_to_target.pointAt(constants.final_distance_to_target / r_to_target.length());
        target.set_pos(t + QPointF(normal_dist(mt), normal_dist(mt)));              // Adding noise to target
        if(target.draw != nullptr) viewer->scene.removeItem(target.draw);
        target.active = true;
        target.draw = viewer->scene.addEllipse(target.get_pos().x()-100, target.get_pos().y()-100, 200, 200, QPen(QColor("magenta")), QBrush(QColor("magenta")));

        // create local grid for mission
        // if new target has changed enough, replace local grid
        QPointF t_in_grid = e2q(from_world_to_grid(target.to_eigen()));
        auto r = grid.dim.adjusted(-grid.dim.left()*0.2, -grid.dim.top()*0.2, -grid.dim.right()*0.2, -grid.dim.bottom()*0.2);
        if( not r.contains(t_in_grid))
        {
            Eigen::Vector2f t_r = from_world_to_robot(target.to_eigen());
            float dist_to_robot = t_r.norm();
            //    qInfo() << __FUNCTION__ << dist_to_robot_1 << dist_to_robot << dist_to_robot_2;
            QRectF dim(-2000, -500, 4000, dist_to_robot + 2000);
            grid_world_pose = {.ang=-atan2(t_r.x(), t_r.y()) + robot_pose.ang, .pos=robot_pose.pos};
            grid.initialize(dim, constants.tile_size, &viewer->scene, false, std::string(),
                            grid_world_pose.toQpointF(), grid_world_pose.ang);
        }
    }
    catch(const Ice::Exception &e)
    {
        qInfo() << "Error connecting to Bill Coppelia";
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
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

void SpecificWorker::draw_path(const std::vector<Eigen::Vector2f> &path_in_robot)
{
    static std::vector<QGraphicsItem *> path_paint;
    static QString path_color = "Green";

    for(auto p : path_paint)
        viewer->scene.removeItem(p);
    path_paint.clear();

    uint s = 100;
    for(auto &&p : path_in_robot)
    {
        auto pw = from_robot_to_world(p);  // in mm
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
void SpecificWorker::draw_solution_path(const std::vector<double> &path, const mpc::MPC::Balls &balls)
{
    static std::vector<QGraphicsItem *> path_paint, ball_paint, ball_grads, ball_centers;
    static QString path_color = "Magenta";
    static QString ball_color = "LightBlue";

    for(auto p : path_paint)
        viewer->scene.removeItem(p);
    path_paint.clear();
    for(auto p : ball_paint)
        viewer->scene.removeItem(p);
    ball_paint.clear();
    for(auto p : ball_grads)
        viewer->scene.removeItem(p);
    ball_grads.clear();
    for(auto p : ball_centers)
        viewer->scene.removeItem(p);
    ball_centers.clear();

    uint s = 100;
    std::vector<Eigen::Vector2f> qpath(path.size()/2);
    std::vector<Eigen::Vector2f> path_centers;
    for(auto &&[i, p] : iter::chunked(path, 2) | iter::enumerate)
    {
        auto pw = from_robot_to_world(Eigen::Vector2f(p[0], p[1])*1000);  // in mm
        path_centers.push_back(pw);
        path_paint.push_back(viewer->scene.addEllipse(pw.x()-s/2, pw.y()-s/2, s , s, QPen(path_color), QBrush(QColor(path_color))));
        path_paint.back()->setZValue(30);
    }
    uint i=0;
    auto balls_temp = balls;
    balls_temp.erase(balls_temp.begin());
    for(auto &[center, r, grad] : balls_temp)
    {
        auto bc = from_robot_to_world(center.cast<float>()*1000);
        auto nr = r*1000;
        ball_paint.push_back(viewer->scene.addEllipse(bc.x()-nr, bc.y()-nr, nr*2 , nr*2, QPen(QBrush("DarkBlue"),15), QBrush(QColor(ball_color))));
        ball_paint.back()->setZValue(15);
        ball_paint.back()->setOpacity(0.07);

        // grads
        if(i < path_centers.size())
        {
            ball_grads.push_back(viewer->scene.addLine(path_centers[i].x(), path_centers[i].y(), bc.x(), bc.y(), QPen(QBrush("Magenta"), 20)));
            i++;
        }

        // centers
        //ball_centers.push_back(viewer_robot->scene.addEllipse(bc.x()-20, bc.y()-20, 40, 40, QPen(QColor("Magenta")), QBrush(QColor("Magenta"))));
        //ball_centers.back()->setZValue(30);
    }

}
void SpecificWorker::new_target_slot(QPointF t)
{
    qInfo() << __FUNCTION__ << " Received new target at " << t;
    target.set_pos(t);
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

            int num_steps = ceil(l.dist/(constants.tile_size/3.0));
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
void SpecificWorker::move_robot(float adv, float rot, float side)
{
    try
    {
        differentialrobot_proxy->setSpeedBase(adv, rot);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
}
float SpecificWorker::gaussian(float x)
{
    const double xset = constants.xset_gaussian;
    const double yset = constants.yset_gaussian;
    const double s = -xset*xset/log(yset);
    return exp(-x*x/s);
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