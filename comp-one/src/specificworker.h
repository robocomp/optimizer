/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
#include <innermodel/innermodel.h>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include "grid.cpp"
#include "grid.h"
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include "qcustomplot.h"
#include <doublebuffer/DoubleBuffer.h>
#include "polypartition.h"
#include "callback.h"

class MyScene : public QGraphicsScene
{
    Q_OBJECT
    signals:
        void new_target(QGraphicsSceneMouseEvent *);
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event)
    { emit new_target(event); }
};

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

        struct RobotPose
        {
            QVector2D pos;
            float ang;
        };

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    protected:
        void resizeEvent(QResizeEvent * event)
        {
            custom_plot.resize(signal_frame->size());
            graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio);
        }
        void closeEvent(QCloseEvent *event)
        {
            QSettings settings("reaffer Soft", "reafferApp");
            settings.beginGroup("MainWindow");
            settings.setValue("size", size());
            settings.setValue("pos", pos());
            settings.endGroup();
            event->accept();
        }
        void readSettings()
        {
            QSettings settings("reaffer Soft", "reafferApp");
            settings.beginGroup("MainWindow");
            resize(settings.value("size", QSize(400, 400)).toSize());
            move(settings.value("pos", QPoint(200, 200)).toPoint());
            settings.endGroup();
        }

    private:
        // constants
        const float MAX_SPIKING_ANGLE_rads = 0.2;
        const float MAX_RDP_DEVIATION_mm  =  70;
        const uint NUM_STEPS = 16;
        float MAX_ADV_SPEED = 1000;
        float MAX_ROT_SPEED = 1;
        float MAX_SIDE_SPEED = 500;

        constexpr static std::size_t STATE_DIM = 3; // Number of variables for the pose
        constexpr static std::size_t CONTROL_DIM = 3; // Number of variables for the velocity

        // general
        std::shared_ptr < InnerModel > innerModel;
        bool startup_check_flag;
        std::tuple<QPolygonF, RoboCompLaser::TLaserData> read_laser();
        RoboCompGenericBase::TBaseState read_base();

        // robot
        const float ROBOT_LENGTH = 505;
        const float ROBOT_WIDTH = 375;
        void stop_robot();
        QPolygonF robot_polygon_extended;
        int OFFSET = 100;  // virtual bumper

        // target
        QPointF target;
        DoubleBuffer<QPointF, QPointF> target_buffer;

        // path
        void draw_path(const std::vector<QPointF> &path);
        std::vector<QPointF> compute_path();
        bool atTarget = true;

        // Grid
        Grid<>::Dimensions dim;  //default values//
        Grid<> grid;
        void fill_grid(const QPolygonF &ldata);

        // map
        std::vector<QPolygonF> read_map_obstacles();
        std::vector<QPolygonF> map_obstacles;

        // convex parrtitions
        using Point = std::pair<float, float>;  //only for RDP, change to QPointF
        using Lines = std::vector<std::tuple<float, float, float>>;
        using Obstacles = std::vector<std::tuple<Lines, QPolygonF>>;
        std::vector<tuple<Lines, QPolygonF>> world_free_regions;

        //controller
        float exponentialFunction(float value, float xValue, float yValue, float min);
        float rewrapAngleRestricted(const float angle);
        void local_controller(const std::vector<QPointF> &path, const RoboCompLaser::TLaserData &laser_data, const QPointF &robot, const QPointF &target);
        void local_controller(float side_vel, float adv_vel, float rot_vel, const RoboCompLaser::TLaserData &laser_data);

        // threads
        std::thread opt_thread;

        Obstacles compute_laser_partitions(QPolygonF  &laser_poly);
        Obstacles compute_external_partitions(Grid<>::Dimensions dim, const std::vector<QPolygonF> &map_obstacles, const QPolygonF &laser_poly, QGraphicsItem* robot_polygon);
        void ramer_douglas_peucker(const vector<Point> &pointList, double epsilon, vector<Point> &out);

        // Model and optimizations
        using ControlVector = Eigen::Matrix<float, CONTROL_DIM, 1>;
        using StateVector = Eigen::Matrix<float, STATE_DIM, 1>;
        GRBEnv env;
        GRBModel *model;
        GRBVar *model_vars;
        GRBVar *state_vars;
        GRBVar *control_vars;
        GRBQuadExpr obj;
        struct ObsData
        {
            struct PolyData
            {
                std::vector<GRBVar> line_vars;
                std::vector<GRBGenConstr> line_constraints;
                GRBVar and_var;
                GRBGenConstr and_constraint;
            };
            std::vector<PolyData> pdata;
            GRBVar or_var;
            GRBGenConstr or_constraint;
            GRBConstr final_constraint;
            void clear(GRBModel *model)
            {
                model->remove(or_var);
                model->remove(or_constraint);
                model->remove(final_constraint);
                for(auto &p : pdata)
                {
                    model->remove(p.and_var);
                    model->remove(p.and_constraint);
                    for(auto &lv : p.line_vars)
                        model->remove(lv);
                    p.line_vars.clear();
                    for(auto &lc : p.line_constraints)
                        model->remove(lc);
                    p.line_constraints.clear();
                }
                pdata.clear();
            }
        };
        std::vector<ObsData> obs_contraints;
        GRBVar *sin_cos_vars;
        void initialize_model(const StateVector &target);
        void optimize(const StateVector &target_state,  const Obstacles &obstacles, const std::vector<QPointF> &path);
        Callback *callback;

        // Draw
        QCustomPlot custom_plot;
        QCPGraph *xGraph, *yGraph, *wGraph, *exGraph, *ewGraph, *timeGraph;
        MyScene scene;
        QGraphicsItem *robot_polygon = nullptr;
        void init_drawing( Grid<>::Dimensions dim);
        QGraphicsEllipseItem *target_draw = nullptr;
        void draw_target(const RoboCompGenericBase::TBaseState &bState, QPointF t);
        void draw_laser(const QPolygonF &poly);
        void draw_signals(const ControlVector &control, float pos_error, float rot_error, float time_elapsed);
        void draw_partitions(const Obstacles &obstacles, const QColor &color, bool print=false);
        int cont=0;

};

#endif
