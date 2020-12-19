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

class MyScene : public QGraphicsScene
{
    Q_OBJECT
    signals:
        void new_target(QGraphicsSceneMouseEvent *);
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event)
    { emit new_target(event); }
};

using namespace std::literals;

typedef std::pair<double, double> Point;
class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);


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
        std::shared_ptr < InnerModel > innerModel;
        bool startup_check_flag;
        QPolygonF read_laser();
        RoboCompGenericBase::TBaseState read_base();

        // robot
        const float ROBOT_LENGTH = 400;
        void stop_robot();

        // target
        QPointF target;
        DoubleBuffer<QPointF, QPointF> target_buffer;

        // path
        void draw_path(const std::vector<QPointF> &path);
        bool atTarget = true;

        // Grid
        Grid<> grid;
        MyScene scene;
        QGraphicsItem *robot_polygon = nullptr;
        void fill_grid(const QPolygonF &ldata);

        // convex parrtitions
        using Line = std::vector<std::tuple<float, float, float>>;
        using Obstacles = std::vector<std::tuple<Line, QPolygonF>>;
        Obstacles compute_laser_partitions(QPolygonF  &laser_poly);
        void ramer_douglas_peucker(const vector<Point> &pointList, double epsilon, vector<Point> &out);

        // Model and optimizations
        constexpr static std::size_t STATE_DIM = 3; // Number of variables for the pose
        constexpr static std::size_t CONTROL_DIM = 3; // Number of variables for the velocity
        using ControlVector = Eigen::Matrix<float, CONTROL_DIM, 1>;
        using StateVector = Eigen::Matrix<float, STATE_DIM, 1>;
        uint NUM_STEPS = 6;
        std::vector<uint> STEPS{1,2,3,4,5};
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
        void initialize_model(const StateVector &target, const Obstacles &obstacles);
        void optimize(const StateVector &current_state,  const Obstacles &obstacles);

        // Draw
        QCustomPlot custom_plot;
        QCPGraph *xGraph, *yGraph, *wGraph, *exGraph, *ewGraph, *timeGraph;
        void init_drawing( Grid<>::Dimensions dim);
        QGraphicsEllipseItem *target_draw = nullptr;
        void draw_target(const RoboCompGenericBase::TBaseState &bState, QPointF t);
        void draw_laser(const QPolygonF &poly);
        void draw(const ControlVector &control, float pos_error, float rot_error, float time_elapsed);
        void draw_partitions(const Obstacles &obstacles, bool print=false);
        int cont=0;

};

#endif
