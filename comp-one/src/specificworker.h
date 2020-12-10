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

#define NP 3 // Number of variables for the pose
#define NV 3 // Number of variables for the velocity

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
	QPolygonF draw_laser(const RoboCompLaser::TLaserData &ldata);
	QPolygonF read_laser();
	RoboCompGenericBase::TBaseState read_base();

	// robot
    std::tuple<float,float,float> state_change(const RoboCompGenericBase::TBaseState &bState, float delta_t);

    // target
    QPointF target;
	float target_ang;
	QVec rtarget;
	bool newTarget;
    DoubleBuffer<QPointF, QPointF> target_buffer;


    // path
    std::vector<QPointF> path;
    std::vector<QGraphicsEllipseItem *> path_paint;
    QString path_color = "#FF00FF";
    void draw_path();
    bool atTarget = true;

	// Grid
	Grid<> grid;
    MyScene scene;
    QGraphicsItem *robot_polygon = nullptr;
    QGraphicsItem *laser_polygon = nullptr;

    // Draw
    QCustomPlot custom_plot;
    QCPGraph *xGraph, *yGraph, *wGraph, *exGraph, *ewGraph;
    void init_drawing( Grid<>::Dimensions dim);
    float jadv = 0.0; float jrot = 0.0; float jside = 0.0;
    QGraphicsEllipseItem *target_draw = nullptr;

	// Model and optimizations
	GRBEnv *env;
	GRBModel *model;
	GRBVar *model_vars;
	GRBVar *pose_vars;
	GRBVar *vel_vars;
	GRBVar *sin_cos_vars;
	GRBQuadExpr obj;
    const float ROBOT_LENGTH = 400;
	void initialize_model();
	void optimize();
    void fill_grid(const QPolygonF &ldata);

};

#endif
