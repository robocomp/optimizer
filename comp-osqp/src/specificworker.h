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
#include <QGraphicsView>
#include "grid.cpp"
#include "grid.h"
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <doublebuffer/DoubleBuffer.h>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QAbstractAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QGraphicsSceneMouseEvent>

using namespace QtCharts;


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

private:
    std::shared_ptr<InnerModel> innerModel;
    bool startup_check_flag;

    QPolygonF draw_laser(const RoboCompLaser::TLaserData &ldata);
    QPolygonF read_laser();
    RoboCompGenericBase::TBaseState read_base();

    // robot
    std::tuple<float, float, float> state_change(const RoboCompGenericBase::TBaseState &bState, float delta_t);
    const float ViriatoBase_WheelRadius = 76.2;
    const float ViriatoBase_DistAxes = 380.f;
    const float ViriatoBase_AxesLength = 422.f;

    // target
    DoubleBuffer<Eigen::Vector2f, Eigen::Vector2f> target_buffer;

    // path
    std::vector<QGraphicsEllipseItem *> path_paint;
    QString path_color = "#FF00FF";
    bool atTarget = true;

    void draw_path(const std::vector<QPointF> &path);

    // Grid
    Grid<> grid;
    MyScene scene;
    QGraphicsItem *robot_polygon = nullptr;
    QGraphicsItem *laser_polygon = nullptr;
    const float ROBOT_LENGTH = 400;
    void fill_grid(const QPolygonF &ldata);

    // Draw
    QChart chart;
    QChartView chartview;
    QLineSeries series_x, series_y;

    // optimizer
    OsqpEigen::Solver solver;
    // controller input and QPSolution vector
    Eigen::Vector2d ctr;
    Eigen::VectorXd QPSolution;
    int mpcWindow = 20;
    // allocate the dynamics matrices
    Eigen::Matrix<double, 2, 2> A;
    Eigen::Matrix<double, 2, 2> B;

    // allocate the constraints vector
    Eigen::Matrix<double, 2, 1> xMax;
    Eigen::Matrix<double, 2, 1> xMin;
    Eigen::Matrix<double, 2, 1> uMax;
    Eigen::Matrix<double, 2, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 2> Q;
    Eigen::DiagonalMatrix<double, 2> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 2, 1> x0;
    Eigen::Matrix<double, 2, 1> xRef;

    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    std::optional<Eigen::Matrix<double, 2, 1>> init_optmizer();
    void setInequalityConstraints(Eigen::Matrix<double, 2, 1> &xMax, Eigen::Matrix<double, 2, 1> &xMin,
                             Eigen::Matrix<double, 2, 1> &uMax, Eigen::Matrix<double, 2, 1> &uMin);
    void setDynamicsMatrices(Eigen::Matrix<double, 2, 2> &A, Eigen::Matrix<double, 2, 2> &B);
    void setWeightMatrices(Eigen::DiagonalMatrix<double, 2> &Q, Eigen::DiagonalMatrix<double, 2> &R);
    void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 2> &Q, const Eigen::DiagonalMatrix<double, 2> &R,
                            int mpcWindow, Eigen::SparseMatrix<double> &hessianMatrix);
    void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 2> &Q, const Eigen::Matrix<double, 2, 1> &xRef, int mpcWindow, Eigen::VectorXd &gradient);
    void updateConstraintVectors(const Eigen::Matrix<double, 2, 1> &x0, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
    void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 2, 2> &dynamicMatrix,
                                     const Eigen::Matrix<double, 2, 2> &controlMatrix,
                                     int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix);
    void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 2, 1> &xMax, const Eigen::Matrix<double, 2, 1> &xMin,
                                      const Eigen::Matrix<double, 2, 1> &uMax, const Eigen::Matrix<double, 2, 1> &uMin,
                                      const Eigen::Matrix<double, 2, 1> &x0,
                                      int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
    double getErrorNorm(const Eigen::Matrix<double, 2, 1> &x, const Eigen::Matrix<double, 2, 1> &xRef);

};


#endif
