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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <Eigen/Dense>
//#include <acado_toolkit.hpp>
//#include <bindings/acado_gnuplot/gnuplot_window.hpp>
#include <casadi/casadi.hpp>
#include <casadi/core/optistack.hpp>

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
	bool startup_check_flag;
    std::tuple<QPolygonF, QPolygonF> read_laser(const RoboCompGenericBase::TBaseState &pose);

    void initialize_differential(const std::vector<double> &target_robot, const std::vector<double> &init_robot);
    std::vector<std::tuple<double, double, double>> points_to_lines(const std::vector<Eigen::Vector2d> &points_in_robot);

};

#endif
