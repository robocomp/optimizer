#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
import random

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
import casadi as ca
import sys, time
from loguru import logger
import matplotlib.pyplot as plt
from rdp import rdp
from ground.base import get_context
from sect.triangulation import Triangulation

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.N = 12  # epoch size
        self.align_point = self.N//3
        self.target_pose = [0, 1]
        self.initialize_differential()
#        self.initialize_omni()

        p_opts = {}
        s_opts = {"max_iter": 10000,
                  'print_level': 0,
                  'acceptable_tol': 1e-8,
                  'acceptable_obj_change_tol': 1e-6}
        self.opti.solver("ipopt", p_opts, s_opts)  # set numerical backend
        self.sol = None
        self.active = True
        self.ant_dist_to_target = 0

        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        x = np.linspace(-5000, 5000, self.N+1)
        y = np.linspace(-2500, 2500, self.N+1)
        self.line_plt, = self.ax.plot(x, y, '*')
        #self.line_plt = self.ax.quiver(x, y, self.u, self.v)
        self.line_laser, = self.ax.plot(x, y)
        target_circle = plt.Circle((self.target_pose[0] * 1000, self.target_pose[1] * 1000), 100, color='blue')
        self.origin_circle = self.ax.add_patch(target_circle)
        robot_square = plt.Rectangle((self.target_pose[0] * 1000, self.target_pose[1] * 1000), 400, 400, color='green')
        self.robot_square = self.ax.add_patch(robot_square)
        self.ax.plot()
        plt.show()

        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            #self.timer.setSingleShot(True)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        return True

    @QtCore.Slot()
    @logger.catch
    def compute(self):
        print("------------------------")
        try:
            #currentPose = self.omnirobot_proxy.getBaseState()
            currentPose = self.differentialrobot_proxy.getBaseState()
            current_tr = np.array([currentPose.x/1000, currentPose.z/1000])
        except:
            print("Error connecting to base")

        # laser
        laser_poly_robot, laser_poly_world = self.read_laser(currentPose)

        start = time.time()
        dist_to_target = np.linalg.norm(current_tr - self.target_pose)
        der_to_target = dist_to_target - self.ant_dist_to_target
        print(f"Dist to target: {dist_to_target:.2f}, Dist ant:, {self.ant_dist_to_target:.2f}, Der:, {der_to_target:.2f}")
        self.ant_dist_to_target = dist_to_target
        print(f"Current pose:, {currentPose.x:.2f}, {currentPose.z:.2f}, {currentPose.alpha:.2f}")
        print(f"Target pose:, {self.target_pose[0]:.2f}, {self.target_pose[1]:.2f}")

        if self.active and dist_to_target > 0.15:

            # ---- initial values for state ---
            self.opti.set_value(self.initial_oparam[0], 0)
            self.opti.set_value(self.initial_oparam[1], 0)
            self.opti.set_value(self.initial_oparam[2], 0)
            R = np.array([[np.cos(currentPose.alpha), -np.sin(currentPose.alpha)],
                          [np.sin(currentPose.alpha), np.cos(currentPose.alpha)]])
            target_in_robot = R.transpose() @ (self.target_pose-current_tr)
            self.opti.set_value(self.target_oparam, target_in_robot)

            # Warm start
            if self.sol:
                for i in range(1, self.N):
                    self.opti.set_initial(self.X[:, i], self.sol.value(self.X[:, i]))

            # ---- solve NLP ------
            self.sol = self.opti.solve()

            # ---- print output -----
            print("Iterations:", self.sol.stats()["iter_count"])
            adv = self.sol.value(self.v_a[0]*1000)
            rot = self.sol.value(self.v_rot[0])
            self.advance = self.sol.value(self.v_a*1000)
            self.rotation = self.sol.value(self.v_rot)

            # update self.vect to the desired rotation value at point P
            vect = self.sol.value(self.X[0:2, self.align_point+1]) - self.sol.value(self.X[0:2, self.align_point-1])
            end = time.time()
            print(f"Elapsed: {end-start:.2f}")
            # move the robot
            try:
                # self.omnirobot_proxy.setSpeedBase( self.sol.value(self.v_x[0]*1000),
                #                                    self.sol.value(self.v_y[0]*1000),
                #                                    self.sol.value(self.v_rot[0]))
                print("Control", adv, rot)
                self.differentialrobot_proxy.setSpeedBase(adv, rot)
                pass
            except Exception as e:
                print(e)

            # draw
            xs = self.sol.value(self.pos_x * 1000)
            ys = self.sol.value(self.pos_y * 1000)
            path_in_world = R @ np.array([xs, ys])
            for i in range(path_in_world.shape[1]):
                path_in_world[:, i] = path_in_world[:, i] + current_tr*1000
            self.line_plt.set_xdata(path_in_world[0, :])
            self.line_plt.set_ydata(path_in_world[1, :])
            # draw laser
            self.line_laser.set_xdata(laser_poly_world[:, 0])
            self.line_laser.set_ydata(laser_poly_world[:, 1])
            # draw convex polys

            # draw robot
            self.robot_square.remove()
            self.robot_square = self.ax.add_patch(plt.Rectangle(current_tr*1000-[150, 250], 300, 500,
                                                                color='green', angle=np.rad2deg(currentPose.alpha)))
            # self.ax.clear()
            # target_circle = plt.Circle((self.target_pose[0] * 1000, self.target_pose[1] * 1000), 100, color='blue')
            # self.ax.add_patch(target_circle)
            # avs = self.sol.value(self.v_a*1000)
            # rs = self.sol.value(self.v_rot)
            # np.savetxt('controls.csv', np.column_stack([avs, rs]), delimiter=',')
            # u = avs * np.sin(rs)
            # u = np.append(u, 100)
            # v = avs * np.cos(rs)
            # v = np.append(v, 100)
            # self.ax.quiver( self.sol.value(self.pos_x*1000),
            #                 self.sol.value(self.pos_y*1000),
            #                 u, v, scale = None)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        else:   # at target
            try:
                #self.omnirobot_proxy.setSpeedBase(0, 0, 0)
                self.differentialrobot_proxy.setSpeedBase(0, 0)
                self.active = False
                print("Stopping")
                sys.exit(0)
            except Exception as e: print(e)

        #plot(sol.value(self.v_x), label="vx")
        #plot(sol.value(self.v_y), label="vy")
        #plot(sol.value(self.v_rot), label="vrot")
        #plt.scatter(sol.value(self.pos_x*1000), sol.value(self.pos_y*1000))

        #plt.plot(sol.value(self.v_x * 1000), sol.value(self.v_y * 1000), '->')
        #plot(sol.value(self.pos_x), label="px")
        #plot(sol.value(self.pos_y), label="py")
        #legend(loc="upper left")
        #show()

    def read_laser(self, rpose):
        ldata_robot = [[0, 0]]
        ldata_world = []
        try:
            ld = self.laser_proxy.getLaserData()
            # laser tips in robot's RS
            ldata_robot.extend([[l.dist*np.sin(l.angle), l.dist*np.cos(l.angle)] for l in ld if l.dist > 0])
            ldata_robot.append([0,0])
            ldata_robot = np.array(rdp(ldata_robot, epsilon=100))
            # laser tips in world's RS
            a = rpose.alpha
            r = np.array([rpose.x, rpose.z])
            R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
            ldata_world = [R@l + r for l in ldata_robot]
            # convexify
            context = get_context()
            Contour, Point, Polygon = context.contour_cls, context.point_cls, context.polygon_cls
            poly = Polygon(Contour([Point(point[0], point[1]) for point in ldata_world]), [])
            res = Triangulation.constrained_delaunay(poly, context=context).triangles()
            #print("Convex polygons", len(res))
            for cont in res:
                for p in cont.vertices:
                    ldata_world.append([p.x, p.y])

        except Exception as e:
            print(e)
        return ldata_robot, np.array(ldata_world)

    #@logger.catch
    def initialize_omni(self):

        self.opti = ca.Opti()  # Optimization problem

        # ---- state variables ---------
        self.X = self.opti.variable(3, self.N + 1)  # state trajectory
        self.pos_x = self.X[0, :]
        self.pos_y = self.X[1, :]
        self.phi = self.X[2, :]

        # ---- inputs variables ---------
        self.U = self.opti.variable(3, self.N)  # control trajectory (throttle)
        self.v_x = self.U[0, :]
        self.v_y = self.U[1, :]
        self.v_rot = self.U[2, :]

        self.target_oparam = self.opti.parameter(2)
        self.initial_oparam = self.opti.parameter(3)

        # ---- cost function          ---------
        self.opti.set_value(self.target_oparam, self.target_pose)
        self.opti.set_value(self.initial_oparam, [0.0, 0.0, 0.0])
        sum_dist = self.opti.parameter()
        self.opti.set_value(sum_dist, 0)
        for k in range(self.N - 1):
            sum_dist += ca.sumsqr(self.X[0:2, k + 1] - self.X[0:2, k])

        self.opti.minimize(sum_dist + ca.sumsqr(self.v_x) + ca.sumsqr(self.v_y) + 0.1*ca.sumsqr(self.v_rot))

        # ---- dynamic constraints for omniwheeled robot --------
        # dx/dt = f(x, u)
        f = lambda x, u: ca.vertcat(ca.horzcat(ca.cos(x[2]), -ca.sin(x[2]), 0),
                                    ca.horzcat(ca.sin(x[2]), ca.cos(x[2]), 0),
                                    ca.horzcat(0, 0, 1)) @ u

        dt = 1 / self.N  # length of a control interval
        dt = 1
        for k in range(self.N):  # loop over control intervals
            # Runge-Kutta 4 integration
            k1 = f(self.X[:, k], self.U[:, k])
            k2 = f(self.X[:, k] + dt / 2 * k1, self.U[:, k])
            k3 = f(self.X[:, k] + dt / 2 * k2, self.U[:, k])
            k4 = f(self.X[:, k] + dt * k3, self.U[:, k])
            x_next = self.X[:, k] + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            #x_next = self.X[:, k] + k1
            self.opti.subject_to(self.X[:, k + 1] == x_next)  # close the gaps

        # ---- control constraints -----------
        self.opti.subject_to(self.opti.bounded(-0.5, self.v_x, 0.5))  # control is limited meters
        self.opti.subject_to(self.opti.bounded(-0.5, self.v_y, 0.5))  # control is limited
        self.opti.subject_to(self.opti.bounded(-1, self.v_rot, 1))  # control is limited

        # ---- differential drive contraint ----------
        self.opti.subject_to(self.v_x == 0)

        # ---- forward drive contraint ----------
        self.opti.subject_to(self.v_y >= 0)

        # ---- trajectory align contraint ----------
        self.vect = self.opti.parameter(2)
        self.opti.set_value(self.vect, [0, 1])
        #self.opti.subject_to(self.phi[self.align_point] == ca.atan2(self.vect[0], self.vect[1]))

        # ---- initial point constraints -----------
        self.opti.subject_to(self.pos_x[0] == self.initial_oparam[0])
        self.opti.subject_to(self.pos_y[0] == self.initial_oparam[1])
        self.opti.subject_to(self.phi[0] == self.initial_oparam[2])

        # ---- target point constraints -----------
        self.opti.subject_to(self.pos_x[-1] == self.target_oparam[0])
        self.opti.subject_to(self.pos_y[-1] == self.target_oparam[1])

    def initialize_differential(self):

        self.opti = ca.Opti()  # Optimization problem

        # ---- state variables ---------
        self.X = self.opti.variable(3, self.N + 1)  # state trajectory
        self.pos_x = self.X[0, :]
        self.pos_y = self.X[1, :]
        self.phi = self.X[2, :]

        # ---- inputs variables 2 adv and rot---------
        self.U = self.opti.variable(2, self.N)  # control
        self.v_a = self.U[0, :]
        self.v_rot = self.U[1, :]

        #self.T = self.opti.variable()  # final time
        self.target_oparam = self.opti.parameter(2)
        self.initial_oparam = self.opti.parameter(3)

        # ---- cost function          ---------
        self.opti.set_value(self.target_oparam, self.target_pose)
        self.opti.set_value(self.initial_oparam, [0.0, 0.0, 0.0])
        #self.opti.set_initial(self.T, 1)

        sum_dist = self.opti.parameter()
        self.opti.set_value(sum_dist, 0)
        for k in range(self.N - 1):
             sum_dist += ca.sumsqr(self.X[0:2, k + 1] - self.X[0:2, k])

        # self.opti.minimize(ca.sumsqr(self.X[0:2, -1] - self.target_oparam)
        #                    + 0.01 * ca.sumsqr(self.v_rot)
        #                    + ca.sumsqr(self.v_a))
        self.opti.minimize(0.001*ca.sumsqr(self.v_rot) + ca.sumsqr(self.v_a))
        self.opti.minimize( sum_dist + ca.sumsqr(self.v_rot) + ca.sumsqr(self.v_a))

        # ---- dynamic constraints for differential robot --------
        # dx/dt = f(x, u)   3 x 2 * 2 x 1 -> 3 x 1
        f = lambda x, u: ca.vertcat(ca.horzcat(ca.sin(x[2]), 0),
                                    ca.horzcat(ca.cos(x[2]), 0),
                                    ca.horzcat(0,            1)) @ u

        dt = 1.0 / self.N  # length of a control interval
        dt = 1   # timer interval in secs
        for k in range(self.N):  # loop over control intervals
            # Runge-Kutta 4 integration
            k1 = f(self.X[:, k], self.U[:, k])
            k2 = f(self.X[:, k] + dt / 2 * k1, self.U[:, k])
            k3 = f(self.X[:, k] + dt / 2 * k2, self.U[:, k])
            k4 = f(self.X[:, k] + dt * k3, self.U[:, k])
            #x_next = self.X[:, k] + dt/6  * (k1 + 2 * k2 + 2 * k3 + k4)
            x_next = self.X[:, k] + k1
            self.opti.subject_to(self.X[:, k + 1] == x_next)  # close the gaps

        # ---- control constraints -----------
        self.opti.subject_to(self.opti.bounded(-0.5, self.v_a, 0.5))  # control is limited meters
        self.opti.subject_to(self.opti.bounded(-1, self.v_rot, 1))  # control is limited

        # ---- initial point constraints -----------
        self.opti.subject_to(self.pos_x[0] == self.initial_oparam[0])
        self.opti.subject_to(self.pos_y[0] == self.initial_oparam[1])
        self.opti.subject_to(self.phi[0] == self.initial_oparam[2])

        # ---- target point constraints -----------
        self.opti.subject_to(self.pos_x[-1] == self.target_oparam[0])
        self.opti.subject_to(self.pos_y[-1] == self.target_oparam[1])

    ######################################################################################################
    def startup_check(self):
        print(f"Testing RoboCompLaser.LaserConfData from ifaces.RoboCompLaser")
        test = ifaces.RoboCompLaser.LaserConfData()
        print(f"Testing RoboCompLaser.TData from ifaces.RoboCompLaser")
        test = ifaces.RoboCompLaser.TData()
        print(f"Testing RoboCompOmniRobot.TMechParams from ifaces.RoboCompOmniRobot")
        test = ifaces.RoboCompOmniRobot.TMechParams()
        QTimer.singleShot(200, QApplication.instance().quit)



