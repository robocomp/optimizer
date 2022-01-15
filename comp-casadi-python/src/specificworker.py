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
from pylab import plot, step, figure, legend, show, spy
import matplotlib.pyplot as plt

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.N = 12  # epoch size
        self.align_point = self.N//3
        self.target_pose = [0, 1]
#        self.initialize_differential()
        self.initialize_omni()

        # p_opts = {"expand": True}
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
        x = np.linspace(-4000, 4000, self.N+1)
        y = np.linspace(-2000, 2000, self.N+1)
        self.line_plt, = self.ax.plot(x, y, '*')
        #self.line_plt = self.ax.quiver(x, y, self.u, self.v)

        target_circle = plt.Circle((self.target_pose[0] * 1000, self.target_pose[1] * 1000), 100, color='blue')
        self.origin_circle = self.ax.add_patch(target_circle)
        self.ax.plot()
        plt.show()

        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            #self.timer.setSingleShot(True)
            self.timer.start(self.Period)
            # self.advance = []
            # self.rotation = []
            # self.compute()
            # for adv, rot in zip(self.advance, self.rotation):
            #     self.differentialrobot_proxy.setSpeedBase(adv, rot)
            #     time.sleep(1)
            # self.differentialrobot_proxy.setSpeedBase(0,0)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    @logger.catch
    def compute(self):
        print("------------------------")
        try:
            currentPose = self.omnirobot_proxy.getBaseState()
            #currentPose = self.differentialrobot_proxy.getBaseState()
            current_tr = np.array([currentPose.x/1000, currentPose.z/1000])
        except:
            print("Error connecting to base")

        start = time.time()
        dist_to_target = np.linalg.norm(current_tr - self.target_pose)
        der_to_target = dist_to_target - self.ant_dist_to_target
        print(f"Dist to target: {dist_to_target:.2f}, Dist ant:, {self.ant_dist_to_target:.2f}, Der:, {der_to_target:.2f}")
        self.ant_dist_to_target = dist_to_target
        print(f"Current pose:, {currentPose.x:.2f}, {currentPose.z:.2f}, {currentPose.alpha:.2f}")
        print(f"Target pose:, {self.target_pose[0]:.2f}, {self.target_pose[1]:.2f}")

        if self.active and dist_to_target > 0.15:

            # ---- initial values for state ---
            self.opti.set_value(self.initial_oparam[0], currentPose.x/1000)
            self.opti.set_value(self.initial_oparam[1], currentPose.z/1000)
            self.opti.set_value(self.initial_oparam[2], currentPose.alpha)
            self.opti.set_value(self.target_oparam, self.target_pose)  # se puede quitar creo

            # Warm start
            if self.sol:
                for i in range(1, self.N):
                    self.opti.set_initial(self.X[:, i], self.sol.value(self.X[:, i]))


            # ---- solve NLP ------
            self.sol = self.opti.solve()

            # ---- print output -----
            print("Iterations:", self.sol.stats()["iter_count"])
            # print("Control", int(self.sol.value(self.v_x[0] * 1000)),
            #                  int(self.sol.value(self.v_y[0] * 1000)),
            #                  int(self.sol.value(self.v_rot[0])))
            #adv = self.sol.value(self.v_a[0]*1000)
            #rot = self.sol.value(self.v_rot[0])
            #print("Control", adv, rot)
            #self.advance = self.sol.value(self.v_a*1000)
            #self.rotation = rot = self.sol.value(self.v_rot)

            # update self.vect to the desired rotation value at point P
            vect = self.sol.value(self.X[0:2, self.align_point+1]) - self.sol.value(self.X[0:2, self.align_point-1])
            #self.opti.set_value(self.vect, np.arctan2(vect[0], vect[1]))

            print(f"First pos {self.sol.value(self.pos_x[1] * 1000):.2f}, {self.sol.value(self.pos_y[2] * 1000):.2f}")
            end = time.time()
            print(f"Elapsed: {end-start:.2f}")
            # move the robot
            try:
                self.omnirobot_proxy.setSpeedBase( self.sol.value(self.v_x[0]*1000),
                                                   self.sol.value(self.v_y[0]*1000),
                                                   self.sol.value(self.v_rot[0]))
                #self.differentialrobot_proxy.setSpeedBase(adv, rot)
                pass

            except Exception as e: print(e)

            # draw
            self.line_plt.set_xdata(self.sol.value(self.pos_x*1000))
            self.line_plt.set_ydata(self.sol.value(self.pos_y*1000))
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
                self.omnirobot_proxy.setSpeedBase(0, 0, 0)
                #self.differentialrobot_proxy.setSpeedBase(0, 0)
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
        dt = 0.1   # timer interval in secs
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



