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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from rdp import rdp
from itertools import islice
from dataclasses import dataclass

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

@dataclass
class Gaussian:
    mu: []
    S: []

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            gaussians = []
            self.timer.timeout.connect(self.compute)
            self.timer.setSingleShot(True)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        return True


    @QtCore.Slot()
    def compute(self):

        try:
            ldata = self.laser_proxy.getLaserData()
            bState = self.differentialrobot_proxy.getBaseState()
        except:
            print("laser data read failed")
            return

        points = [[l.dist * np.sin(l.angle), l.dist*np.cos(l.angle)] for l in ldata]
        rdp_points = rdp(points, epsilon=70)
        rdp_points.append([0,0])
        rdp_points.insert(0, [0, 0])

        fig = plt.figure()
        ax = fig.add_subplot(211, aspect='auto')
        ax2 = fig.add_subplot(212, aspect='auto')
        x = np.linspace(-5, 5, 100)
        y = np.linspace(-5, 5, 100)
        xv, yv = np.meshgrid(x, y)
        C = 0.3
        lC = np.log(C)
        self.gaussians = []

        for i in range(len(rdp_points) - 1):
            pr1, pr2 = rdp_points[i: i + 2]
            p1 = np.array(pr1)/1000.0
            p2 = np.array(pr2)/1000.0
            print("Points:  ", p1, p2)
            mu = (p2 + p1) / 2
            e1 = p1 - mu
            sx = -(e1 @ e1) / (2 * lC)
            print('Computed sx', sx)
            if sx < 0:
                sy = sx
            else:
                sy = sx/50
            sxy = 0

            K = np.array([[sx, sxy], [sxy, sy]])
            print('K', K)
            # rotate
            dir = p2 - p1
            a = np.arctan2(dir[1], dir[0])
            print('atan angle', a)
            R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])  # CCW

            KR = R @ K @ R.transpose()
            self.gaussians.append(Gaussian(mu, KR))
            print('KR', KR)

            g = self.gauss(e1, KR, C)
            w, v = np.linalg.eigh(KR)
            print('vals', w)
            print('vec', v)
            max_i = np.argmax(w)
            min_i = np.argmin(w)
            angle = np.arctan2(v[max_i, 1], v[max_i, 0])
            print('angle', np.rad2deg(angle))
            print('gauss', mu, '[', KR[0, 0], KR[1, 1], KR[0, 1], ']', g)

            plt.plot(p1[0], p1[1], 'ro')
            plt.plot(p2[0], p2[1], 'mo')

            ellipse = Ellipse((mu[0], mu[1]), width=2 + w[max_i], height=2 * w[min_i], angle=np.rad2deg(angle),
                              edgecolor='blue', fill=False, linewidth=2)
            ax.add_patch(ellipse)
            ax.set_xlim(-5, 5)
            ax.set_ylim(-5, 5)

        # add symmetric gaussians in points
        gauss_points = []
        for p in rdp_points:
            gauss_points.append(Gaussian(np.array(p)/1000, [[0.07, 0], [0, 0.07]]))

        Z = np.zeros((100, 100))
        for i in range(100):
            for j in range(100):
                vals = []
                for g in self.gaussians:
                    vals.append(self.gauss(np.array([xv[i, j] - g.mu[0], yv[i, j] - g.mu[1]]), g.S, C))
                for g in gauss_points:
                    vals.append(self.gauss(np.array([xv[i, j] - g.mu[0], yv[i, j] - g.mu[1]]), g.S, C))
                Z[i, j] = np.max(vals)
        ax2.contourf(xv, yv, Z)
        plt.show()

        return True

    def gauss(self, e, S, C):
        v = np.exp(-0.5 * (e.transpose() @ np.linalg.inv(S) @ e))
        if v >= 0.1:

            return v
        else:
            return 0
    #############################################################################################
    def startup_check(self):
        print(f"Testing RoboCompBillCoppelia.Pose from ifaces.RoboCompBillCoppelia")
        test = ifaces.RoboCompBillCoppelia.Pose()
        print(f"Testing RoboCompDifferentialRobot.TMechParams from ifaces.RoboCompDifferentialRobot")
        test = ifaces.RoboCompDifferentialRobot.TMechParams()
        print(f"Testing RoboCompLaser.LaserConfData from ifaces.RoboCompLaser")
        test = ifaces.RoboCompLaser.LaserConfData()
        print(f"Testing RoboCompLaser.TData from ifaces.RoboCompLaser")
        test = ifaces.RoboCompLaser.TData()
        QTimer.singleShot(200, QApplication.instance().quit)



    ######################
    # From the RoboCompBillCoppelia you can call this methods:
    # self.billcoppelia_proxy.getPose(...)
    # self.billcoppelia_proxy.setSpeed(...)
    # self.billcoppelia_proxy.setTarget(...)

    ######################
    # From the RoboCompBillCoppelia you can use this types:
    # RoboCompBillCoppelia.Pose

    ######################
    # From the RoboCompDifferentialRobot you can call this methods:
    # self.differentialrobot_proxy.correctOdometer(...)
    # self.differentialrobot_proxy.getBasePose(...)
    # self.differentialrobot_proxy.getBaseState(...)
    # self.differentialrobot_proxy.resetOdometer(...)
    # self.differentialrobot_proxy.setOdometer(...)
    # self.differentialrobot_proxy.setOdometerPose(...)
    # self.differentialrobot_proxy.setSpeedBase(...)
    # self.differentialrobot_proxy.stopBase(...)

    ######################
    # From the RoboCompDifferentialRobot you can use this types:
    # RoboCompDifferentialRobot.TMechParams

    ######################
    # From the RoboCompLaser you can call this methods:
    # self.laser_proxy.getLaserAndBStateData(...)
    # self.laser_proxy.getLaserConfData(...)
    # self.laser_proxy.getLaserData(...)

    ######################
    # From the RoboCompLaser you can use this types:
    # RoboCompLaser.LaserConfData
    # RoboCompLaser.TData


