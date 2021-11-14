#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

#from casadi import *
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
from more_itertools import pairwise
import itertools
from enum import Enum
from scipy.spatial.transform import Rotation as Rot
from scipy import linspace, polyval, polyfit, sqrt, stats, randn
import copy


class SpecificWorker(GenericWorker):
    class Criteria(Enum):
        AREA = 1
        CLOSENESS = 2
        VARIANCE = 3

    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        self.criteria = self.Criteria.VARIANCE
        self.min_dist_of_closeness_criteria = 0.01  # [m]
        self.d_theta_deg_for_search = 1.0  # [deg]
        self.R0 = 0.5  # [m] range segmentation param
        self.Rd = 0.001  # [m] range segmentation param

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

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
    def compute(self):
        print('SpecificWorker.compute...')
        # computeCODE
        # try:
        #   self.differentialrobot_proxy.setSpeedBase(100, 0)
        # except Ice.Exception as e:
        #   traceback.print_exc()
        #   print(e)

        # The API of python-innermodel is not exactly the same as the C++ version
        # self.innermodel.updateTransformValues('head_rot_tilt_pose', 0, 0, 0, 1.3, 0, 0)
        # z = librobocomp_qmat.QVec(3,0)
        # r = self.innermodel.transform('rgbd', z, 'laser')
        # r.printvector('d')
        # print(r[0], r[1], r[2])

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    #detection related methods
    def fitting(self, ox, oy):

        # step1: Adaptive Range Segmentation
        id_sets = self._adoptive_range_segmentation(ox, oy)

        # step2 Rectangle search
        rects = RoboCompRoomDetection.Rooms()
        for ids in id_sets:  # for each cluster
            cx = [ox[i] for i in range(len(ox)) if i in ids]
            cy = [oy[i] for i in range(len(oy)) if i in ids]
            rects.append(self._rectangle_search(cx, cy))

        return rects

    @staticmethod
    def _calc_area_criterion(c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)

        alpha = -(c1_max - c1_min) * (c2_max - c2_min)

        return alpha

    def _calc_closeness_criterion(self, c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)

        # Vectorization
        D1 = np.minimum(c1_max - c1, c1 - c1_min)
        D2 = np.minimum(c2_max - c2, c2 - c2_min)
        d = np.maximum(np.minimum(D1, D2), self.min_dist_of_closeness_criteria)
        beta = (1.0 / d).sum()

        return beta

    @staticmethod
    def _calc_variance_criterion(c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)

        # Vectorization
        D1 = np.minimum(c1_max - c1, c1 - c1_min)
        D2 = np.minimum(c2_max - c2, c2 - c2_min)
        E1 = D1[D1 < D2]
        E2 = D2[D1 >= D2]
        V1 = - np.var(E1) if len(E1) > 0 else 0.
        V2 = - np.var(E2) if len(E2) > 0 else 0.
        gamma = V1 + V2

        return gamma

    def _rectangle_search(self, x, y):

        X = np.array([x, y]).T

        d_theta = np.deg2rad(self.d_theta_deg_for_search)
        min_cost = (-float('inf'), None)
        for theta in np.arange(0.0, np.pi / 2.0 - d_theta, d_theta):

            rot = Rot.from_euler('z', theta).as_matrix()[0:2, 0:2]
            c = X @ rot
            c1 = c[:, 0]
            c2 = c[:, 1]

            # Select criteria
            cost = 0.0
            if self.criteria == self.Criteria.AREA:
                cost = self._calc_area_criterion(c1, c2)
            elif self.criteria == self.Criteria.CLOSENESS:
                cost = self._calc_closeness_criterion(c1, c2)
            elif self.criteria == self.Criteria.VARIANCE:
                cost = self._calc_variance_criterion(c1, c2)

            if min_cost[0] < cost:
                min_cost = (cost, theta)

        # calc best rectangle
        sin_s = np.sin(min_cost[1])
        cos_s = np.cos(min_cost[1])

        c1_s = X @ np.array([cos_s, sin_s]).T
        c2_s = X @ np.array([-sin_s, cos_s]).T

        a = [None] * 4
        b = [None] * 4
        c = [None] * 4

        a[0] = cos_s
        b[0] = sin_s
        c[0] = min(c1_s)
        a[1] = -sin_s
        b[1] = cos_s
        c[1] = min(c2_s)
        a[2] = cos_s
        b[2] = sin_s
        c[2] = max(c1_s)
        a[3] = -sin_s
        b[3] = cos_s
        c[3] = max(c2_s)

        rect_c = RoboCompRoomDetection.Corner()
        rectangle = RoboCompRoomDetection.ListOfPoints()
        rect_c.x, rect_c.y = self.calc_cross_point( a[0:2], b[0:2], c[0:2])
        rectangle.append(copy.deepcopy(rect_c))
        rect_c.x, rect_c.y = self.calc_cross_point(a[1:3], b[1:3], c[1:3])
        rectangle.append(copy.deepcopy(rect_c))
        rect_c.x, rect_c.y = self.calc_cross_point(a[2:4], b[2:4], c[2:4])
        rectangle.append(copy.deepcopy(rect_c))
        rect_c.x, rect_c.y = self.calc_cross_point([a[3], a[0]], [b[3], b[0]], [c[3], c[0]])
        rectangle.append(copy.deepcopy(rect_c))
        rect_c.x, rect_c.y = rectangle[0].x, rectangle[0].y
        rectangle.append(copy.deepcopy(rect_c))
        return rectangle

    @staticmethod
    def calc_cross_point(a, b, c):
        x = (b[0] * -c[1] - b[1] * -c[0]) / (a[0] * b[1] - a[1] * b[0])
        y = (a[1] * -c[0] - a[0] * -c[1]) / (a[0] * b[1] - a[1] * b[0])
        return x, y

    def _adoptive_range_segmentation(self, ox, oy):

        # Setup initial cluster
        S = []
        for i, _ in enumerate(ox):
            C = set()
            R = self.R0 + self.Rd * np.linalg.norm([ox[i], oy[i]])
            for j, _ in enumerate(ox):
                d = np.hypot(ox[i] - ox[j], oy[i] - oy[j])
                if d <= R:
                    C.add(j)
            S.append(C)

        # Merge cluster
        while 1:
            no_change = True
            for (c1, c2) in list(itertools.permutations(range(len(S)), 2)):
                if S[c1] & S[c2]:
                    S[c1] = (S[c1] | S.pop(c2))
                    no_change = False
                    break
            if no_change:
                break

        return S


    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of detectRoom method from RoomDetection interface
    #
    def RoomDetection_detectRoom(self, l):

        lX = [p.x for p in l]
        lY = [p.y for p in l]
        ret = self.fitting(lX, lY)

        #
        # write your CODE here
        #
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompRoomDetection you can use this types:
    # RoboCompRoomDetection.Corner


