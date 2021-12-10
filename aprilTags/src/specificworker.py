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

import json
import os.path
import cv2 as cv
import math as m
import numpy as np
from casadi import *
from genericworker import *
import dt_apriltags as april
from rich.console import Console
from PySide2.QtCore import QTimer
from collections import defaultdict
from PySide2.QtWidgets import QApplication
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager
import ast

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        # Reading json file
        self.camDict = {}
        self.load_from_json(os.path.join(os.path.dirname(__file__), '../JsonGeneration/data.json'))

        # Creating one capturer for each camera
        self.capturers = {}
        for cam_name, camera in self.camDict.items():
            self.capturers[camera['name']] = cv.VideoCapture(camera['id'])
        print(len(self.capturers))

        # AprilTags' detector
        self.detector = april.Detector(searchpath=['apriltags'], families='tagStandard41h12', nthreads=1,
                                       quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
                                       debug=0)

        # Robot's pose data
        self.x, self.y, self.a, self.num_tags = 0, 0, 0, 0
        self.indices = {}

        # Compute-triggering code
        self.iter = 0
        self.tm = TransformManager()
        self.timer.start(self.Period)
        self.Period = 10

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        try:
            self.indices = {k: v for k, v in ast.literal_eval(params["coords"])}
        except KeyError:
            print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        april_tags = defaultdict(list)
        self.extract_tags(april_tags)
        for k, v in april_tags.items():
            print(f"{k}: {v}")
        if april_tags != {}:
            self.num_tags = len(april_tags.keys())
            self.optimize_position(april_tags)
        print(f"(x, y, rz) = ({self.x}, {self.y}, {self.a})")
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def optimize_position(self, april_tags):
        cx, cy, ca, x, y = SX.sym('cx'), SX.sym('cy'), SX.sym('ca'), SX.sym('x'), SX.sym('y')
        tags = april_tags
        cost = SX.sym('E', 1, 16)
        corner = Function('corner', [cx, cy, ca, x, y], [x*cos(ca) - y*sin(ca) + cx, x*sin(ca) + y*cos(ca) + cy])
        cost = 0
        for k in tags.keys():  # for every corner...
            point = self.indices[k]
            x, y = point[0], point[1]
            rt_point = corner(cx, cy, ca, x, y)
            for j in range(len(tags[k])):  # for every measure...
                cost += (rt_point[0] - tags[k][j][0]) ** 2 + (rt_point[1] - tags[k][j][1]) ** 2
        nlp = {'x': vertcat(cx, cy, ca), 'f': cost}
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
        }
        solution = nlpsol('S', 'ipopt', nlp, opts)
        result = solution(x0=[0, 0, 0], lbg=0, ubg=0)
        cx, cy, ca = np.array(result['x']).flatten()
        self.x, self.y, self.a = cx, cy, ca

    def extract_tags(self, april_tags):
        tags, ids = [], []
        for camera in self.capturers.keys():
            cap = self.capturers[camera]
            ret, frame = cap.read()
            if ret:
                img, cam_tags, cam_ids = self.process_image(self.camDict[camera], frame)
                tags.append(cam_tags)
                ids.append(cam_ids)
                rotation = cv.ROTATE_90_COUNTERCLOCKWISE if camera == 'cam2' else cv.ROTATE_90_CLOCKWISE
                cv.imshow(camera, cv.resize(cv.rotate(img, rotation), (540, 810)))
                cv.waitKey(10)
            else:
                print("{} is not available :(".format(camera))

        for tags_, ids_ in zip(tags, ids):
            for tag, t_id in zip(tags_, ids_):
                april_tags[t_id].append([tag[1, 3], tag[0, 3], self.rot_mat_2_euler(tag[:-1, :-1])[2] * 180 / np.pi])

    def process_image(self, camera, frame):
        h, w = frame.shape[:2]
        mat, roi = cv.getOptimalNewCameraMatrix(camera['mean'], camera['dist'], (w, h), 1, (w, h))
        dst = cv.undistort(frame, camera['mean'], camera['dist'], None, mat)
        transforms, tag_ids = self.calibrate_with_apriltag(dst, mat[0][0], mat[1][1], mat[0][2], mat[1][2])
        tags = self.calculate_coords(transforms, camera['world'])
        return dst, tags, tag_ids

    def calibrate_with_apriltag(self, rgb, focalx, focaly, cx, cy):
        grey = cv.cvtColor(rgb, cv.COLOR_RGB2GRAY)
        transform = []
        tag_ids = []
        tags = self.detector.detect(grey, estimate_tag_pose=True,
                                    camera_params=[focalx, focaly, cx, cy], tag_size=0.0825)
        if len(tags) > 0:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv.line(rgb, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)),
                            (0, 255, 0))
                    cv.putText(rgb, str(tag.tag_id),
                               org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                               fontFace=cv.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
                t = tag.pose_t.ravel() * 1000.0
                transform.append(pt.transform_from(tag.pose_R, t))
                tag_ids.append(tag.tag_id)
        else:
            print("No tags detected")
        return np.array(transform), np.array(tag_ids)

    # es necesario?? --> MAP
    def calculate_coords(self, transform, world_mat):
        world_i = np.linalg.inv(world_mat)
        return np.array([world_i @ mat for mat in transform])

    def is_rot_mat(self, mat):
        should_be_identity = np.transpose(mat) @ mat
        identity = np.identity(3, dtype=mat.dtype)
        return np.linalg.norm(identity - should_be_identity) < 1e-6

    def rot_mat_2_euler(self, mat):
        assert(self.is_rot_mat(mat))
        sy = m.sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0])
        singular = sy < 1e-6
        if not singular:
            ret = [m.atan2(mat[2, 1], mat[2, 2]), m.atan2(-mat[2, 0], sy), m.atan2(mat[1, 0], mat[0, 0])]
        else:
            ret = [m.atan2(-mat[1, 2], mat[1, 1]), m.atan2(-mat[2, 0], sy), m.atan2(mat[1, 0]), 0]
        return np.array(ret)

    def load_from_json(self, file_path):
        with open(file_path) as file:
            self.camDict = json.load(file)
            for cam_name, camera in self.camDict.items():
                camera['dist'] = np.array(camera['dist'])
                camera['mean'] = np.array(camera['mean'])
                camera['world'] = np.array(camera['world'])

    # # IMPLEMENTATION of getFullPoseEuler method from FullPoseEstimation interface
    def FullPoseEstimation_getFullPoseEuler(self):
        ret = RoboCompFullPoseEstimation.FullPoseEuler()
        ret.x = self.x
        ret.y = self.y
        ret.z = self.num_tags
        ret.rz = self.a
        return ret
