import glob
import json
import random
import cv2 as cv
import numpy as np
import dt_apriltags as april
from functools import reduce
from pytransform3d import transformations as pt


class Calibrator(object):
    def __init__(self, camera_name, camera_port):
        self.cam_name = camera_name
        self.cam_port = camera_port
        self.cap = cv.VideoCapture(camera_port)
        self.mean = []
        self.dist = []
        self.world = []

    def __del__(self):
        print('~Destructor~')

    def capture_images(self, number_of_pics=150):
        print("{}: Capturing images".format(self.cam_name))
        for i in range(number_of_pics):
            ret, frame = self.cap.read()
            if ret:
                cv.imwrite("fotos/{}_{}.jpeg".format(self.cam_name, i), frame)
            else:
                print("{} not available, pic {} missed :(".format(self.cam_name, i))
        print("{}: Capturing images DONE".format(self.cam_name))

    def get_dist_coefficients(self, board_x=9, board_y=7, num_calibs=2):
        print("{}: Getting coefficients".format(self.cam_name))
        l_mat = []
        l_dist = []
        for i in range(num_calibs):
            print("{}:   Calibration {}/{}".format(self.cam_name, i+1, num_calibs))
            mtx, dist = self._calibrate(board_x - 1, board_y - 1)
            l_mat.append(np.array(mtx))
            l_dist.append(np.array(dist))
        self.mean = reduce(lambda a, b: a + b, l_mat) / len(l_mat)
        self.dist = reduce(lambda a, b: a + b, l_dist) / len(l_dist)
        print("{}: Getting coefficients DONE".format(self.cam_name))

    def _calibrate(self, corners_x, corners_y):
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        obj_p = np.zeros((corners_y * corners_x, 3), np.float32)
        obj_p[:, :2] = np.mgrid[0:corners_x, 0:corners_y].T.reshape(-1, 2)
        obj_p = np.array([x * 30 for x in obj_p])
        obj_points = []
        img_points = []
        images = glob.glob("fotos/{}_*.jpeg".format(self.cam_name))
        # print(len(images))
        random.shuffle(images)
        for pic in images[:30]:
            img = cv.imread(pic)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, (corners_x, corners_y), None)
            if ret:
                obj_points.append(obj_p)
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                img_points.append(corners)
                cv.drawChessboardCorners(img, (corners_x, corners_y), corners2, ret)
            gray_ = gray
        cv.destroyAllWindows()
        ret, mtx, dist, _, _ = cv.calibrateCamera(obj_points, img_points, gray_.shape[::-1], None, None)
        if ret:
            return mtx, dist

    def get_world_position(self):
        print("{}: Getting world coords".format(self.cam_name))
        ret, frame = self.cap.read()
        if ret:
            h, w = frame.shape[:2]
            new_mat, roi = cv.getOptimalNewCameraMatrix(self.mean, self.dist, (w, h), 1, (w, h))
            new_img = cv.undistort(frame, self.mean, self.dist, None, new_mat)
            self.world = self._calibrate_apriltag(new_img, new_mat[0][0], new_mat[1][1], new_mat[0][2], new_mat[1][2])
            cv.imshow('FRAME', frame)
            cv.imshow('RECTIFIED', new_img)
            cv.waitKey(10)
        else:
            print("{} not available :(".format(self.cam_name))
        print("{}: Getting world coords DONE".format(self.cam_name))

    def _calibrate_apriltag(self, rgb, focal_x, focal_y, cx, cy):
        world = []
        grey = cv.cvtColor(rgb, cv.COLOR_RGB2GRAY)
        detector = april.Detector(searchpath=['apriltags'], families='tagStandard41h12', nthreads=1, quad_decimate=1.0,
                                  quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)
        tags = detector.detect(grey, estimate_tag_pose=True, camera_params=[focal_x, focal_y, cx, cy], tag_size=0.276)
        # print(tags)
        if len(tags) > 0:
            tag = tags[0]
            for idx in range(len(tag.corners)):
                cv.line(rgb, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)),
                        (0, 255, 0))
                cv.putText(rgb, str(tag.tag_id),
                           org=(tag.corners[0, 0].astype(int)+10, tag.corners[0, 1].astype(int)+10),
                           fontFace=cv.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
            world = pt.transform_from(tag.pose_R, tag.pose_t.ravel() * 1000.0)
        return world

    def get_data(self):
        # print(self.mean[0], self.mean)
        cam = {'name': self.cam_name, 'id': int(self.cam_port), 'dist': self.dist[0].tolist(),
               'mean': self.mean.tolist(), 'world': self.world.tolist()}
        return self.cam_name, cam

    def calibrate(self, dict_data, photos=False):
        print("## {}: STARTING CALIBRATION".format(self.cam_name))
        if photos:
            self.capture_images()
        self.get_dist_coefficients()
        self.get_world_position()
        name, dic = self.get_data()
        dict_data[name] = dic
        print("## {}: CALIBRATION ENDED".format(self.cam_name))


cameras = {'2': ('cam1', True),
           '0': ('cam2', True)}
cameras_data = {}

for cam_id, cam_data in cameras.items():
    if cam_data[1]:
        cal = Calibrator(cam_data[0], int(cam_id))
        cal.calibrate(cameras_data)
with open('data.json', 'w') as outfile:
    json.dump(cameras_data, outfile)
