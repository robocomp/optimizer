from casadi import *
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

# DATA
corners = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]])
corners = corners;
gamma = 0
rot = np.array([[np.cos(gamma), -np.sin(gamma)], [np.sin(gamma), np.cos(gamma)]])
corners = np.transpose(np.matmul(rot, np.transpose(corners)))

pts_o = np.array([corners[0]])
for a, b in pairwise(corners):
    pts_o = np.append(pts_o, np.array([a*landa + b*(1-landa) for landa in np.arange(0, 1, 0.05)]), axis=0)
pts_o = np.append(pts_o, np.array([corners[0]*landa + b*(1-landa) for landa in np.arange(0, 1, 0.1)]), axis=0)
#print(pts.shape, pts)
npoints = pts_o.shape[0]

noise = 0.03 * np.random.randn(npoints, 2)
pts = pts_o + noise;

# add points beyond the door
#pts = np.append(pts, np.array([[2, 0], [2, 0.1], [2, 0.2], [2, 0.3], [2, -0.1], [2, -0.2], [2, -0.3]]), axis=0)
pts = np.append(pts, np.array([[2, 0], [2, 0.1], [2, -0.1], [2, -0.2]]), axis=0)

#https://www.ri.cmu.edu/wp-content/uploads/2017/07/Xiao-2017-Efficient-L-Shape-Fitting.pdf
##############################################
class LShapeFitting:

    class Criteria(Enum):
        AREA = 1
        CLOSENESS = 2
        VARIANCE = 3

    def __init__(self):
        # Parameters
        self.criteria = self.Criteria.VARIANCE
        self.min_dist_of_closeness_criteria = 0.01  # [m]
        self.d_theta_deg_for_search = 1.0  # [deg]
        self.R0 = 3.0  # [m] range segmentation param
        self.Rd = 0.001  # [m] range segmentation param

    def fitting(self, ox, oy):

        # step1: Adaptive Range Segmentation
        id_sets = self._adoptive_range_segmentation(ox, oy)

        # step2 Rectangle search
        rects = []
        for ids in id_sets:  # for each cluster
            cx = [ox[i] for i in range(len(ox)) if i in ids]
            cy = [oy[i] for i in range(len(oy)) if i in ids]
            rects.append(self._rectangle_search(cx, cy))

        return rects, id_sets

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

        rect = RectangleData()
        rect.a[0] = cos_s
        rect.b[0] = sin_s
        rect.c[0] = min(c1_s)
        rect.a[1] = -sin_s
        rect.b[1] = cos_s
        rect.c[1] = min(c2_s)
        rect.a[2] = cos_s
        rect.b[2] = sin_s
        rect.c[2] = max(c1_s)
        rect.a[3] = -sin_s
        rect.b[3] = cos_s
        rect.c[3] = max(c2_s)

        return rect

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
class RectangleData:

    def __init__(self):
        self.a = [None] * 4
        self.b = [None] * 4
        self.c = [None] * 4

        self.rect_c_x = [None] * 5
        self.rect_c_y = [None] * 5

    def plot(self):
        self.calc_rect_contour()
        plt.plot(self.rect_c_x, self.rect_c_y, "-r")

    def calc_rect_contour(self):

        self.rect_c_x[0], self.rect_c_y[0] = self.calc_cross_point(
            self.a[0:2], self.b[0:2], self.c[0:2])
        self.rect_c_x[1], self.rect_c_y[1] = self.calc_cross_point(
            self.a[1:3], self.b[1:3], self.c[1:3])
        self.rect_c_x[2], self.rect_c_y[2] = self.calc_cross_point(
            self.a[2:4], self.b[2:4], self.c[2:4])
        self.rect_c_x[3], self.rect_c_y[3] = self.calc_cross_point(
            [self.a[3], self.a[0]], [self.b[3], self.b[0]], [self.c[3], self.c[0]])
        self.rect_c_x[4], self.rect_c_y[4] = self.rect_c_x[0], self.rect_c_y[0]

    @staticmethod
    def calc_cross_point(a, b, c):
        x = (b[0] * -c[1] - b[1] * -c[0]) / (a[0] * b[1] - a[1] * b[0])
        y = (a[1] * -c[0] - a[0] * -c[1]) / (a[0] * b[1] - a[1] * b[0])
        return x, y

l_shape_fitting = LShapeFitting()
rects, id_sets = l_shape_fitting.fitting(pts[:, 0], pts[:, 1])
print(len(rects))

# # DEFINING THE PROBLEM
# # center of quadric
# cx = SX.sym('cx')
# cy = SX.sym('cy')
# ca = SX.sym('ca')
# #sh = SX.sym('sh')
# sh = 0.1
# sa = SX.sym('sa')
# sb = SX.sym('sb')
#
# # Sum( |X|_q * (1 - F(|X|_q)) )
# # for one point (x,y)
# # || R_t(X - Q_t) || * (1 - ((x/sa)^(2/sh)- (x/sb)^(2/sh))))²
#
# rot = SX.sym('rot', 2, 2)
# rot[0, 0] = cos(ca); rot[0, 1] = -sin(ca); rot[1, 0] = sin(ca); rot[1, 1] = cos(ca)
# trans = SX.sym('tr', 2)
# trans[0] = cx; trans[1] = cy
#
# cost = SX.sym("E", 1, pts.shape[0]) ; cost = 0
# for i in range(0, pts.shape[0]):
#     cost = cost + power(norm_2(transpose(rot) @ (pts[i]-trans)) * power((1 - power((pts[i][0]/sa), (2/sh)) - power((pts[i][1]/sb), (2/sh))), 2), 2)
# cost = cost * sa * sb
# #print(cost)
#
# nlp = {'x': vertcat(cx, cy, ca, sa, sb), 'f': cost}
# S = nlpsol('S', 'ipopt', nlp)
#
# # RESULTS
# # initial values
# centerx = np.mean(pts[:, 0])
# centery = np.mean(pts[:, 1])
# width = np.max(pts[:, 0]) - np.min(pts[:, 0])
# height = np.max(pts[:, 1]) - np.min(pts[:, 1])
# print('Initial values', centerx, centery, width, height)
# r = S(x0=[centerx, centery, 0, width/2, height/2], lbg=0, ubg=0)
# params_opt = r['x']
# params = np.array(params_opt).flatten()
# print("center", params)
# cox = params[0]
# coy = params[1]
# coa = params[2]
# co_sa = params[3]
# co_sb = params[4]

# DRAW
fig, ax = plt.subplots()
plt.plot(pts[:, 0], pts[:, 1], 'go')

# xs = [cox + co_sa*np.power(np.abs(cos(a)), sh)*np.sign(cos(a)) for a in np.arange(-np.pi, np.pi, 0.05)]
# ys = [coy + co_sb*np.power(np.abs(sin(a)), sh)*np.sign(sin(a)) for a in np.arange(-np.pi, np.pi, 0.05)]
# plt.plot(xs, ys, 'bo')

for rect in rects:
    rect.plot()

# rect = patches.Rectangle((cox-co_sa, coy-co_sb), co_sa*2, co_sb*2, linewidth=2, edgecolor='b', facecolor='none')
# t2 = mpl.transforms.Affine2D().rotate(np.degrees(coa)) + ax.transData
# rect.set_transform(t2)
# ax.add_patch(rect)
# plt.grid(True)
plt.show()



# constraints = [
#         cp.square(X[0] - X[1]) + cp.square(Y[0] - Y[1]) <= 10000, cp.square(X[1] - X[2]) + cp.square(Y[1] - Y[2]) <= 10000,
#         cp.square(X[2] - X[3]) + cp.square(Y[2] - Y[3]) <= 10000, cp.square(X[3] - X[0]) + cp.square(Y[3] - Y[0]) <= 10000,
#         cp.square(X[0] - X[2]) + cp.square(Y[0] - Y[2]) <= 20000, cp.square(X[1] - X[3]) + cp.square(Y[1] - Y[3]) <= 20000,
#         poly_area(np.array(list(X)), np.array(list(Y))) >= 10000,
#         ]
# distances = (distance(m, p) for tag, p in zip(tags.values(), zip(X, Y)) for m in tag)
#error = sum(distances)


# sw = [1, 1, 1, 1, 1, 1, 1, 1]  # switch to activate/deactivate read tags
# nlp = {'x': vertcat(cx, cy, ca), 'f': sw[0] * (corner(ca, cx+1, cy+1)[0]-rtags[0][0][0])**2 +
#                                       sw[0] * (corner(ca, cx+1, cy+1)[1]-rtags[0][0][1])**2 +
#                                       sw[1] * ((cos(ca)*(cx+1)-sin(ca)*(cy+1))-rtags[0][1][0])**2 +
#                                       sw[1] * ((sin(ca)*(cx+1)+cos(ca)*(cy+1))-rtags[0][1][1])**2 +
#                                       sw[2] * ((cos(ca)*(cx+1)-sin(ca)*(cy-1))-rtags[1][0][0])**2 +
#                                       sw[2] * ((sin(ca)*(cx+1)+cos(ca)*(cy-1))-rtags[1][0][1])**2 +
#                                       sw[3] * ((cos(ca)*(cx+1)-sin(ca)*(cy-1))-rtags[1][1][0])**2 +
#                                       sw[3] * ((sin(ca)*(cx+1)+cos(ca)*(cy-1))-rtags[1][1][1])**2 +
#                                       sw[4] * ((cos(ca)*(cx-1)-sin(ca)*(cy-1))-rtags[2][0][0])**2 +
#                                       sw[4] * ((sin(ca)*(cx-1)+cos(ca)*(cy-1))-rtags[2][0][1])**2 +
#                                       sw[5] * ((cos(ca)*(cx-1)-sin(ca)*(cy-1))-rtags[2][1][0])**2 +
#                                       sw[5] * ((sin(ca)*(cx-1)+cos(ca)*(cy-1))-rtags[2][1][1])**2 +
#                                       sw[6] * ((cos(ca)*(cx-1)-sin(ca)*(cy+1))-rtags[3][0][0])**2 +
#                                       sw[6] * ((sin(ca)*(cx-1)+cos(ca)*(cy+1))-rtags[3][0][1])**2 +
#                                       sw[7] * ((cos(ca)*(cx-1)-sin(ca)*(cy+1))-rtags[3][1][0])**2 +
#                                       sw[7] * ((sin(ca)*(cx-1)+cos(ca)*(cy+1))-rtags[3][1][1])**2}

