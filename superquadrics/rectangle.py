from casadi import *
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
from more_itertools import pairwise
import itertools
from enum import Enum
from scipy.spatial.transform import Rotation as Rot
from scipy import linspace, polyval, polyfit, sqrt, stats, randn
import numpy as np

# DATA
corners = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]])
corners = corners;
gamma = 0
rot = np.array([[np.cos(gamma), -np.sin(gamma)], [np.sin(gamma), np.cos(gamma)]])
corners = np.transpose(np.matmul(rot, np.transpose(corners)))

pts_o = np.array([corners[0]])
for a, b in pairwise(corners):
    pts_o = np.append(pts_o, np.array([a*landa + b*(1-landa) for landa in np.arange(0, 1, 0.1)]), axis=0)
pts_o = np.append(pts_o, np.array([corners[0]*landa + b*(1-landa) for landa in np.arange(0, 1, 0.1)]), axis=0)
npoints = pts_o.shape[0]

noise = 0.001 * np.random.randn(npoints, 2)
pts = pts_o + noise;
#pts = pts_o
print(pts.shape)

# # DRAW
# fig, ax = plt.subplots()
# plt.plot(pts[:, 0], pts[:, 1], 'go')
# plt.show()

# add points beyond the door
#pts = np.append(pts, np.array([[2, 0], [2, 0.1], [2, 0.2], [2, 0.3], [2, -0.1], [2, -0.2], [2, -0.3]]), axis=0)
#pts = np.append(pts, np.array([[2, 0], [2, 0.1], [2, -0.1], [2, -0.2]]), axis=0)

# # DEFINING THE PROBLEM
# center of rectangle
cx = SX.sym('cx')
cy = SX.sym('cy')
# rotation angle
ca = SX.sym('ca')
# sides
sw = SX.sym('sw')
sh = SX.sym('sh')
# rotation matrix
rot = SX.sym('rot', 2, 2)
rot[0, 0] = cos(ca); rot[0, 1] = -sin(ca); rot[1, 0] = sin(ca); rot[1, 1] = cos(ca)
# translation vector
tr = SX.sym('tr', 2)
tr[0] = cx; tr[1] = cy
# corner coordinates
# c1
c1 = SX.sym('c1', 2)
c1[0] = sw; c1[1] = sh
c1_w = SX.sym('c1_w', 2)
#c1_w = rot @ c1 + tr
c1_w = c1 + tr
# c2
c2 = SX.sym('c2', 2)
c2[0] = sw; c2[1] = -sh
c2_w = SX.sym('c2_w', 2)
#c2_w = rot @ c2 + tr
c2_w = c2 + tr
# c3
c3 = SX.sym('c3', 2)
c3[0] = -sw; c3[1] = -sh
c3_w = SX.sym('c3_w', 2)
#c3_w = rot @ c3 + tr
c3_w = c3 + tr
# c4
c4 = SX.sym('c4', 2)
c4[0] = -sw; c4[1] = sh
c4_w = SX.sym('c4_w', 2)
#c4_w = rot @ c4 + tr
c4_w = c4 + tr
# line segments: A=y1-y2 B=x2-x1 C=(x1-x2)y1 + (y2-y1)x1
# ls1 = c1(x1,y1)-c2(x2,y2)
ls1 = SX.sym('ls1', 3)
ls1[0] = c1_w[1]-c2_w[1]
ls1[1] = c2_w[0]-c1_w[0]
ls1[2] = (c1_w[0]-c2_w[0])*c1_w[1] - (c2_w[1]-c1_w[1])*c1_w[0]
# ls2 = c2(x1,y1)-c3(x2,y2)
ls2 = SX.sym('ls2', 3)
ls2[0] = c2_w[1]-c3_w[1]
ls2[1] = c3_w[0]-c2_w[0]
ls2[2] = (c2_w[0]-c3_w[0])*c2_w[1] - (c3_w[1]-c2_w[1])*c2_w[0]
# ls3 = c3(x1,y1)-c4(x2,y2)
ls3 = SX.sym('ls3', 3)
ls3[0] = c3_w[1]-c4_w[1]
ls3[1] = c4_w[0]-c3_w[0]
ls3[2] = (c3_w[0]-c4_w[0])*c3_w[1] - (c4_w[1]-c3_w[1])*c3_w[0]
# ls4 = c4(x1,y1)-c1(x2,y2)
ls4 = SX.sym('ls4', 3)
ls4[0] = c4_w[1]-c1_w[1]
ls4[1] = c1_w[0]-c4_w[0]
ls4[2] = (c4_w[0]-c1_w[0])*c4_w[1] - (c1_w[1]-c4_w[1])*c4_w[0]
# distance to line: d = | Ax + By + C | / sqrt(A² + B²)
p = SX.sym('p', 3)
# to line 1
f_dist_to_ls1 = Function('f_dist_to_ls1', [p], [casadi.fabs(ls1.T @ p) / casadi.sqrt(ls1[0]**2 + ls1[1]**2)])
# to line 2
f_dist_to_ls2 = Function('f_dist_to_ls2', [p], [casadi.fabs(ls2.T @ p) / casadi.sqrt(ls2[0]**2 + ls2[1]**2)])
# to line 3
f_dist_to_ls3 = Function('f_dist_to_ls3', [p], [casadi.fabs(ls3.T @ p) / casadi.sqrt(ls3[0]**2 + ls3[1]**2)])
# to line 4
f_dist_to_ls4 = Function('f_dist_to_ls4', [p], [casadi.fabs(ls4.T @ p) / casadi.sqrt(ls4[0]**2 + ls4[1]**2)])
# minimun distance to all lines
min_vector = SX.sym('m_vector', 4)

# point = SX.sym('point', 3)
# point[0] = 3; point[1] = 2; point[2] = 1
print(f_dist_to_ls4)

p[2] = 1
cost = SX.sym("E", 1, pts.shape[0]) ; cost = 0
for i in range(0, pts.shape[0]):
    p[0] = pts[i][0]
    p[1] = pts[i][1]
    p[2] = 1
    min_vector[0] = f_dist_to_ls1(p)
    min_vector[1] = f_dist_to_ls2(p)
    min_vector[2] = f_dist_to_ls3(p)
    min_vector[3] = f_dist_to_ls4(p)
    cost = cost + mmin(min_vector)
#
#print(cost)
#
nlp = {'x': vertcat(cx, cy, ca, sw, sh), 'f': cost}
options = {"ipopt": {"max_iter": 5000}}
S = nlpsol('S', 'ipopt', nlp)

#
# # RESULTS
# # initial values
# centerx = np.mean(pts[:, 0])
# centery = np.mean(pts[:, 1])
# width = np.max(pts[:, 0]) - np.min(pts[:, 0])
# height = np.max(pts[:, 1]) - np.min(pts[:, 1])
# print('Initial values', centerx, centery, width, height)
#r = S(x0=[centerx, centery, 0, width/2, height/2], lbg=0, ubg=0)

r = S(x0=[0, 0, 0, 1, 1], lbg=0, ubg=0)
params_opt = r['x']
params = np.array(params_opt).flatten()
print("center", params)
cox = params[0]
coy = params[1]
coa = params[2]
co_sw = params[3]
co_sh = params[4]
#

# # DRAW
fig, ax = plt.subplots()
plt.plot(pts[:, 0], pts[:, 1], 'go')

rect = patches.Rectangle((cox-co_sw, coy-co_sh), co_sw*2, co_sh*2, linewidth=2, edgecolor='b', facecolor='none')
t2 = mpl.transforms.Affine2D().rotate(np.degrees(coa)) + ax.transData
rect.set_transform(t2)
ax.add_patch(rect)
# plt.grid(True)
plt.show()



