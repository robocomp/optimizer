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
pts_o = np.c_[pts_o, np.ones(npoints)]

noise = np.random.normal(0, 0.1, size=(npoints, 2))
noise = np.c_[noise, np.zeros(npoints)]
pts = pts_o + noise;
#pts = pts_o
print(pts.shape)

# # DRAW
# fig, ax = plt.subplots()
# plt.plot(pts[:, 0], pts[:, 1], 'go')
# plt.show()

# add points beyond the door
#pts = np.append(pts, np.array([[2, 0], [2, 0.1], [2, 0.2], [2, 0.3], [2, -0.1], [2, -0.2], [2, -0.3]]), axis=0)
pts = np.append(pts, np.array([[2, 0, 1], [2, 0.1, 1], [2, -0.1, 1], [2, -0.2, 1]]), axis=0)

def error(params):
    [cx, cy, sw, sh] = params
    c1 = np.array([sw, sh])
    c2 = np.array([sw, -sh])
    c3 = np.array([-sw, -sh])
    c4 = np.array([-sw, sh])
    L = np.asarray([[[c1[1] - c2[1]], [c2[0] - c1[0]], [(c1[0] - c2[0]) * c1[1] + (c2[1] - c1[1]) * c1[0]]],
                    [[c2[1] - c3[1]], [c3[0] - c2[0]], [(c2[0] - c3[0]) * c2[1] + (c3[1] - c2[1]) * c2[0]]],
                    [[c3[1] - c4[1]], [c4[0] - c3[0]], [(c3[0] - c4[0]) * c3[1] + (c4[1] - c3[1]) * c3[0]]],
                    [[c4[1] - c1[1]], [c1[0] - c4[0]], [(c4[0] - c1[0]) * c4[1] + (c1[1] - c4[1]) * c4[0]]]])

    L = L.reshape([4, 3])
    M = np.asarray([np.sqrt(m[0] * m[0] + m[1] * m[1]) for m in L])
    #print(L.shape, M.shape, pts.shape)

    # add center offset
    n_pts = pts + np.array([cx, cy, 0])
    dist = L @ n_pts.transpose()
    dist = np.fabs(dist)
    dist = dist / M[:, None]
    #print(dist)
    m_dist = np.amin(dist, axis=0)
    #print("----------------")
    #print(m_dist.sum())
    return m_dist.sum()

cx = 0.2
cy = -0.1
sw = 0.7
sh = 1.3
axis = [cx, cy, sw, sh]
deltas = [-0.01, 0.01]
e_ant = np.finfo(float).max
idx = np.random.choice(4, 1)
new_delta = np.random.choice(deltas, 1)

for i in range(10000):
    axis[idx[0]] += new_delta[0]
    e = error(axis)
    if e < 1:
        break
    if( e >= e_ant):
        idx = np.random.choice(4, 1)
        new_delta = np.random.choice(deltas, 1)
        #print("axis", idx[0])
        e_ant = np.finfo(float).max
    e_ant = e
print(e, i)

# # DRAW
fig, ax = plt.subplots()
plt.plot(pts[:, 0], pts[:, 1], 'go')
[cx, cy, sw, sh] = axis
rect = patches.Rectangle((cx-sw, cy-sh), sw*2, sh*2, linewidth=2, edgecolor='b', facecolor='none')
#t2 = mpl.transforms.Affine2D().rotate(np.degrees(coa)) + ax.transData
#rect.set_transform(t2)
ax.add_patch(rect)
# # plt.grid(True)
plt.show()



