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
np.set_printoptions(precision=2)

# DATA
corners = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]])
corners = corners;
gamma = 0
rot = np.array([[np.cos(gamma), -np.sin(gamma)], [np.sin(gamma), np.cos(gamma)]])
corners = np.transpose(np.matmul(rot, np.transpose(corners)))

pts_o = np.array([corners[0]])
for a, b in pairwise(corners):
    pts_o = np.append(pts_o, np.array([a*landa + b*(1-landa) for landa in np.arange(0, 1, 0.01)]), axis=0)
pts_o = np.append(pts_o, np.array([corners[0]*landa + b*(1-landa) for landa in np.arange(0, 1, 0.01)]), axis=0)
npoints = pts_o.shape[0]
pts_o = np.c_[pts_o, np.ones(npoints)]

noise = np.random.normal(0, 0.07, size=(npoints, 2))
noise = np.c_[noise, np.zeros(npoints)]
pts = pts_o + noise;
#pts = pts_o
print(pts.shape)

# # DRAW
# fig, ax = plt.subplots()
# plt.plot(pts[:, 0], pts[:, 1], 'go')
# plt.show()

# add points beyond the door
pts = np.append(pts, np.array([[2, 0, 1], [2, 0.1, 1], [2, 0.2, 1], [2, 0.3, 1], [2, 0.4, 1], [2, 0.5, 1],
                               [2, -0.1, 1], [2, -0.2, 1], [2, -0.3, 1], [2, -0.4, 1], [2, -0.5, 1]]), axis=0)
pts = np.append(pts, np.array([[2.2, 0, 1], [2.2, 0.1, 1], [2.2, 0.2, 1], [2.2, 0.3, 1], [2.2, 0.4, 1], [2.2, 0.5, 1],
                               [2.2, -0.1, 1], [2.2, -0.2, 1], [2.2, -0.3, 1], [2.2, -0.4, 1], [2.2, -0.5, 1]]), axis=0)
pts = np.append(pts, np.array([[0, -2, 1], [0.1, -2, 1], [0.2, -2, 1], [0.3, -2, 1],
                               [-0.1, -2, 1], [-0.2, -2, 1], [-0.3, -2, 1], [-0.4, -2, 1]]), axis=0)
# pts = np.append(pts, np.array([[0, 0, 1], [0.1, 0, 1], [0.2, 0, 1], [0.3, 0, 1],
#                                [-0.1, 0, 1], [-0.2, 0, 1], [-0.3, 0, 1], [-0.4, 0, 1]]), axis=0)

# move to millimeters in Beta
pts[:, 0] *= 1000
pts[:, 1] *= 1000

def error(params, pts):
    [cx, cy, sw, sh] = params
    center = np.array([cx, cy])
    c1 = np.array([sw, sh]) + center
    c2 = np.array([sw, -sh]) + center
    c3 = np.array([-sw, -sh]) + center
    c4 = np.array([-sw, sh]) + center
    L = np.asarray([[[c1[1] - c2[1]], [c2[0] - c1[0]], [(c1[0] - c2[0]) * c1[1] + (c2[1] - c1[1]) * c1[0]]],
                    [[c2[1] - c3[1]], [c3[0] - c2[0]], [(c2[0] - c3[0]) * c2[1] + (c3[1] - c2[1]) * c2[0]]],
                    [[c3[1] - c4[1]], [c4[0] - c3[0]], [(c3[0] - c4[0]) * c3[1] + (c4[1] - c3[1]) * c3[0]]],
                    [[c4[1] - c1[1]], [c1[0] - c4[0]], [(c4[0] - c1[0]) * c4[1] + (c1[1] - c4[1]) * c4[0]]]])

    L = L.reshape([4, 3])
    M = np.asarray([np.sqrt(m[0] * m[0] + m[1] * m[1]) for m in L])

    # compute distance to lines.  4x3 * 3xN -> 4xN
    dist = L @ pts.transpose()
    dist = np.fabs(dist)
    dist = dist / M[:, None]
    # Huber robust estimator
    k = 1000
    dist[dist > k] -= k/2.0
    # min distance to all lines  4xN -> N
    m_dist = np.amin(dist, axis=0)
    # sum of residuals
    res = m_dist.sum()
    # median of residuals
    #res = np.median(m_dist)
    return res, m_dist

def optimize(pts, axis, deltas, max_iter):
    e_ant = np.finfo(float).max
    idx = np.random.choice(len(axis), 1)
    new_delta = np.random.choice(deltas, 1)
    for i in range(max_iter):
        axis[idx[0]] += new_delta[0]
        e, m_dist = error(axis, pts)
        if e < 1:
            break
        if (e >= e_ant):
            idx = np.random.choice(len(axis), 1)
            new_delta = np.random.choice(deltas, 1)
            e_ant = np.finfo(float).max
        e_ant = e
    return axis, e, i, m_dist

#################################################################################
# initial values
center = pts.mean(axis=0)
cx = center[0]
cy = center[1]
sides = pts.std(axis=0)
sw = sides[0]
sh = sides[1]
print("Initial values:", center, sides)
print("Points:", pts.shape[0])
axis = [cx, cy, sw, sh]
#deltas = [-0.01, 0.01]
deltas = [-sides[0]/500, sides[0]/500]
pts_orig = pts      # save to draw
desv = np.finfo(float).max
print("Optmizing...")
while( desv > 150):
    axis, e, iter, m_dist = optimize(pts, axis, deltas, 30000)
    print("Error %.2f" % e)
    print(" Params:", axis)
    print(" iter:", iter)
    desv = m_dist.std()
    print(" Mean: %.2f" % m_dist.mean(), "Var: %.5f" % m_dist.std())
    # outliers: those with error greater than 10 times the std
    outliers = np.where(m_dist > m_dist.mean()*(desv/80.0))
    print(" Num outliers:", len(outliers[0]))
    # remove outliers
    pts = np.delete(pts, outliers[0], axis=0)
    print("----------------")


# # DRAW
fig, ax = plt.subplots()
plt.plot(pts_orig[:, 0], pts_orig[:, 1], 'go')
[cx, cy, sw, sh] = axis
rect = patches.Rectangle((cx-sw, cy-sh), sw*2, sh*2, linewidth=2, edgecolor='b', facecolor='none')
#t2 = mpl.transforms.Affine2D().rotate(np.degrees(coa)) + ax.transData
#rect.set_transform(t2)
ax.add_patch(rect)
# # plt.grid(True)
plt.show()



