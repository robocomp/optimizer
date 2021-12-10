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

# # DEFINING THE PROBLEM
# # center of quadric
cx = SX.sym('cx')
cy = SX.sym('cy')
ca = SX.sym('ca')
#sh = SX.sym('sh')
sh = 0.1
sa = SX.sym('sa')
sb = SX.sym('sb')

# Sum( |X|_q * (1 - F(|X|_q)) )
# for one point (x,y)
# || R_t(X - Q_t) || * (1 - ((x/sa)^(2/sh)- (x/sb)^(2/sh))))²

rot = SX.sym('rot', 2, 2)
rot[0, 0] = cos(ca); rot[0, 1] = -sin(ca); rot[1, 0] = sin(ca); rot[1, 1] = cos(ca)
trans = SX.sym('tr', 2)
trans[0] = cx; trans[1] = cy

cost = SX.sym("E", 1, pts.shape[0]) ; cost = 0
for i in range(0, pts.shape[0]):
    cost = cost + power(norm_2(transpose(rot) @ (pts[i]-trans)) * power((1 - power((pts[i][0]/sa), (2/sh)) - power((pts[i][1]/sb), (2/sh))), 2), 2)
cost = cost * sa * sb
#print(cost)

nlp = {'x': vertcat(cx, cy, ca, sa, sb), 'f': cost}
S = nlpsol('S', 'ipopt', nlp)

# RESULTS
# initial values
centerx = np.mean(pts[:, 0])
centery = np.mean(pts[:, 1])
width = np.max(pts[:, 0]) - np.min(pts[:, 0])
height = np.max(pts[:, 1]) - np.min(pts[:, 1])
print('Initial values', centerx, centery, width, height)
r = S(x0=[centerx, centery, 0, width/2, height/2], lbg=0, ubg=0)
params_opt = r['x']
params = np.array(params_opt).flatten()
print("center", params)
cox = params[0]
coy = params[1]
coa = params[2]
co_sa = params[3]
co_sb = params[4]

# DRAW
fig, ax = plt.subplots()
plt.plot(pts[:, 0], pts[:, 1], 'go')


#xs = [cox + co_sa*np.power(np.abs(cos(a)), sh)*np.sign(cos(a)) for a in np.arange(-np.pi, np.pi, 0.05)]
#ys = [coy + co_sb*np.power(np.abs(sin(a)), sh)*np.sign(sin(a)) for a in np.arange(-np.pi, np.pi, 0.05)]
#plt.plot(xs, ys, 'bo')

rect = patches.Rectangle((cox-co_sa, coy-co_sb), co_sa*2, co_sb*2, linewidth=2, edgecolor='b', facecolor='none')
t2 = mpl.transforms.Affine2D().rotate(np.degrees(coa)) + ax.transData
rect.set_transform(t2)
ax.add_patch(rect)
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

