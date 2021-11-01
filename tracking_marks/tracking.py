from casadi import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl

class Corner(Callback):
  def __init__(self, name, ca, cx, cy, dx, dy, opts={}):
    Callback.__init__(self)
    self.R = MX.sym('r', 2, 2)
    # counterclock rotation
    self.R[0, 0] = cos(ca)
    self.R[0, 1] = -sin(ca)
    self.R[1, 0] = sin(ca)
    self.R[1, 1] = cos(ca)
    self.T = MX.sym('t', 2)
    self.T[0] = cx + dx
    self.T[1] = cy + dy
    self.construct(name, opts)

  # Number of inputs and outputs
  def get_n_in(self): return 2
  def get_n_out(self): return 1

  # Initialize the object
  def init(self):
     print('initializing object')

  # Evaluate numerically
  def eval(self, arg):
    res = self.R @ self.T
    # rx = (cos(ca)*(cx+dx) - sin(ca)*(cy+dy))
    # ry = (sin(ca)*(cx+dx) + cos(ca)*(cy+dy))
    # ret = vertcat(rx, ry)
    print(res)
    return [res]

# DATA
tags = np.array([[[1, 1],[1,1]], [[1, -1],[1,-1]], [[-1, -1],[-1,-1]], [[-1, 1],[-1,1]]])
noise = np.random.randn(4, 2, 2)/7
rtags = tags + noise;
print(tags, noise, rtags)


# DEFINING THE PROBLEM
# center of artifact
cx = SX.sym('cx')
cy = SX.sym('cy')
ca = SX.sym('ca')
#corner11 = Corner('c', ca, cx, cy, 1, 1)
#print("corner", corner11)

corner = Function('f11', [ca, cx, cy], [cos(ca)*(cx) - sin(ca)*(cy), sin(ca)*(cx) + cos(ca)*(cy)])

# corner coordinates from center
indices = [[1, 1], [1, -1], [-1, -1], [-1, 1]]

cost = SX.sym("E", 1, 16)
cost = (corner(ca, cx+indices[0][0], cx+indices[0][1])[0] - rtags[0][0][0])**2
cost = cost + (corner(ca, cx + indices[0][0], cx + indices[0][1])[1] - rtags[0][0][1]) ** 2
for i in range(1, 4):  # 4 corners, 2 coordinates each
  for j in range(2):   # 2 measures
    cost = cost + (corner(ca, cx+indices[i][0], cx+indices[i][1])[0] - rtags[i][j][0])**2
    cost = cost + (corner(ca, cx+indices[i][0], cx+indices[i][1])[1] - rtags[i][j][1])**2

nlp = {'x': vertcat(cx, cy, ca), 'f': cost}
S = nlpsol('S', 'ipopt', nlp)

# RESULTS
r = S(x0=[0, 0, 0], lbg=0, ubg=0)
centre_opt = r['x']
center = np.array(centre_opt).flatten()
print("center", center[0], center[1], center[2])

# DRAW
fig, ax = plt.subplots()
plt.plot(tags[:, :, 0], tags[:, :, 1], 'go')
plt.plot(rtags[:, :, 0], rtags[:, :, 1], 'bo')
rect = patches.Rectangle((center[0]-1, center[1]-1), 2, 2, linewidth=1, edgecolor='r', facecolor='none')
t2 = mpl.transforms.Affine2D().rotate(center[2]) + ax.transData
rect.set_transform(t2)
ax.add_patch(rect)
plt.grid(True)
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

