import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

poses_flat = np.fromfile("bin/poses.raw", dtype=np.float32)
poses = []

for i in range(5):
  start = i * 13
  R = poses_flat[start:start+9].reshape(3, 3)
  t = poses_flat[start+9:start+12]
  s = poses_flat[start+12]
  poses.append({'R': R, 't': t, 's': s})

N = len(poses)
ts = np.array([p['t'] for p in poses])
cmap = plt.get_cmap('viridis')
colors = cmap(np.linspace(0, 1, N))

#print(poses)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(ts[:,0], ts[:,1], ts[:,2], c='r', marker='o')
for i in range(N):
  t = ts[i]
  ax.scatter(t[0], t[1], t[2], color=colors[i], marker='o')

for i in range(N-1):
  ax.plot(ts[i:i+2, 0], ts[i:i+2, 1], ts[i:i+2, 2], color=colors[i])

for i, t in enumerate(ts):
  ax.text(t[0], t[1], t[2], str(i), color='black', fontsize=8)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Camera Poses')
ax.view_init(elev=90, azim=90)
plt.show()

