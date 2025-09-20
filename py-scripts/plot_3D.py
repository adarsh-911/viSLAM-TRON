import numpy as np
import matplotlib.pyplot as plt

def set_axes_equal(ax):
  x_limits = ax.get_xlim3d()
  y_limits = ax.get_ylim3d()
  z_limits = ax.get_zlim3d()

  x_range = abs(x_limits[1] - x_limits[0])
  y_range = abs(y_limits[1] - y_limits[0])
  z_range = abs(z_limits[1] - z_limits[0])

  max_range = max([x_range, y_range, z_range]) / 2.0

  mid_x = np.mean(x_limits)
  mid_y = np.mean(y_limits)
  mid_z = np.mean(z_limits)

  ax.set_xlim3d([mid_x - max_range, mid_x + max_range])
  ax.set_ylim3d([mid_y - max_range, mid_y + max_range])
  ax.set_zlim3d([mid_z - max_range, mid_z + max_range])

w = 1242
h = 375

points = np.fromfile("bin/WorldPoints.bin", dtype=np.float32).reshape(-1, 3)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(points[:,0], points[:,1], points[:,2], c=points[:,2], cmap='plasma')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.view_init(elev=-90, azim=-90) # +Z view
ax.invert_yaxis()
set_axes_equal(ax)

plt.show()