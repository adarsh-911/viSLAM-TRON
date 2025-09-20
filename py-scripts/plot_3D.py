import numpy as np
import matplotlib.pyplot as plt

w = 1242
h = 375

points = np.fromfile("bin/WorldPoints.bin", dtype=np.float32).reshape(-1, 3)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(points[:,0], points[:,1], points[:,2], c=points[:,2]*(-1), cmap='plasma')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.view_init(elev=90, azim=-90) # +Z view
plt.show()