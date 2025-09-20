import numpy as np
import matplotlib.pyplot as plt

w = 1242
h = 375

MAX_POINTS = 1200

img_data = np.fromfile("bin/frame1.raw", dtype=np.uint8).reshape(h, w)

points = np.fromfile("bin/kp1.raw", dtype=np.int32).reshape(-1, 2)
points = points[:MAX_POINTS]

#plt.imshow(img_data, cmap='gray')
xs, ys = zip(*points)
plt.scatter(xs, ys, c='red', marker='o', s=2)
plt.gca().set_xlim([0, w])
plt.gca().set_ylim([0, h])
plt.gca().invert_yaxis()
plt.gca().set_aspect('equal', 'box')
plt.title("Frame 1")
plt.grid(True)
plt.show()