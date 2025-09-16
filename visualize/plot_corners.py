import numpy as np
import matplotlib.pyplot as plt

w = 1242
h = 375

img_data1 = np.fromfile("bin/frame1.raw", dtype=np.uint8).reshape(h, w)
img_data2 = np.fromfile("bin/frame2.raw", dtype=np.uint8).reshape(h, w)

points1 = np.fromfile("bin/kp1.raw", dtype=np.int32).reshape(-1, 2)
points2 = np.fromfile("bin/kp2.raw", dtype=np.int32).reshape(-1, 2)

#print(points)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 12), sharex=True, sharey=True)

ax1.imshow(img_data1, cmap='gray', interpolation='nearest')
xs1, ys1 = zip(*points1)
ax1.scatter(xs1, ys1, c='lime', marker='o', s=1)
ax1.set_title("Frame 1")

ax2.imshow(img_data2, cmap='gray', interpolation='nearest')
xs2, ys2 = zip(*points2)
ax2.scatter(xs2, ys2, c='blue', marker='o', s=1)
ax2.set_title("Frame 2")

plt.tight_layout()
plt.show()

plt.show()