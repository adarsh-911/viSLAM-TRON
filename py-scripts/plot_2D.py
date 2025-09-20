import numpy as np
import matplotlib.pyplot as plt

w = 1242
h = 375

MAX_POINTS = 800

points = np.fromfile("bin/kp1.raw", dtype=np.int32).reshape(-1, 2)

xs, ys = zip(*points)
plt.scatter(xs, ys, c='red', marker='o', s=2)
plt.title("Frame 1")
plt.grid(True)
plt.show()