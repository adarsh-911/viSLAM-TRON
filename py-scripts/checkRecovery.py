import numpy as np
import matplotlib.pyplot as plt

def rigid_transform_from_points(X1, X2, with_scale=False):
    assert X1.shape == X2.shape
    N = X1.shape[0]
    assert N >= 3

    mu1 = X1.mean(axis=0)
    mu2 = X2.mean(axis=0)

    X1c = X1 - mu1
    X2c = X2 - mu2

    H = X1c.T @ X2c

    U, S, Vt = np.linalg.svd(H)
    V = Vt.T

    D = np.eye(3)
    if np.linalg.det(V @ U.T) < 0:
        D[2,2] = -1

    R = V @ D @ U.T

    if with_scale:
        var1 = (X1c**2).sum() / N
        s = (S * D.diagonal()).sum() / (N * var1)
    else:
        s = 1.0

    t = mu2 - s * R @ mu1

    return R, t, s

def main():

  w = 1242
  h = 375

  X1 = np.fromfile("bin/x1.raw", dtype=np.float32).reshape(-1, 3)
  X2 = np.fromfile("bin/x2.raw", dtype=np.float32).reshape(-1, 3)

  print(X1.min(), X1.max())
  print(X2.min(), X2.max())

  R, t, s = rigid_transform_from_points(X1, X2, with_scale=True)
  print(R)
  print(t)
  print(s)
  print("------CHECK------")

  print("X1[:5]:", X1[:5])
  print("X2[:5]:", X2[:5])

  print("------")
  
  X1_transformed = s * (R @ X1.T).T + t
  print(X1_transformed)
  print(X2)
  error = np.linalg.norm(X1_transformed - X2, axis=1)
  print("Mean error:", error.mean())
  print("Max error:", error.max())

  K = np.array([[7.215377e+02, 0.000000e+00, 6.095593e+02], [0.000000e+00, 7.215377e+02, 1.728540e+02], [0.000000e+00, 0.000000e+00, 1.000000e+00]])

  pixels = []

  for i in range(len(X1)):
    x2 = s*(R @ X1[i]) + t
    if (x2[2] <= 0) :
      continue
    p = K @ x2
    p[0] /= p[2]
    p[1] /= p[2]
    if (p[0] > 0 and p[1] > 0) :
      pixels.append((int(p[0]), int(p[1]))) 
    #print(int(p[0]), int(p[1]))

  img_data2 = np.fromfile("bin/frame2.raw", dtype=np.uint8).reshape(h, w)
  
  if pixels:
    xs, ys = zip(*pixels)
  else:
      xs, ys = [], []

  plt.figure(figsize=(8, 6))
  plt.imshow(img_data2, cmap='gray', origin='upper')
  plt.scatter(xs, ys, c='lime', s=5, marker='o')
  plt.title("Frame 2")
  plt.axis('off')
  plt.tight_layout()
  plt.show()

if __name__ == "__main__":
  main()