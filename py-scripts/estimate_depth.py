import sys
import cv2
import numpy as np
import onnxruntime as ort

w = 1242
h = 375

# Load model
session = ort.InferenceSession("py-scripts/model.onnx")

base_dir = "dataset/"

# Load image
img = cv2.imread(base_dir + str(sys.argv[1]) + ".png", cv2.IMREAD_GRAYSCALE)
img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
img_resized = cv2.resize(img_rgb, (w, h))
img_float = img_resized.astype(np.float32) / 255.0

# Pre Process
input_data = np.transpose(img_float, (2, 0, 1))
input_data = np.expand_dims(input_data, axis=0)
#input_data = np.expand_dims(img_rgb, axis=0).astype(np.float32)

# Run inference
input_name = session.get_inputs()[0].name
output_name = session.get_outputs()[0].name
depth_map = session.run([output_name], {input_name: input_data})[0]

# Post process
depth_map = np.squeeze(depth_map)
#depth_map = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
#depth_map = depth_map.astype(np.uint8)

#depth_map.tofile(f"bin/depth_map_{sys.argv[1]}.raw")
#print(depth_map.shape)

depth_map_fullres = cv2.resize(depth_map, (w, h), interpolation=cv2.INTER_LINEAR)

# Normalize and save
depth_map_fullres_norm = cv2.normalize(depth_map_fullres, None, 0, 255, cv2.NORM_MINMAX)
depth_map_fullres_norm = depth_map_fullres_norm.astype(np.uint8)

depth_map_fullres_norm.tofile(f"bin/depth_map_{sys.argv[1]}.raw")

#cv2.imshow("Depth Map", depth_map_fullres_norm)
#cv2.imwrite("depth_map_f1.png", depth_map)
#cv2.waitKey(0)