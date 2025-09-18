#include "../../inc/data_structs.h"
#include "../../inc/feature_detect/feature_detect.h"

int is_fast_corner(Img *image, int x, int y, int threshold) {
  if (x < 3 || y < 3 || x >= image->w - 3 || y >= image->h - 3)
      return 0;
  
  uint8_t center = image->pixels[y * image->w + x];
  int brighter = 0, darker = 0;
  
  int circle[16][2] = {
    {0, -3}, {1, -3}, {2, -2}, {3, -1},
    {3, 0}, {3, 1}, {2, 2}, {1, 3},
    {0, 3}, {-1, 3}, {-2, 2}, {-3, 1},
    {-3, 0}, {-3, -1}, {-2, -2}, {-1, -3}
  };
  
  for (int i = 0; i < 16; i++) {
    int dx = circle[i][0], dy = circle[i][1];
    uint8_t pixel = image->pixels[(y + dy) * image->w + (x + dx)];
    
    if (pixel > center + threshold) brighter++;
    if (pixel < center - threshold) darker++;
  }
  
  return (brighter >= FAST_ARC_THRESHOLD) || (darker >= FAST_ARC_THRESHOLD);
}

Keypoint* detect_fast_keypoints(Img *image, int *num_keypoints) {
  Keypoint *keypoints = malloc(image->w * image->h * sizeof(Keypoint));
  *num_keypoints = 0;
  
  for (int y = 3; y < image->h - 3; y++) {
    for (int x = 3; x < image->w - 3; x++) {
      if (is_fast_corner(image, x, y, FAST_THRESHOLD)) {
        keypoints[*num_keypoints].x = x;
        keypoints[*num_keypoints].y = y;
        keypoints[*num_keypoints].response = 1.0f;
        (*num_keypoints)++;
      }
    }
  }
  
  return realloc(keypoints, *num_keypoints * sizeof(Keypoint));
}