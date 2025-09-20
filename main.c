#include "inc/init_map/init_map.h"
#include "inc/file_manager.h"

#define MAX_WORLD_POINTS 800
 
void save_files (RawKP *kp1, RawKP* kp2, int size) {
  save_raw("bin/frame1.raw", frame1.pixels, frame1.w, frame1.h);
  save_points("bin/kp1.raw", kp1, size);

  save_raw("bin/frame2.raw", frame2.pixels, frame1.w, frame1.h);
  save_points("bin/kp2.raw", kp2, size);

  return;

}

int main() {

  // Current world points
  vec3 CURRENT_WORLD_POINTS[MAX_WORLD_POINTS];

  // Current Pose
  mat3 R; vec3 t;

  // Assign space for depth map
  uint8_t *depth1 = (uint8_t *) malloc(sizeof(uint8_t) * HEIGHT * WIDTH);
  uint8_t *depth2 = (uint8_t *) malloc(sizeof(uint8_t) * HEIGHT * WIDTH);

  // Generate depth map
  get_depth_map("bin/depth_map_00.raw", depth1, HEIGHT, WIDTH);
  get_depth_map("bin/depth_map_01.raw", depth2, HEIGHT, WIDTH);

  // Map Init (get raw 2D points)
  RawKP pixels1[MAX_WORLD_POINTS], pixels2[MAX_WORLD_POINTS];
  init_map("dataset/00.png", "dataset/01.png", depth1, depth2, CURRENT_WORLD_POINTS, (int) MAX_WORLD_POINTS, &R, &t, "yes", pixels1, pixels2);

  // Save raw 2D points
  save_files (pixels1, pixels2, MAX_WORLD_POINTS);

  // Save world points
  save_world_points("bin/WorldPoints.bin", CURRENT_WORLD_POINTS, MAX_WORLD_POINTS);

  // Print world space points
  printf("World space points :\n");
  for (int i = 0 ; i < 10 ; i++) vec3_print("Point", CURRENT_WORLD_POINTS[i]);
  puts("...");

  // Print the current pose
  printf("Recovered pose :\n");
  mat3_print("R", R);
  vec3_print("t", t);

  printf("-----------------------------------------\n");
  printf("Map Initialization successful!\n");

  return 0;
}