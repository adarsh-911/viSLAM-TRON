#include "inc/init_map/init_map.h"
#include "inc/tracking/track_frames.h"
#include "inc/loader.h"
#include "inc/intrinsics.h"
#include "inc/file_manager.h"

#define MAX_WORLD_POINTS 1000
 
void save_files (Img frame1, Img frame2, RawKP *kp1, RawKP* kp2, int size) {
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
  Pose currPose;

  // Assign space for depth map
  uint8_t *depth1 = (uint8_t *) malloc(sizeof(uint8_t) * HEIGHT * WIDTH);
  uint8_t *depth2 = (uint8_t *) malloc(sizeof(uint8_t) * HEIGHT * WIDTH);

  // Generate depth map
  get_depth_map("bin/depth_map_00.raw", depth1, HEIGHT, WIDTH);
  get_depth_map("bin/depth_map_01.raw", depth2, HEIGHT, WIDTH);

  // Map Init (get raw 2D points)
  RawKP pixels1[MAX_WORLD_POINTS], pixels2[MAX_WORLD_POINTS];
  int num_matches;
  Img frame1, frame2;
  if (load_frames("dataset/00.png", &frame1, "dataset/01.png", &frame2)) return 1;
  init_map(&frame1, &frame2, K_MAT, depth1, depth2, &num_matches, CURRENT_WORLD_POINTS, (int) MAX_WORLD_POINTS, &currPose, "yes", pixels1, pixels2);

  int minSize = MAX_WORLD_POINTS;
  if (num_matches < (int)MAX_WORLD_POINTS) minSize = num_matches;

  printf("----\n num matches = %d\n----\n", minSize);

  // Save raw 2D points
  save_files (frame1, frame2, pixels1, pixels2, minSize);

  // Save world points
  save_world_points("bin/WorldPoints.bin", CURRENT_WORLD_POINTS, minSize);

  // Print world space points
  printf("World space points :\n");
  for (int i = 0 ; i < 10 ; i++) vec3_print("Point", CURRENT_WORLD_POINTS[i]);
  puts("...");

  // Print the current pose
  printf("Recovered pose :\n");
  mat3_print("R", currPose.R);
  vec3_print("t", currPose.t);
  printf("s : %f\n", currPose.s);

  printf("-----------------------------------------\n");
  printf("Map Initialization successful!\n");
  printf("-----------------------------------------\n");
  printf("Tracking...\n");
  
  // Tracking
  Img frame3; bool st;
  load_image("dataset/02.png", &frame3, &st);
  tracking_thread(CURRENT_WORLD_POINTS, K_MAT, &frame3, &frame2, &currPose, minSize);
  
  return 0;
}