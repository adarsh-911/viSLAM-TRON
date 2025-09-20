#include "inc/init_map/init_map.h"
#include "inc/save_files.h"

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

  // Map Init (get raw 2D points)
  RawKP pixels1[MAX_WORLD_POINTS], pixels2[MAX_WORLD_POINTS];
  init_map("dataset/00.png", "dataset/01.png", CURRENT_WORLD_POINTS, (int) MAX_WORLD_POINTS, &R, &t, "yes", pixels1, pixels2);

  // Save raw points
  save_files (pixels1, pixels2, MAX_WORLD_POINTS);

  // Print world space points
  printf("World space points :\n");
  for (int i = 0 ; i < 10 ; i++) vec3_print("Point", CURRENT_WORLD_POINTS[i]);

  // Print the current pose
  printf("Recovered pose :\n");
  mat3_print("R", R);
  vec3_print("t", t);

  return 0;
}