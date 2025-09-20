#include "inc/init_map/init_map.h"

#define MAX_WORLD_POINTS 100

int main() {

  // Current world points
  vec3 CURRENT_WORLD_POINTS[MAX_WORLD_POINTS];

  // Current Pose
  mat3 R; vec3 t;

  // Map Init
  init_map(CURRENT_WORLD_POINTS, (int) MAX_WORLD_POINTS, &R, &t);

  // Print the current pose
  printf("Recovered pose :\n");
  mat3_print("R", R);
  vec3_print("t", t);

  return 0;
}