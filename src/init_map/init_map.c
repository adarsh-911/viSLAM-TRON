#include "../../inc/intrinsics.h"
#include "../../inc/math_structs.h"
#include "../../inc/feature_detect/feature_detect.h"
#include "../../inc/feature_match/feature_match.h"
#include "../../inc/svd.h"
#include "../../inc/init_map/init_map.h"
#include "../../inc/loader.h"

Img frame1, frame2;
int num_matches;
mat3 K_MAT_INV;

RawKP *match1, *match2;

Correspondence* match_and_correspond (int num1, int num2, Feature *features_1, Feature* features_2, int *num_matches) {

  Match *matches = match_features(features_1, num1, features_2, num2, num_matches, MAX_HAMMING_DISTANCE);

  Correspondence *correspondences = get_correspondences(features_1, features_2, matches, *num_matches);

  return correspondences;
}

void normalize_pixel_to_vec (RawKP *kp1, RawKP *kp2, vec2 *kp1_norm, vec2 *kp2_norm, int size) {
  int i = 0;
  while (i < size) {
    vec3 curr_pixel1 = {kp1[i].x, kp1[i].y, 1.0f};
    vec3 dir1 = product_mat3_vec3(K_MAT_INV, curr_pixel1);
    kp1_norm[i].x = dir1.x/dir1.z;
    kp1_norm[i].y = dir1.y/dir1.z;

    vec3 curr_pixel2 = {kp2[i].x, kp2[i].y, 1.0f};
    vec3 dir2 = product_mat3_vec3(K_MAT_INV, curr_pixel2);
    kp2_norm[i].x = dir2.x/dir2.z;
    kp2_norm[i].y = dir2.y/dir2.z;

    i++;
  }
}

void extract_depth_from_npu (RawKP *kp1, float *kp1_depth, RawKP *kp2, float *kp2_depth, int size) {
  for (int i = 0 ; i < size ; i++) {
    kp1_depth[i] = 1.0f * SCALING_FACTOR_DEPTH_MODEL;
    //kp1_depth[i] = get_depth_from_npu(kp1[i].x, kp1[i].y, &frame1) * SCALING_FACTOR_DEPTH_MODEL;

    kp2_depth[i] = 0.5f * SCALING_FACTOR_DEPTH_MODEL;
    //kp2_depth[i] = get_depth_from_npu(kp2[i].x, kp2[i].y, &frame2) * SCALING_FACTOR_DEPTH_MODEL;
  }

  return;
}

void pack_to_point3 (vec2 *kp1_norm, float *kp1_depth, vec3 *kp1_cam1, vec2 *kp2_norm, float* kp2_depth, vec3 *kp2_cam2, int size) {
  for (int i = 0 ; i < size ; i++) {
    kp1_cam1[i].x = kp1_norm[i].x; kp1_cam1[i].y = kp1_norm[i].y;
    kp1_cam1[i].z = kp1_depth[i];

    kp2_cam2[i].x = kp2_norm[i].x; kp2_cam2[i].y = kp2_norm[i].y;
    kp2_cam2[i].z = kp2_depth[i];
  }

  return;
}

void get_raw_points (RawKP *points1, RawKP *points2, int size) {
  for (int i = 0 ; i < size ; i++) {
    if (i == num_matches) break;

    points1[i] = match1[i];
    points2[i] = match2[i];
  }

  return;
}

int init_map(char *frame_1, char *frame_2, vec3 *world_points, int max, mat3 *R, vec3 *t, char *getRaw, RawKP *raw_match1, RawKP *raw_match2) {

  if (load_frames(frame_1, &frame1, frame_2, &frame2)) return 1;

  K_MAT_INV = compute_inv(K_MAT);

  int num_features1, num_features2;
  Feature *features_1 = extract_features(&frame1, &num_features1);
  Feature *features_2 = extract_features(&frame2, &num_features2);

  Correspondence* correspondences = match_and_correspond(num_features1, num_features2, features_1, features_2, &num_matches);
  
  RawKP kp1_raw_match[num_matches], kp2_raw_match[num_matches];

  for (int i = 0 ; i < num_matches ; i++) {
    kp1_raw_match[i].x = correspondences->x1;
    kp1_raw_match[i].y = correspondences->y1;

    kp2_raw_match[i].x = correspondences->x2;
    kp2_raw_match[i].y = correspondences->y2;

    correspondences++;
  }

  match1 = kp1_raw_match;
  match2 = kp2_raw_match;

  if (strcmp(getRaw, "yes") == 0) get_raw_points(raw_match1, raw_match2, max);

  vec2 kp1_norm[num_matches], kp2_norm[num_matches];
  float kp1_depth[num_matches], kp2_depth[num_matches];

  vec3 kp1_cam1[num_matches], kp2_cam2[num_matches];

  normalize_pixel_to_vec(kp1_raw_match, kp2_raw_match, kp1_norm, kp2_norm, num_matches);

  extract_depth_from_npu(kp1_raw_match, kp1_depth, kp2_raw_match, kp2_depth, num_matches);

  pack_to_point3(kp1_norm, kp1_depth, kp1_cam1, kp2_norm, kp2_depth, kp2_cam2, num_matches);

  for (int i = 0 ; i < max ; i++) {
    if (i == num_matches) break;
    world_points[i] = kp1_cam1[i];
  }

  recover_pose_svd(kp1_cam1, kp2_cam2, num_matches, R, t);

  return 0;
}
