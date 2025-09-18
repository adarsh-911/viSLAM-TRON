#include "../../inc/intrinsics.h"
#include "../../inc/math_structs.h"
#include "../../inc/loader.h"
#include "../../inc/feature_detect/feature_detect.h"
#include "../../inc/feature_match/feature_match.h"
#include "../../inc/save_files.h"

Img frame1, frame2;
Feature *features_1, *features_2;

int load_frames() {

  bool status1, status2;
  load_image("dataset/00.png", &frame1, &status1);
  load_image("dataset/01.png", &frame2, &status2);

  if (!status1 || !status2) {
    printf("Error loading frames!\n");
    free_img(&frame1);
    free_img(&frame2);

    return 1;
  }

  return 0;
}

void print_point(int x, int y) {
  printf("(%d, %d)\n", x, y);

  return;
}

Correspondence* match_and_correspond (int num1, int num2, int *num_matches) {
  int max_hamming_distance = 80;
  Match *matches = match_features(features_1, num1, features_2, num2, num_matches, max_hamming_distance);

  Correspondence *correspondences = get_correspondences(features_1, features_2, matches, *num_matches);

  return correspondences;
}

void normalize_pixel_to_vec (RawKP *kp1, RawKP *kp2, RawKP *kp1_norm, RawKP *kp2_norm, int size) {
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

void find_essential_matrix (RawKP *kp1_norm, RawKP *kp2_norm, int size) {
  
}

void save_files (RawKP *kp1, RawKP* kp2, int size) {
  save_raw("bin/frame1.raw", frame1.pixels, frame1.w, frame1.h);
  save_points("bin/kp1.raw", kp1, size);

  save_raw("bin/frame2.raw", frame2.pixels, frame1.w, frame1.h);
  save_points("bin/kp2.raw", kp2, size);

  return;

}

int main() {

  if (load_frames()) return 1;

  K_MAT_INV = compute_inv(K_MAT);

  int num_features1, num_features2, num_matches;
  features_1 = extract_features(&frame1, &num_features1);
  features_2 = extract_features(&frame2, &num_features2);

  Correspondence* correspondences = match_and_correspond(num_features1, num_features2, &num_matches);
  
  RawKP kp1_raw_match[num_matches], kp2_raw_match[num_matches];

  for (int i = 0 ; i < num_matches ; i++) {
    kp1_raw_match[i].x = correspondences->x1;
    kp1_raw_match[i].y = correspondences->y1;

    kp2_raw_match[i].x = correspondences->x2;
    kp2_raw_match[i].y = correspondences->y2;

    correspondences++;
  }

  RawKP kp1_norm[num_matches], kp2_norm[num_matches];

  normalize_pixel_to_vec(kp1_raw_match, kp2_raw_match, kp1_norm, kp2_norm, num_matches);

  save_files(kp1_raw_match, kp2_raw_match, num_matches);

  return 0;
}
