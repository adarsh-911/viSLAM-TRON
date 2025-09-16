#include "../../inc/feature_detect/feature_detect.h"

int pattern[BRIEF_DESCRIPTOR_SIZE][4];
int pattern_initialized = 0;

void initialize_brief_pattern() {
  if (!pattern_initialized) {
    srand(42);
    generate_test_pattern(pattern, BRIEF_DESCRIPTOR_SIZE);
    pattern_initialized = 1;
  }
}

Feature* extract_features(Img *img, int *num_features) {
  
  initialize_brief_pattern();

  int num_kps;
  Keypoint *keypoints = detect_fast_keypoints(img, &num_kps);
  
  Feature *features = malloc(num_kps * sizeof(Feature));
  
  for (int i = 0; i < num_kps; i++) {
    features[i].keypoint = keypoints[i];
    features[i].descriptor = compute_brief_descriptor(img, keypoints[i], pattern);
  }
  
  free(keypoints);
  *num_features = num_kps;
  return features;
}

int hamming_distance(uint64_t a, uint64_t b) {
  return __builtin_popcountll(a ^ b);
}

int brief_distance(BriefDescriptor d1, BriefDescriptor d2) {
  int distance = 0;
  for (int i = 0; i < 4; i++) {
    distance += hamming_distance(d1.descriptor[i], d2.descriptor[i]);
  }
  return distance;
}