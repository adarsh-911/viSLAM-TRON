#pragma once

#include "../data_structs.h"

#define FAST_THRESHOLD 20
#define FAST_ARC_THRESHOLD 10
#define ENABLE_NMS 0
#define BRIEF_DESCRIPTOR_SIZE 256

Keypoint* detect_fast_keypoints(Img *image, int *num_keypoints);
BriefDescriptor compute_brief_descriptor(Img *image, Keypoint kp, int pattern[][4]);
void generate_test_pattern(int pattern[][4], int size);
Feature* extract_features(Img *img, int *num_features);
int brief_distance(BriefDescriptor d1, BriefDescriptor d2);