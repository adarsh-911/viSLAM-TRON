#pragma once

#include "../data_structs.h"

Match* match_features(Feature *features1, int num_features1, Feature *features2, int num_features2, int *num_matches, int max_distance);
Correspondence* get_correspondences(Feature *features1, Feature *features2, Match *matches, int num_matches);