#include "../../inc/data_structs.h"
#include "../../inc/feature_detect/feature_detect.h"
#include "../../inc/feature_match/feature_match.h"

#include <stdio.h>

Match* match_features(Feature *features1, int num_features1, Feature *features2, int num_features2, int *num_matches, int max_distance) {
    
  Match *matches = malloc(num_features1 * sizeof(Match));
  *num_matches = 0;
  
  for (int i = 0; i < num_features1; i++) {
    int best_idx = -1;
    int best_distance = INT_MAX;
    int second_best_distance = INT_MAX;
    
    for (int j = 0; j < num_features2; j++) {
      int dist = brief_distance(features1[i].descriptor, features2[j].descriptor);
      
      if (dist < best_distance) {
        second_best_distance = best_distance;
        best_distance = dist;
        best_idx = j;
      } else if (dist < second_best_distance) {
        second_best_distance = dist;
      }
    }
    
    // Lowe's ratio test
    if (best_idx != -1 && best_distance < max_distance && (second_best_distance == INT_MAX || (float)best_distance < 0.8 * second_best_distance)) {
      matches[*num_matches].idx1 = i;
      matches[*num_matches].idx2 = best_idx;
      matches[*num_matches].distance = best_distance;
      matches[*num_matches].confidence = 1.0 - ((float)best_distance / max_distance);
      (*num_matches)++;
    }
  }
  
  return realloc(matches, *num_matches * sizeof(Match));
}

Correspondence* get_correspondences(Feature *features1, Feature *features2, Match *matches, int num_matches) {
    
  Correspondence *corrs = malloc(num_matches * sizeof(Correspondence));

  //printf("%d\n", num_matches);
  
  for (int i = 0; i < num_matches; i++) {
    int idx1 = matches[i].idx1;
    int idx2 = matches[i].idx2;
    
    //printf("%d, %d\n", idx1, idx2);

    corrs[i].x1 = features1[idx1].keypoint.x;
    corrs[i].y1 = features1[idx1].keypoint.y;
    corrs[i].x2 = features2[idx2].keypoint.x;
    corrs[i].y2 = features2[idx2].keypoint.y;
    corrs[i].confidence = matches[i].confidence;
  }
  
  return corrs;
}