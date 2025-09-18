#include "../../inc/data_structs.h"
#include "../../inc/feature_detect/feature_detect.h"

void generate_test_pattern(int pattern[][4], int size) {
  for (int i = 0; i < size; i++) {
    pattern[i][0] = rand() % 31 - 15;
    pattern[i][1] = rand() % 31 - 15;
    pattern[i][2] = rand() % 31 - 15;
    pattern[i][3] = rand() % 31 - 15;
  }
}

BriefDescriptor compute_brief_descriptor(Img *image, Keypoint kp, int pattern[][4]) {
  BriefDescriptor desc = {0};
  
  for (int i = 0; i < BRIEF_DESCRIPTOR_SIZE; i++) {
    int x1 = kp.x + pattern[i][0];
    int y1 = kp.y + pattern[i][1];
    int x2 = kp.x + pattern[i][2];
    int y2 = kp.y + pattern[i][3];
    
    if (x1 < 0 || x1 >= image->w || y1 < 0 || y1 >= image->h ||  x2 < 0 || x2 >= image->w || y2 < 0 || y2 >= image->h) continue;
    
    uint8_t val1 = image->pixels[y1 * image->w + x1];
    uint8_t val2 = image->pixels[y2 * image->w + x2];
    
    if (val1 < val2) {
      int bit_pos = i % 64;
      int array_pos = i / 64;
      desc.descriptor[array_pos] |= (1ULL << bit_pos);
    }
  }
  
  return desc;
}