#pragma once

#include "../data_structs.h"

#define SCALING_FACTOR_DEPTH_MODEL (-0.002)
#define MAX_HAMMING_DISTANCE 30

extern Img frame1, frame2;

int init_map(char *frame_1, char *frame_2, uint8_t *depth1, uint8_t *depth2, int *num, vec3 *world_points, int max, mat3 *R, vec3 *t, char *getRaw, RawKP *raw_match1, RawKP *raw_match2);
void get_raw_points (RawKP *points1, RawKP *points2, int size);