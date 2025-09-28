#pragma once

#include "data_structs.h"

void recover_pose_svd(vec3 *kp1, vec3 *kp2, int size, mat3 *R, vec3 *t, float *s);