#pragma once

#include "lucasKanade.h"
#include "pose_only_ba.h"

int tracking_thread (vec3* worldPoints, const mat3 K_MAT, Img* currFrame, Img* recentFrame, Pose* recentPose, int numPoints);

extern Pose currOptimalPose;