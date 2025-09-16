#pragma once

#include "math_structs.h"

typedef struct {
  uint8_t *pixels;
  int w, h;
} Img;

typedef struct {
  int x, y;
  float response;
} Keypoint;

typedef struct {
  int x;
  int y;
} RawKP;

typedef struct {
  uint64_t descriptor[4]; // 256 bits
} BriefDescriptor;

typedef struct {
  Keypoint keypoint;
  BriefDescriptor descriptor;
} Feature;

typedef struct {
  int idx1, idx2;
  int distance;
  float confidence;  // Match quality (0-1)
} Match;

typedef struct {
  int x1, y1;
  int x2, y2;
  float confidence;
} Correspondence;