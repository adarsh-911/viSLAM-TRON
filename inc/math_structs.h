#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>

typedef struct Vec2 {
  float x, y;
} vec2;

typedef struct Vec3 {
  float x, y, z;
} vec3;

typedef struct Vec4 {
  float x, y, z, w;
} vec4;

typedef struct Mat3 {
  float m[3][3];
} mat3;

typedef struct Mat4 {
  float m[4][4];
} mat4;

vec2 vec2_add (vec2 a, vec2 b);
vec3 vec3_add (vec3 a, vec3 b);
vec4 vec4_add (vec4 a, vec4 b);

vec3 product_mat3_vec3(mat3 mat, vec3 vec);

mat3 compute_inv(const mat3 mat);
