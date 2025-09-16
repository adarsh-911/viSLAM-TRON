#include "../inc/math_structs.h"

mat3 compute_inv(const mat3 mat) {
  float a = mat.m[0][0], b = mat.m[0][1], c = mat.m[0][2];
  float d = mat.m[1][0], e = mat.m[1][1], f = mat.m[1][2];
  float g = mat.m[2][0], h = mat.m[2][1], i = mat.m[2][2];
  
  mat3 matOut;

  float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
  if (det == 0.0f) return matOut;

  float invDet = 1.0f / det;

  matOut.m[0][0] =  (e*i - f*h) * invDet;
  matOut.m[0][1] = -(b*i - c*h) * invDet;
  matOut.m[0][2] =  (b*f - c*e) * invDet;

  matOut.m[1][0] = -(d*i - f*g) * invDet;
  matOut.m[1][1] =  (a*i - c*g) * invDet;
  matOut.m[1][2] = -(a*f - c*d) * invDet;

  matOut.m[2][0] =  (d*h - e*g) * invDet;
  matOut.m[2][1] = -(a*h - b*g) * invDet;
  matOut.m[2][2] =  (a*e - b*d) * invDet;

  return matOut;
}

vec3 product_mat3_vec3(mat3 mat, vec3 vec) {
  vec3 out;
  out.x = (mat.m[0][0] * vec.x) + (mat.m[0][1] * vec.y) + (mat.m[0][2] * vec.z);
  out.y = (mat.m[1][0] * vec.x) + (mat.m[1][1] * vec.y) + (mat.m[1][2] * vec.z);
  out.z = (mat.m[2][0] * vec.x) + (mat.m[2][1] * vec.y) + (mat.m[2][2] * vec.z);

  return out;
}