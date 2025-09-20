#include "../../inc/svd.h"
#include "../../inc/data_structs.h"
#include "../../inc/math_structs.h"

// Jacobi for SVD
void svd_mat3(mat3 M, mat3 *U, vec3 *S, mat3 *V) {
  for (int i = 0 ; i < 3 ; i++) for (int j = 0 ; j < 3 ; j++) {
      U->m[i][j] = M.m[i][j];
      V->m[i][j] = (i == j ? 1 : 0);
  }

  for (int iter = 0 ; iter < 50 ; iter++) {
    for (int p = 0 ; p < 2 ; p++) {
      for (int q = p+1 ; q < 3 ; q++) {

        float alpha = 0, beta = 0, gamma = 0;
        for (int k = 0 ; k < 3 ; k++) {
          alpha += U->m[k][p]*U->m[k][p];
          beta  += U->m[k][q]*U->m[k][q];
          gamma += U->m[k][p]*U->m[k][q];
        }
        if (fabs(gamma) < 1e-10) continue;
        float zeta = (beta - alpha)/(2 * gamma);
        float t = (zeta >= 0 ? 1.0f : -1.0f)/(fabs(zeta) + sqrt(1+zeta*zeta));
        float c = 1/sqrt(1+t*t);
        float s = c*t;

        for (int k = 0 ; k < 3 ; k++) {
          float up = U->m[k][p], uq = U->m[k][q];
          U->m[k][p] = c*up - s*uq;
          U->m[k][q] = s*up + c*uq;
        }

        for (int k = 0 ; k < 3 ; k++) {
          float vp = V->m[k][p], vq = V->m[k][q];
          V->m[k][p] = c*vp - s*vq;
          V->m[k][q] = s*vp + c*vq;
        }
      }
    }
  }

  float temp_s[3]; 

  for (int j = 0 ; j < 3 ; j++) {
    float norm = 0;
    for (int i = 0 ; i < 3 ; i++) norm += U->m[i][j]*U->m[i][j];
    norm = sqrt(norm);
    temp_s[j] = norm;
    for (int i = 0 ; i < 3 ; i++) U->m[i][j] /= (norm > 1e-12 ? norm : 1);
  }

  S->x = temp_s[0]; S->y = temp_s[1]; S->z = temp_s[2];
}

void rigid_transform(vec3 *A, vec3 *B, int N, mat3 *R, vec3 *t) {
  vec3 muA = {0,0,0}, muB = {0,0,0};
  for(int i = 0 ; i < N ; i++) {
    muA.x += A[i].x; muA.y += A[i].y; muA.z += A[i].z;
    muB.x += B[i].x; muB.y += B[i].y; muB.z += B[i].z;
  }
  muA.x /= N; muA.y /= N; muA.z /= N;
  muB.x /= N; muB.y /= N; muB.z /= N;

  float PA[N][3], PB[N][3];
  for(int i = 0 ; i < N ; i++) {
    PA[i][0] = A[i].x - muA.x;
    PA[i][1] = A[i].y - muA.y;
    PA[i][2] = A[i].z - muA.z;
    PB[i][0] = B[i].x - muB.x;
    PB[i][1] = B[i].y - muB.y;
    PB[i][2] = B[i].z - muB.z;
  }

  mat3 H = (mat3){{0}};
  for(int i = 0 ; i < N ; i++) {
    for(int r = 0 ; r < 3 ; r++) for(int c = 0 ; c < 3 ; c++) {
      H.m[r][c] += PA[i][r] * PB[i][c];
    }
  }

  mat3 U, V;
  vec3 S;
  svd_mat3(H,&U,&S,&V);

  mat3 Ut, tmp;
  mat3_transpose(U, &Ut);
  mat3_mult(V, Ut ,R);

  if (mat3_det(R) < 0) {
    for(int i = 0 ; i < 3 ; i++) V.m[i][2] *= -1;
    mat3_mult(V,Ut,R);
  }

  float rmu[3], temp[3];

  for(int i = 0 ; i < 3 ; i++) rmu[i] = R->m[i][0]*muA.x + R->m[i][1]*muA.y + R->m[i][2]*muA.z;
  for(int i = 0 ; i < 3 ; i++) temp[i] = (&muB.x)[i]-rmu[i];

  t->x = temp[0]; t->y = temp[1]; t->z = temp[2];
}

void recover_pose_svd(vec3 *kp1, vec3 *kp2, int size, mat3 *R, vec3 *t) {

  rigid_transform(kp1, kp2, size, R, t);

  return;
}
