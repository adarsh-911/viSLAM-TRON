#pragma once

#include <stdint.h>
#include <stdio.h>
#include "data_structs.h"

void save_raw(const char *filename, uint8_t *image, int width, int height) {
  FILE *fp = fopen(filename, "wb");
  if (!fp) {
    perror("Failed to open file");
    return;
  }

  size_t written = fwrite(image, sizeof(uint8_t), width * height, fp);
  if (written != (size_t)(width * height)) {
    perror("Failed to write full image");
  }

  fclose(fp);
}

void save_points(const char *filename, RawKP *points, int count) {
  FILE *fp = fopen(filename, "wb");
  if (!fp) {
    perror("Failed to open file");
    return;
  }

  size_t written = fwrite(points, sizeof(RawKP), count, fp);
  if (written != (size_t)count) {
    perror("Failed to write all points");
  }

  fclose(fp);
}