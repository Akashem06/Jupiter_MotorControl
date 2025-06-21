/*******************************************************************************************************************************
 * @file   math_utils.c
 *
 * @brief  Source file for Math utilities
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <math.h>  // To be removed for embedded applications

/* Inter-component Headers */

/* Intra-component Headers */
#include "math_utils.h"

float clamp(float value, float min, float max) {
  if (value >= max) {
    return max;
  } else if (value <= min) {
    return min;
  } else {
    return value;
  }
}

float fminf(float value_1, float value_2) {
  return value_1 < value_2 ? value_1 : value_2;
}

float normalize_angle(float angle) {
  while (angle >= MATH_TWO_PI) angle -= MATH_TWO_PI;
  while (angle < 0.0f) angle += MATH_TWO_PI;
  return angle;
}

float mech_to_elec_angle(float mechanical_angle, uint8_t pole_pairs) {
  float electrical_angle = mechanical_angle * (float)pole_pairs;
  return normalize_angle(electrical_angle);
}

void fast_sin_cos(float angle, float *sin_out, float *cos_out) {
  *sin_out = sinf(angle);
  *cos_out = cosf(angle);
}
