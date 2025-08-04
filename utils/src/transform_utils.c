/*******************************************************************************************************************************
 * @file   transform_utils.c
 *
 * @brief  Source file for Park/Clarke transform utilities
 *
 * @date   2025-08-03
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stddef.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "transform_utils.h"
#include "math_utils.h"

UtilsError_t clarke_transform_2phase(float ia, float ib, float *alpha, float *beta) {
  if (alpha == NULL || beta == NULL) {
    return UTILS_INVALID_ARGS;
  }

  *alpha = ia;
  *beta = (ia + (2.0f * ib)) * (INV_SQRT3);

  return UTILS_OK;
}

UtilsError_t clarke_transform_3phase(float ia, float ib, float ic, float *alpha, float *beta) {
  if (alpha == NULL || beta == NULL) {
    return UTILS_INVALID_ARGS;
  }

  *alpha = ia;
  *beta = (INV_SQRT3) * (ib - ic);

  return UTILS_OK;
}

UtilsError_t park_transform(float alpha, float beta, float theta, float *d, float *q) {
  if (d == NULL || q == NULL) {
    return UTILS_INVALID_ARGS;
  }

  float sin_theta, cos_theta;
  fast_sin_cos(theta, &sin_theta, &cos_theta);

  *d = alpha * cos_theta + beta * sin_theta;
  *q = -alpha * sin_theta + beta * cos_theta;

  return UTILS_OK;
}

UtilsError_t inverse_park_transform(float d, float q, float theta, float *alpha, float *beta) {
  if (alpha == NULL || beta == NULL) {
    return UTILS_INVALID_ARGS;
  }

  float sin_theta, cos_theta;
  fast_sin_cos(theta, &sin_theta, &cos_theta);

  *alpha = d * cos_theta - q * sin_theta;
  *beta = d * sin_theta + q * cos_theta;

  return UTILS_OK;
}
