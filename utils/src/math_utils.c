/*******************************************************************************************************************************
 * @file   math_utils.c
 *
 * @brief  Source file for Math utilities
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

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
