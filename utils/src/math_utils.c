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
