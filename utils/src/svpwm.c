/*******************************************************************************************************************************
 * @file   svpwm.c
 *
 * @brief  Source file for Space vector Pulse-width Modulation
 *
 * @date   2025-08-03
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stddef.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "svpwm.h"
#include "math_utils.h"

UtilsError_t svpwm_generate(float theta_e, float vref_mag, float *duty_A, float *duty_B, float *duty_C) {
  if (duty_A == NULL || duty_B == NULL || duty_C == NULL) {
    return UTILS_INVALID_ARGS;
  }

  theta_e = normalize_angle(theta_e);

  if (vref_mag > 1.0f) vref_mag = 1.0f;
  if (vref_mag < 0.0f) vref_mag = 0.0f;

  uint8_t sector_num = (uint8_t)(theta_e / MATH_PI_OVER_3);     /**< Sector number */
  float sector_theta = theta_e - (sector_num * MATH_PI_OVER_3); /**< Angle within the sector */

  float sin_a, cos_a;
  fast_sin_cos(sector_theta, &sin_a, &cos_a);

  float sin_b, cos_b;
  fast_sin_cos(MATH_PI_OVER_3 - sector_theta, &sin_b, &cos_b);

  /*
   * T1 = Percentage of time in sector B (Nearest sector)
   * T2 = Percentage of time in sector A (Other nearest sector)
   * T0 = Percentage of time in null sector
   */
  float T1 = vref_mag * sin_b * INV_SQRT3_OVER_2;
  float T2 = vref_mag * sin_a * INV_SQRT3_OVER_2;
  float T0 = 1.0f - T1 - T2;

  float Ta, Tb, Tc;

  /**
   * The switching times T1 and T2 determine how long we apply the two active space vectors
   * in the current sector, and T0 is the remaining zero vector time. For center-aligned PWM,
   * T0 is evenly split at the start and end of each PWM cycle.
   *
   * Each sector corresponds to a pair of adjacent active vectors:
   * - Sector 0 [100]: V1 (0°), V2 (60°)
   * - Sector 1 [110]: V2 (60°), V3 (120°)
   * - Sector 2 [010]: V3 (120°), V4 (180°)
   * - Sector 3 [011]: V4 (180°), V5 (240°)
   * - Sector 4 [001]: V5 (240°), V6 (300°)
   * - Sector 5 [101]: V6 (300°), V1 (360° or 0° again)
   *
   * The duty cycles (Ta, Tb, Tc) for phases A, B, and C are assigned based on which sector we're in.
   * This mapping ensures proper synthesis of the reference vector using the two active vectors.
   */
  switch (sector_num) {
    case 0U:
      Ta = (T1 + T2 + T0 / 2.0f); /**< Phase A: leading phase */
      Tb = (T2 + T0 / 2.0f);      /**< Phase B: second active vector */
      Tc = (T0 / 2.0f);           /**< Phase C: lagging phase */
      break;

    case 1U:
      Ta = (T1 + T0 / 2.0f);      /**< Phase A: second active vector */
      Tb = (T1 + T2 + T0 / 2.0f); /**< Phase B: leading phase */
      Tc = (T0 / 2.0f);           /**< Phase C: lagging phase */
      break;

    case 2U:
      Ta = (T0 / 2.0f);           /**< Phase A: lagging phase */
      Tb = (T1 + T2 + T0 / 2.0f); /**< Phase B: leading phase */
      Tc = (T2 + T0 / 2.0f);      /**< Phase C: second active vector */
      break;

    case 3U:
      Ta = (T0 / 2.0f);           /**< Phase A: lagging phase */
      Tb = (T1 + T0 / 2.0f);      /**< Phase B: second active vector */
      Tc = (T1 + T2 + T0 / 2.0f); /**< Phase C: leading phase */
      break;

    case 4U:
      Ta = (T2 + T0 / 2.0f);      /**< Phase A: second active vector */
      Tb = (T0 / 2.0f);           /**< Phase B: lagging phase */
      Tc = (T1 + T2 + T0 / 2.0f); /**< Phase C: leading phase */
      break;

    case 5U:
      Ta = (T1 + T2 + T0 / 2.0f); /**< Phase A: leading phase */
      Tb = (T0 / 2.0f);           /**< Phase B: lagging phase */
      Tc = (T1 + T0 / 2.0f);      /**< Phase C: second active vector */
      break;

    default:
      return UTILS_INTERNAL_ERROR;
  }

  *duty_A = Ta;
  *duty_B = Tb;
  *duty_C = Tc;

  return UTILS_OK;
}
