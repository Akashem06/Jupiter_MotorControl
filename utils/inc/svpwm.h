#pragma once

/*******************************************************************************************************************************
 * @file   svpwm.h
 *
 * @brief  Header file for Space vector Pulse-width Modulation
 *
 * @date   2025-08-03
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "utils_error.h"

/**
 * @defgroup SVPWM_Utils Space vector Pulse-width Modulation
 * @brief    Space vector Pulse-width Modulation math utilities for 3-phase inverters
 * @{
 */

/**
 * @brief   Generate SVPWM duty cycles using angle-based sector detection
 * @param   theta_e Electrical angle (radians, normalized 0 to 2π)
 * @param   vref_mag Normalized voltage magnitude (0.0 to 1.0, modulation index)
 * @param   duty_A Pointer to store phase A duty cycle (0.0 to 1.0)
 * @param   duty_B Pointer to store phase B duty cycle (0.0 to 1.0)
 * @param   duty_C Pointer to store phase C duty cycle (0.0 to 1.0)
 * @return  MOTOR_OK if successful, MOTOR_INVALID_ARGS if null pointers
 */
UtilsError_t svpwm_generate(float theta_e, float vref_mag, float *duty_A, float *duty_B, float *duty_C);

/** @} */
