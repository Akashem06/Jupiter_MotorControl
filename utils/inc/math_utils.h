#pragma once

/*******************************************************************************************************************************
 * @file   math_utils.h
 *
 * @brief  Header file for Math utilities
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup Math_Utils Math Utilities
 * @brief    Math utilities for 3-phase inverters
 * @{
 */

/** @brief  Pi */
#define MATH_PI 3.141592653f

/** @brief  2 Pi */
#define MATH_TWO_PI 6.283185307f

/** @brief  Pi / 3 */
#define MATH_PI_OVER_3 1.047197551f

#define SQRT3 1.732050807f            /**< sqrt(3) */
#define SQRT3_OVER_2 0.866025408f     /**< sqrt(3) / 2 */
#define INV_SQRT3_OVER_2 1.154700538f /**< 2 / sqrt(3) */
#define INV_SQRT3 0.577350269f        /**< 1 / sqrt(3) */

/**
 * @brief   Clamp a provided value between a minimum and maximum
 * @details If the value is between the min and max, it shall remain the same
 * @param   value Value to be clamped
 * @param   min Minimum of the value
 * @param   max Maximum of the value
 * @return  Clamped value
 */
float clamp(float value, float min, float max);

/**
 * @brief   Returns the minimum float value between two provided values
 * @param   value_1 First float to compare with
 * @param   value_2 Second float to compare with
 * @return  Minimum float value of the two parameters
 */
float fminf(float value_1, float value_2);

/**
 * @brief   Returns the absolute valued float
 * @param   x Value subject to an absolute value operation
 * @return  Absolute value version of x
 */
float fabsf(float x);

/**
 * @brief   Returns the square root of a float
 * @param   x Value subject to a square root operation
 * @return  Square root of x
 */
float sqrtf(float x);

/**
 * @brief   Normalize angle into [0, 2π)
 * @param   angle Input angle (radians)
 * @return  Normalized angle (radians)
 */
float normalize_angle(float angle);

/**
 * @brief   Convert mechanical angle to electrical angle
 * @param   mechanical_angle Mechanical angle (radians)
 * @param   pole_pairs Number of motor pole pairs
 * @return  Electrical angle (radians)
 */
float mech_to_elec_angle(float mechanical_angle, uint8_t pole_pairs);

/**
 * @brief   Compute sine and cosine of an angle efficiently
 * @param   angle Input angle (radians)
 * @param   sin_out Pointer to store sine result
 * @param   cos_out Pointer to store cosine result
 */
void fast_sin_cos(float angle, float *sin_out, float *cos_out);

/** @} */
