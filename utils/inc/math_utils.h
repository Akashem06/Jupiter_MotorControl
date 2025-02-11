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

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup Math_Utils Math Utilities
 * @brief    Math utilities for motor control
 * @{
 */

/** @brief  Pi */
#define MATH_PI 3.14159265358979323846f

/**
 * @brief   Clamp a provided value between a minimum and maximum
 * @details If the value is between the min and max, it shall remain the same
 * @param   value Value to be clamped
 * @param   min Minimum of the value
 * @param   max Maximum of the value
 * @return  Clamped value
 */
float clamp(float value, float min, float max);

/** @} */
