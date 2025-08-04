#pragma once

/*******************************************************************************************************************************
 * @file   transform_utils.h
 *
 * @brief  Header file for Park/Clarke transform utilities
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
 * @defgroup Transform_Utils Park/Clarke transform utilities
 * @brief    Park/Clarke transform utilities math utilities for 3-phase inverters
 * @{
 */

/**
 * @brief   Perform a 2-phase Clarke transform to convert 3-phase currents to αβ domain
 * @details Converts phase A and B currents into orthogonal α (real) and β (imaginary) components under the assumption of a balanced system
 *          Phase C is inferred as: ( I_c = -I_a - I_b)
 * @param   ia Phase A current
 * @param   ib Phase B current
 * @param   alpha Pointer to store α-axis result
 * @param   beta Pointer to store β-axis result
 * @return  MOTOR_OK if successful
 *          MOTOR_INVALID_ARGS if output pointers are null
 */
UtilsError_t clarke_transform_2phase(float ia, float ib, float *alpha, float *beta);

/**
 * @brief   Full 3-phase Clarke transform for unbalanced or fully sensed systems
 * @details Uses all three phase currents to compute the αβ components without assuming a balanced load
 *          More accurate under asymmetrical or fault conditions
 * @param   ia Phase A current
 * @param   ib Phase B current
 * @param   ic Phase C current
 * @param   alpha Pointer to store α-axis result
 * @param   beta Pointer to store β-axis result
 * @return  MOTOR_OK if successful
 *          MOTOR_INVALID_ARGS if output pointers are null
 */
UtilsError_t clarke_transform_3phase(float ia, float ib, float ic, float *alpha, float *beta);

/**
 * @brief   Perform Park transform to convert αβ components to dq rotating frame
 * @details Transforms αβ stationary frame values into d (direct) and q (quadrature) axes based on the rotor angle θ
 *          This enables independent control of torque (q) and flux (d) in the synchronous frame
 * @param   alpha α-axis input
 * @param   beta β-axis input
 * @param   theta Rotor electrical angle (radians)
 * @param   d Pointer to store direct-axis result
 * @param   q Pointer to store quadrature-axis result
 * @return  MOTOR_OK if successful
 *          MOTOR_INVALID_ARGS if output pointers are null
 */
UtilsError_t park_transform(float alpha, float beta, float theta, float *d, float *q);

/**
 * @brief   Perform inverse Park transform to convert dq components back to αβ frame
 * @details Converts the rotating dq frame values back to stationary αβ for synthesis into phase voltages
 * @param   d Direct-axis input
 * @param   q Quadrature-axis input
 * @param   theta Rotor electrical angle (radians)
 * @param   alpha Pointer to store α-axis result
 * @param   beta Pointer to store β-axis result
 * @return  MOTOR_OK if successful
 *          MOTOR_INVALID_ARGS if output pointers are null
 */
UtilsError_t inverse_park_transform(float d, float q, float theta, float *alpha, float *beta);

/** @} */
