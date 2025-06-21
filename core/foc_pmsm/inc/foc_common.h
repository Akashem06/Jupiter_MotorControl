#pragma once

/*******************************************************************************************************************************
 * @file   foc_common.h
 *
 * @brief  Header file for FOC common library
 *
 * @date   2025-06-18
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */
#include "motor_error.h"

/**
 * @defgroup FOC_PMSMMotor FOC control motor class
 * @brief    PMSM FOC control motor class
 * @{
 */

/*******************************************************************************************************************************
 * Private defines and enums
 *******************************************************************************************************************************/

/**
 * @brief   Motor mode definitions
 */
typedef enum {
  MOTOR_MODE_IDLE,       /**< Motor is in idle state (Just initialized) */
  MOTOR_MODE_STOPPED,    /**< Motor is stopped */
  MOTOR_MODE_ALIGNING,   /**< Initial rotor alignment phase */
  MOTOR_MODE_OPEN_LOOP,  /**< Open-loop startup sequence */
  MOTOR_MODE_TRANSITION, /**< Transition from open to closed loop */
  MOTOR_MODE_RUNNING,    /**< Normal closed-loop operation */
  MOTOR_MODE_BRAKING,    /**< Active braking */
  MOTOR_MODE_ERROR       /**< Error state */
} FOCMotorMode_t;

/*******************************************************************************************************************************
 * Variables
 *******************************************************************************************************************************/

/*******************************************************************************************************************************
 * Function definitions
 *******************************************************************************************************************************/

/**
 * @brief   Perform a 2-phase Clarke transform to convert 3-phase currents to αβ domain
 * @details Converts phase A and B currents into orthogonal α (real) and β (imaginary) components under the assumption of a balanced system
 *          Phase C is inferred as: ( I_c = -I_a - I_b)
 * @param   ia Phase A current
 * @param   ib Phase B current
 * @param   alpha Pointer to store α-axis result
 * @param   beta Pointer to store β-axis result
 * @retval  MOTOR_OK if successful
 *          MOTOR_INVALID_ARGS if output pointers are null
 */
MotorError_t clarke_transform_2phase(float ia, float ib, float *alpha, float *beta);

/**
 * @brief   Full 3-phase Clarke transform for unbalanced or fully sensed systems
 * @details Uses all three phase currents to compute the αβ components without assuming a balanced load
 *          More accurate under asymmetrical or fault conditions
 * @param   ia Phase A current
 * @param   ib Phase B current
 * @param   ic Phase C current
 * @param   alpha Pointer to store α-axis result
 * @param   beta Pointer to store β-axis result
 * @retval  MOTOR_OK if successful
 *          MOTOR_INVALID_ARGS if output pointers are null
 */
MotorError_t clarke_transform_3phase(float ia, float ib, float ic, float *alpha, float *beta);

/**
 * @brief   Perform Park transform to convert αβ components to dq rotating frame
 * @details Transforms αβ stationary frame values into d (direct) and q (quadrature) axes based on the rotor angle θ
 *          This enables independent control of torque (q) and flux (d) in the synchronous frame
 * @param   alpha α-axis input
 * @param   beta β-axis input
 * @param   theta Rotor electrical angle (radians)
 * @param   d Pointer to store direct-axis result
 * @param   q Pointer to store quadrature-axis result
 * @retval  MOTOR_OK if successful
 *          MOTOR_INVALID_ARGS if output pointers are null
 */
MotorError_t park_transform(float alpha, float beta, float theta, float *d, float *q);

/**
 * @brief   Perform inverse Park transform to convert dq components back to αβ frame
 * @details Converts the rotating dq frame values back to stationary αβ for synthesis into phase voltages
 * @param   d Direct-axis input
 * @param   q Quadrature-axis input
 * @param   theta Rotor electrical angle (radians)
 * @param   alpha Pointer to store α-axis result
 * @param   beta Pointer to store β-axis result
 * @retval  MOTOR_OK if successful
 *          MOTOR_INVALID_ARGS if output pointers are null
 */
MotorError_t inverse_park_transform(float d, float q, float theta, float *alpha, float *beta);

/**
 * @brief Generate SVPWM duty cycles using angle-based sector detection
 * @param theta_e Electrical angle (radians, normalized 0 to 2π)
 * @param vref_mag Normalized voltage magnitude (0.0 to 1.0, modulation index)
 * @param duty_A Pointer to store phase A duty cycle (0.0 to 1.0)
 * @param duty_B Pointer to store phase B duty cycle (0.0 to 1.0)
 * @param duty_C Pointer to store phase C duty cycle (0.0 to 1.0)
 * @retval MOTOR_OK if successful, MOTOR_INVALID_ARGS if null pointers
 */
MotorError_t svpwm_generate(float theta_e, float vref_mag, float *duty_A, float *duty_B, float *duty_C);

/** @} */
