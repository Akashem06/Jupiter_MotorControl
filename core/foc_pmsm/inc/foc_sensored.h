#pragma once

/*******************************************************************************************************************************
 * @file   foc_sensored.h
 *
 * @brief  Header file for the sensored BLDC driver
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "foc_common.h"
#include "foc_field_weakening.h"
#include "motor.h"

/**
 * @defgroup FOC_PMSMMotor FOC control motor class
 * @brief    PMSM FOC control motor class
 * @{
 */

/**
 * D axis - Flux enhancing vector
 */
#define FOC_PID_DEFAULT_D_KP (2.0f)
#define FOC_PID_DEFAULT_D_KI (500.0f)
#define FOC_PID_DEFAULT_D_KD (0.0f)
#define FOC_PID_DEFAULT_D_OUTPUT_MAX (24.0f)  /**< Volts */
#define FOC_PID_DEFAULT_D_OUTPUT_MIN (-24.0f) /**< Volts */
#define FOC_PID_DEFAULT_D_DERIV_EMA_ALPHA (0.1f)

/**
 * Q axis - Torque producing vector
 */
#define FOC_PID_DEFAULT_Q_KP (2.0f)
#define FOC_PID_DEFAULT_Q_KI (500.0f)
#define FOC_PID_DEFAULT_Q_KD (0.0f)
#define FOC_PID_DEFAULT_Q_OUTPUT_MAX (24.0f)  /**< Volts */
#define FOC_PID_DEFAULT_Q_OUTPUT_MIN (-24.0f) /**< Volts */
#define FOC_PID_DEFAULT_Q_DERIV_EMA_ALPHA (0.1f)

struct FOCSensoredData_t {
  float electrical_angle; /**< Electrical angle [rad] */
  float id;               /**< D-axis current [A] */
  float iq;               /**< Q-axis current [A] */
  float vd;               /**< D-axis voltage command */
  float vq;               /**< Q-axis voltage command */

  struct PidConfig_t current_d_pid_config; /**< D-axis current PID Configuration */
  struct PidConfig_t current_q_pid_config; /**< Q-axis current PID Configuration */
  struct PidController_t current_d;        /**< D-axis current PID controller */
  struct PidController_t current_q;        /**< Q-axis current PID controller */

  struct FieldWeakeningConfig_t field_weakening_config;
  struct FieldWeakeningState_t field_weakening_state;

  FOCMotorMode_t mode;
};

/**
 * @brief   Initializes and registers the sensored FOC driver functions
 *          into the provided Motor_t structure
 * @param   motor Pointer to the Motor_t structure to be populated
 */
void foc_sensored_create_driver(struct Motor_t *motor);

/** @} */
