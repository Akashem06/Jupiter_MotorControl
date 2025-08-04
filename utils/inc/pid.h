#pragma once

/*******************************************************************************************************************************
 * @file   pid.h
 *
 * @brief  Header file for PID control loop
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup PIDControl_Utils PID Control loop
 * @brief    PID Control loop utilities for 3-phase inverters
 * @{
 */

/**
 * @brief   PID Controller storage class
 */
struct PidConfig_t {
  float kp;                   /**< Derivative constant */
  float ki;                   /**< Integral constant */
  float kd;                   /**< Derivative constant */
  float output_min;           /**< Minimum output */
  float output_max;           /**< Maximum output */
  float derivative_ema_alpha; /**< IIR Filter alpha for low-pass filtering */
};

/**
 * @brief   PID Controller storage class
 */
struct PidController_t {
  struct PidConfig_t *config; /**< Pointer to the PID config class */
  float integral;             /**< Error integral */
  float prev_error;           /**< Previous error for derivative calculation */
  float prev_derivative;      /**< Previous derivative calculation for low-pass filter */
  bool is_initialized;        /**< Initialized flag */
};

/**
 * @brief   Initialize the PID Controller class
 * @param   pid Pointer to the PID Controller class
 * @param   config Pointer to the PID config class
 */
void pid_init(struct PidController_t *pid, struct PidConfig_t *config);

/**
 * @brief   Update the PID Controller output
 * @param   pid Pointer to the PID Controller class
 * @param   measurement Latest measurement
 * @param   delta_time Time difference between last measurement and current measurement
 * @return  Updated float output
 */
float pid_update(struct PidController_t *pid, float set_point, float measurement, float delta_time);

/** @} */
