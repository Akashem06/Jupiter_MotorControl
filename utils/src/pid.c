/*******************************************************************************************************************************
 * @file   pid.c
 *
 * @brief  Source file for PID control loop
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stddef.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "math_utils.h"
#include "pid.h"

void pid_init(struct PidController_t *pid, struct PidConfig_t *config) {
  if (pid == NULL || config == NULL) {
    return;
  }

  pid->config = config;

  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->prev_derivative = 0.0f;
  pid->is_initialized = true;
}

float pid_update(struct PidController_t *pid, float set_point, float measurement, float delta_time) {
  if (pid == NULL || pid->is_initialized == false) {
    return 0.0f;
  }

  float error = set_point - measurement;

  /* Trapezoidal rule integral */
  pid->integral = pid->integral + (0.5f * delta_time) * (error + pid->prev_error);

  /* IIR Low pass filter */
  float derivative = 0.0f;

  /* Only calculate if time is not 0 and previous error is a valid value */
  if (delta_time > 0.0f && pid->prev_error != 0.0f) {
    derivative = (error - pid->prev_error) / delta_time;
    derivative = (pid->config->derivative_ema_alpha * derivative) + ((1.0f - pid->config->derivative_ema_alpha) * pid->prev_derivative);
    pid->prev_derivative = derivative;
  }

  pid->prev_error = error;

  float output = (pid->config->kp * error) + (pid->config->ki * pid->integral) + (pid->config->kd * derivative);

  /* Integral windup */
  if (output > pid->config->output_max) {
    /* Saturation has occurred, prevent further integration */
    pid->integral -= (output - pid->config->output_max) / pid->config->ki;
    output = pid->config->output_max;
  } else if (output < pid->config->output_min) {
    /* Saturation has occurred, prevent further integration */
    pid->integral -= (pid->config->output_min - output) / pid->config->ki;
    output = pid->config->output_min;
  }

  return output;
}
