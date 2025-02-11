/*******************************************************************************************************************************
 * @file   motor_model.c
 *
 * @brief  Source file for the simulation motor model
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stddef.h>

/* Inter-component Headers */
#include "math_utils.h"

/* Intra-component Headers */
#include "motor.h"

MotorError_t motor_run(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  MotorError_t err;

  err = motor->driver.update_state(motor);
  if (err != MOTOR_OK) {
    return err;
  }

  err = motor->driver.commutate(motor);
  if (err != MOTOR_OK) {
    return err;
  }

  err = motor->driver.update_pwm(motor);
  if (err != MOTOR_OK) {
    return err;
  }

  return MOTOR_OK;
}