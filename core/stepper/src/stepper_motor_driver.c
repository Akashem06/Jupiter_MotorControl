/*******************************************************************************************************************************
 * @file   stepper_motor_driver.c
 *
 * @brief  Source file for the Stepper motor driver
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */
#include "hal.h"

/* Intra-component Headers */
#include "stepper_motor_driver.h"

static struct StepperMotorData_t s_stepper_motor_data = { 0 };

static MotorError_t stepper_motor_init(struct Motor_t *motor, struct MotorConfig_t *config) {
  motor->config = config;

  motor->private_data = &s_stepper_motor_data;
}

static MotorError_t stepper_motor_deinit(struct Motor_t *motor) {}

static MotorError_t stepper_motor_update_state(struct Motor_t *motor) {}

static MotorError_t stepper_motor_commutate(struct Motor_t *motor) {}

static MotorError_t stepper_motor_update_pwm(struct Motor_t *motor) {}

static MotorError_t stepper_motor_set_voltage(struct Motor_t *motor, float voltage) {
  motor->setpoint.voltage = voltage;
}

static MotorError_t stepper_motor_set_current(struct Motor_t *motor, float current) {
  motor->setpoint.current = current;
}

static MotorError_t stepper_motor_set_velocity(struct Motor_t *motor, float velocity) {
  motor->setpoint.velocity = velocity;
}

static MotorError_t stepper_motor_set_position(struct Motor_t *motor, float position) {
  motor->setpoint.position = position;
}

static MotorError_t stepper_motor_set_torque(struct Motor_t *motor, float torque) {
  motor->setpoint.torque = torque;
}

void stepper_motor_create_driver(struct Motor_t *motor) {
  motor->driver.init = stepper_motor_init;
  motor->driver.deinit = stepper_motor_deinit;
  motor->driver.update_state = stepper_motor_update_state;
  motor->driver.set_voltage = stepper_motor_set_voltage;
  motor->driver.set_current = stepper_motor_set_current;
  motor->driver.set_velocity = stepper_motor_set_velocity;
  motor->driver.set_position = stepper_motor_set_position;
  motor->driver.set_torque = stepper_motor_set_torque;
}
