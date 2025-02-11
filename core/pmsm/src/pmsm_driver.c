/*******************************************************************************************************************************
 * @file   pmsm_driver.c
 *
 * @brief  Source file for the PMSM driver
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */
#include "hal.h"

/* Intra-component Headers */
#include "pmsm_driver.h"

static struct PMSMData_t s_pmsm_data = { 0 };

static MotorError_t pmsm_init(struct Motor_t *motor, struct MotorConfig_t *config) {
  motor->config = config;

  motor->private_data = &s_pmsm_data;
}

static MotorError_t pmsm_deinit(struct Motor_t *motor) {}

static MotorError_t pmsm_update_state(struct Motor_t *motor) {}

static MotorError_t pmsm_commutate(struct Motor_t *motor) {}

static MotorError_t pmsm_update_pwm(struct Motor_t *motor) {}

static MotorError_t pmsm_set_voltage(struct Motor_t *motor, float voltage) {
  motor->setpoint.voltage = voltage;
}

static MotorError_t pmsm_set_current(struct Motor_t *motor, float current) {
  motor->setpoint.current = current;
}

static MotorError_t pmsm_set_velocity(struct Motor_t *motor, float velocity) {
  motor->setpoint.velocity = velocity;
}

static MotorError_t pmsm_set_position(struct Motor_t *motor, float position) {
  motor->setpoint.position = position;
}

static MotorError_t pmsm_set_torque(struct Motor_t *motor, float torque) {
  motor->setpoint.torque = torque;
}

void pmsm_create_driver(struct Motor_t *motor) {
  motor->driver.init = pmsm_init;
  motor->driver.deinit = pmsm_deinit;
  motor->driver.update_state = pmsm_update_state;
  motor->driver.set_voltage = pmsm_set_voltage;
  motor->driver.set_current = pmsm_set_current;
  motor->driver.set_velocity = pmsm_set_velocity;
  motor->driver.set_position = pmsm_set_position;
  motor->driver.set_torque = pmsm_set_torque;
}
