/*******************************************************************************************************************************
 * @file   test_bldc_sensorless_driver_driver.c
 *
 * @brief  Source file for the BLDC driver
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Inter-component Headers */
#include "bldc_6step_sensorless.h"
#include "hal.h"
#include "motor.h"
#include "pid.h"
#include "unity.h"

/* Intra-component Headers */
#include "hal_mock.h"
#include "test_bldc_sensorless_driver.h"

#define NUM_COMMUTATION_STEPS 6U

static uint8_t determine_floating_phase(uint8_t step) {
  switch (step) {
    case 0U:
      return MOTOR_PHASE_C;
    case 1U:
      return MOTOR_PHASE_C;
    case 2U:
      return MOTOR_PHASE_B;
    case 3U:
      return MOTOR_PHASE_A;
    case 4U:
      return MOTOR_PHASE_B;
    case 5U:
      return MOTOR_PHASE_A;
    default:
      return MOTOR_PHASE_C;
  }
}

void bldc_sensorless_driver_test_set_up() {}

void bldc_sensorless_driver_test_tear_down() {}

/* Helper: Prepare a valid motor configuration */
static void prepare_valid_config(struct MotorConfig_t *config) {
  memset(config, 0, sizeof(*config));
  config->type = MOTOR_TYPE_BLDC;
  config->control_method = CONTROL_METHOD_SENSORLESS;
  config->pole_pairs = 1;
  config->phase_resistance = 0.1f;
  config->phase_inductance = 0.0001f;
  config->max_current = 20.0f;
  config->max_voltage = 24.0f;
  config->max_velocity = 1000.0f;

  config->pwm_config.frequency = 20000;
  config->pwm_config.dead_time_ns = 1000;
  config->pwm_config.resolution = 12;
  config->pwm_config.complementary_output = true;

  config->adc_config.sampling_freq = 20000;
  config->adc_config.resolution = 12;
  config->adc_config.v_ref = 3.3f;
  config->adc_config.current_gain = 0.1f;
  config->adc_config.voltage_gain = 0.1f;
}

void test_bldc_sensorless_driver_init_success() {
  struct Motor_t motor;
  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);

  hal_mock_set_test_micros(1000);
  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);
  TEST_ASSERT_TRUE(motor.state.is_initialized);
  TEST_ASSERT_NOT_NULL(motor.private_data);
}

void test_bldc_sensorless_driver_init_null_config() {
  struct Motor_t motor;
  bldc_6step_sensorless_create_driver(&motor);

  MotorError_t err = motor.driver.init(&motor, NULL);
  TEST_ASSERT_EQUAL(MOTOR_INVALID_ARGS, err);
}

void test_bldc_sensorless_driver_deinit() {
  struct Motor_t motor;
  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);

  hal_mock_set_test_micros(1000);
  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);
  err = motor.driver.deinit(&motor);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);
  TEST_ASSERT_FALSE(motor.state.is_initialized);
}

void test_bldc_sensorless_driver_update_state_normal() {
  struct Motor_t motor;
  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);

  config.control_method = CONTROL_MODE_CURRENT;

  hal_mock_set_test_micros(1000);
  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  for (int i = 0; i < NUM_MOTOR_PHASES; i++) {
    hal_mock_set_test_phase_voltage(i, 12.0f); /* below max_voltage */
    hal_mock_set_test_phase_current(i, 5.0f);  /* below max_current */
  }

  motor.control.current = (struct PidController_t){ 0 };
  motor.setpoint.current = 10.0f;
  motor.state.last_update_time = 1000;

  hal_mock_set_test_micros(2000); /* simulate 1ms later */

  err = motor.driver.update_state(&motor);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);
}

void test_bldc_sensorless_driver_update_state_overvoltage() {
  struct Motor_t motor;
  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);
  config.control_method = CONTROL_MODE_CURRENT;

  hal_mock_set_test_micros(1000);
  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  struct BLDC6StepSensorlessData_t *bldc = (struct BLDC6StepSensorlessData_t *)motor.private_data;
  bldc->mode = MOTOR_MODE_RUNNING;

  /* Set one phase voltage above max */
  hal_mock_set_test_phase_voltage(MOTOR_PHASE_A, 30.0f);

  for (int i = 1; i < NUM_MOTOR_PHASES; i++) {
    hal_mock_set_test_phase_voltage(i, 12.0f);
  }

  for (int i = 0; i < NUM_MOTOR_PHASES; i++) {
    hal_mock_set_test_phase_current(i, 5.0f);
  }

  hal_mock_set_test_micros(2000);

  err = motor.driver.update_state(&motor);
  TEST_ASSERT_EQUAL(MOTOR_OVERVOLTAGE_ERROR, err);
}

void test_bldc_sensorless_driver_update_state_overcurrent() {
  struct Motor_t motor;
  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);
  config.control_method = CONTROL_MODE_CURRENT;

  hal_mock_set_test_micros(1000);
  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  struct BLDC6StepSensorlessData_t *bldc = (struct BLDC6StepSensorlessData_t *)motor.private_data;
  bldc->mode = MOTOR_MODE_RUNNING;

  for (int i = 0; i < NUM_MOTOR_PHASES; i++) {
    hal_mock_set_test_phase_voltage(i, 12.0f);
  }

  /* Set one phase current above max */
  hal_mock_set_test_phase_current(MOTOR_PHASE_C, 25.0f);

  for (int i = 0; i < NUM_MOTOR_PHASES; i++) {
    if (i != 2) {
      hal_mock_set_test_phase_current(i, 5.0f);
    }
  }

  hal_mock_set_test_micros(2000);

  err = motor.driver.update_state(&motor);
  TEST_ASSERT_EQUAL(MOTOR_OVERCURRENT_ERROR, err);
}

void test_bldc_sensorless_driver_commutate_sensorless() {
  struct Motor_t motor;
  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);
  config.control_method = CONTROL_METHOD_SENSORLESS;

  hal_mock_set_test_micros(0);

  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  hal_mock_set_test_micros(100);
  /* For sensorless commutation, the driver checks the filtered back-EMF in the floating phase.
     Set that value to be above the threshold so that a commutation occurs.
     (determine_floating_phase() is used internally to pick the phase.) */
  struct BLDC6StepSensorlessData_t *bldc = (struct BLDC6StepSensorlessData_t *)motor.private_data;

  bldc->mode = MOTOR_MODE_RUNNING;

  uint8_t floating_phase = determine_floating_phase(bldc->step);
  bldc->bemf_filtered[floating_phase] = bldc->zc_threshold + 0.51f; /* Hystersis = 0.5 */
  bldc->direction = true;
  uint8_t old_step = bldc->step;
  err = motor.driver.commutate(&motor);

  TEST_ASSERT_EQUAL(MOTOR_OK, err);
  TEST_ASSERT_EQUAL((old_step + 1U) % NUM_COMMUTATION_STEPS, bldc->step);
}

void test_bldc_sensorless_driver_update_pwm() {
  hal_mock_reset();

  struct Motor_t motor;
  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);
  config.control_method = CONTROL_METHOD_SENSORLESS;

  hal_mock_set_test_micros(1000);
  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  /* Set a known commutation step and PWM duty */
  struct BLDC6StepSensorlessData_t *bldc = (struct BLDC6StepSensorlessData_t *)motor.private_data;
  bldc->step = 0; /* using commutation table row 0: {1,0,0,1,0,0} */
  bldc->pwm_duty = 1000;

  err = motor.driver.update_pwm(&motor);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);
  /* According to the commutation table row 0:
       - MOTOR_PHASE_A: high → PWM should be set to 1000
       - MOTOR_PHASE_B: low  → GPIO low (state == 1)
       - MOTOR_PHASE_C: float → GPIO float (state == 0)
  */
  TEST_ASSERT_EQUAL(1000U, hal_mock_get_test_pwm_duty_cycles()[MOTOR_PHASE_A]);
  TEST_ASSERT_EQUAL(1U, hal_mock_get_test_gpio_states()[MOTOR_PHASE_B]);
  TEST_ASSERT_EQUAL(0U, hal_mock_get_test_gpio_states()[MOTOR_PHASE_C]);
}

/* Test: bldc_set_voltage clamps the setpoint appropriately */
void test_bldc_sensorless_driver_set_voltage() {
  struct Motor_t motor;
  memset(&motor, 0, sizeof(motor));

  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);

  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  motor.driver.set_voltage(&motor, 12.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, motor.setpoint.voltage);

  motor.driver.set_voltage(&motor, 30.0f); /* above max_voltage */
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 24.0f, motor.setpoint.voltage);
}

void test_bldc_sensorless_driver_set_current() {
  struct Motor_t motor;
  memset(&motor, 0, sizeof(motor));

  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);

  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  motor.driver.set_current(&motor, 10.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, motor.setpoint.current);

  motor.driver.set_current(&motor, 25.0f); /* above max_current */
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, motor.setpoint.current);
}

void test_bldc_sensorless_driver_set_velocity() {
  struct Motor_t motor;
  memset(&motor, 0, sizeof(motor));

  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);

  config.max_velocity = 1000.0f;
  config.max_current = 20.0f;

  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  /* Within limits */
  motor.driver.set_velocity(&motor, 500.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 500.0f, motor.setpoint.velocity);

  /* Above maximum */
  motor.driver.set_velocity(&motor, 1500.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, motor.setpoint.velocity);
}

void test_bldc_sensorless_driver_set_position() {
  struct Motor_t motor;
  memset(&motor, 0, sizeof(motor));

  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);

  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  motor.driver.set_position(&motor, 1.57f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.57f, motor.setpoint.position);
}

void test_bldc_sensorless_driver_set_torque() {
  struct Motor_t motor;
  memset(&motor, 0, sizeof(motor));

  bldc_6step_sensorless_create_driver(&motor);

  struct MotorConfig_t config;
  prepare_valid_config(&config);

  MotorError_t err = motor.driver.init(&motor, &config);
  TEST_ASSERT_EQUAL(MOTOR_OK, err);

  motor.driver.set_torque(&motor, 5.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, motor.setpoint.torque);
}

void run_bldc_sensorless_driver_tests() {
  RUN_TEST(test_bldc_sensorless_driver_init_success);
  RUN_TEST(test_bldc_sensorless_driver_init_null_config);
  RUN_TEST(test_bldc_sensorless_driver_deinit);
  RUN_TEST(test_bldc_sensorless_driver_update_state_normal);
  RUN_TEST(test_bldc_sensorless_driver_update_state_overvoltage);
  RUN_TEST(test_bldc_sensorless_driver_update_state_overcurrent);
  RUN_TEST(test_bldc_sensorless_driver_commutate_sensorless);
  RUN_TEST(test_bldc_sensorless_driver_update_pwm);
  RUN_TEST(test_bldc_sensorless_driver_set_voltage);
  RUN_TEST(test_bldc_sensorless_driver_set_current);
  RUN_TEST(test_bldc_sensorless_driver_set_velocity);
  RUN_TEST(test_bldc_sensorless_driver_set_position);
  RUN_TEST(test_bldc_sensorless_driver_set_torque);
}
