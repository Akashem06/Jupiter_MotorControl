/*******************************************************************************************************************************
 * @file   test_bldc_driver.c
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
#include "bldc_model.h"
#include "math_utils.h"
#include "motor_model.h"
#include "unity.h"

/* Intra-component Headers */
#include "test_bldc_model.h"

#define NUM_COMMUTATION_STEPS 6U

void bldc_model_test_set_up() {}

void bldc_model_test_tear_down() {}

/* Helper: Prepare a valid motor configuration */
static void prepare_valid_config(struct MotorModelConfig_t *config) {
  memset(config, 0, sizeof(*config));
  config->type = MOTOR_TYPE_BLDC;
  config->pole_pairs = 1;
  config->phase_resistance = 0.1f;
  config->phase_inductance = 0.0001f;
  config->friction_coefficient = 0.0f; /* Test with 0 friction */
  config->moment_of_inertia = 1.0f;    /* Test with unit moment of inertia */
  config->max_current = 20.0f;
  config->max_voltage = 24.0f;

  config->params.ac.back_emf_constant = 1.0f;
  config->params.ac.torque_constant = 1.0f;
}

void test_bldc_model_init_success() {
  struct MotorModel_t motor_model;
  struct MotorModelConfig_t config;
  prepare_valid_config(&config);

  motor_model_init(&motor_model, &config);

  TEST_ASSERT_EQUAL_PTR(motor_model.config, &config);
}

void test_bldc_model_init_null_config() {
  struct MotorModel_t motor_model;

  motor_model_init(&motor_model, NULL);

  TEST_ASSERT_NOT_EQUAL(motor_model.config, NULL);
}

void test_bldc_model_calculate_back_emf_elec_angle_0() {
  struct MotorModel_t motor_model;
  struct MotorModelConfig_t config;
  prepare_valid_config(&config);

  motor_model_init(&motor_model, &config);

  TEST_ASSERT_EQUAL_PTR(motor_model.config, &config);

  motor_model.state.electrical_angle = 0.0f; /* Let electrical angle be 0 */
  motor_model.state.rotor_speed = 5.0f;
  motor_model.config->params.ac.back_emf_constant = 1.0f; /* Let back emf constant be 1 */

  motor_model.calculate_back_emf(&motor_model);

  /* Back EMF will be positive speed * Ke */
  TEST_ASSERT_EQUAL(5.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_A]);

  /* Back EMF will be negative speed * Ke */
  TEST_ASSERT_EQUAL(-5.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_B]);

  /* Back EMF will be posistive speed * Ke */
  TEST_ASSERT_EQUAL(5.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_C]);
}

void test_bldc_model_calculate_back_emf_elec_angle_90() {
  struct MotorModel_t motor_model;
  struct MotorModelConfig_t config;
  prepare_valid_config(&config);

  motor_model_init(&motor_model, &config);

  TEST_ASSERT_EQUAL_PTR(motor_model.config, &config);

  motor_model.state.electrical_angle = MATH_PI / 2.0f; /* Let electrical angle be 180 degrees */
  motor_model.state.rotor_speed = 50.0f;
  motor_model.config->params.ac.back_emf_constant = 0.1f; /* Let back emf constant be 1 */

  motor_model.calculate_back_emf(&motor_model);

  /* Back EMF will be positive speed * Ke. Angle = PI/2 */
  TEST_ASSERT_EQUAL(5.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_A]);

  /* Back EMF will be negative speed * Ke. Angle = PI/2 + 2PI/3 = 7PI/6 */
  TEST_ASSERT_EQUAL(-5.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_B]);

  /* Back EMF will be posistive speed * Ke. Angle = PI/2 + 4PI/3 = 11PI/6 */
  TEST_ASSERT_EQUAL(5.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_C]);
}

void test_bldc_model_calculate_back_emf_elec_angle_180() {
  struct MotorModel_t motor_model;
  struct MotorModelConfig_t config;
  prepare_valid_config(&config);

  motor_model_init(&motor_model, &config);

  TEST_ASSERT_EQUAL_PTR(motor_model.config, &config);

  motor_model.state.electrical_angle = MATH_PI; /* Let electrical angle be 180 degrees */
  motor_model.state.rotor_speed = 10.0f;
  motor_model.config->params.ac.back_emf_constant = 1.0f; /* Let back emf constant be 1 */

  motor_model.calculate_back_emf(&motor_model);

  /* Back EMF will be negative speed * Ke */
  TEST_ASSERT_EQUAL(-10.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_A]);

  /* Back EMF will be posistive speed * Ke */
  TEST_ASSERT_EQUAL(10.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_B]);

  /* Back EMF will be posistive speed * Ke */
  TEST_ASSERT_EQUAL(10.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_C]);
}

void test_bldc_model_calculate_back_emf_elec_angle_270() {
  struct MotorModel_t motor_model;
  struct MotorModelConfig_t config;
  prepare_valid_config(&config);

  motor_model_init(&motor_model, &config);

  TEST_ASSERT_EQUAL_PTR(motor_model.config, &config);

  motor_model.state.electrical_angle =
      3.0f * MATH_PI / 2.0f; /* Let electrical angle be 270 degrees */
  motor_model.state.rotor_speed = 0.1f;
  motor_model.config->params.ac.back_emf_constant = 10.0f; /* Let back emf constant be 1 */

  motor_model.calculate_back_emf(&motor_model);

  /* Back EMF will be negative speed * Ke */
  TEST_ASSERT_EQUAL(1.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_A]);

  /* Back EMF will be posistive speed * Ke */
  TEST_ASSERT_EQUAL(1.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_B]);

  /* Back EMF will be negative speed * Ke */
  TEST_ASSERT_EQUAL(-1.0f, motor_model.state.traits.ac.back_emf[MOTOR_PHASE_C]);
}

void run_bldc_model_tests() {
  RUN_TEST(test_bldc_model_init_success);
  RUN_TEST(test_bldc_model_init_null_config);
  RUN_TEST(test_bldc_model_calculate_back_emf_elec_angle_0);
  RUN_TEST(test_bldc_model_calculate_back_emf_elec_angle_90);
  RUN_TEST(test_bldc_model_calculate_back_emf_elec_angle_180);
  RUN_TEST(test_bldc_model_calculate_back_emf_elec_angle_270);
}
