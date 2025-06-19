/*******************************************************************************************************************************
 * @file   test_pid.c
 *
 * @brief  Source file for PID tests
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */
#include "pid.h"
#include "unity.h"

/* Intra-component Headers */

void test_pid_init() {
  struct PidConfig_t config = { .kp = 1.0f, .ki = 1.0f, .kd = 1.0f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 0.1f };

  struct PidController_t pid;

  pid_init(&pid, &config);

  TEST_ASSERT_TRUE(pid.is_initialized);
  TEST_ASSERT_EQUAL(0.0f, pid.integral);
  TEST_ASSERT_EQUAL(0.0f, pid.prev_error);
  TEST_ASSERT_EQUAL_PTR(&config, pid.config);
}

void test_pid_invalid_config() {
  struct PidController_t pid;
  pid_init(&pid, NULL);

  float output = pid_update(&pid, 10.0f, 5.0f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, output);
}

void test_pid_update_no_error() {
  struct PidConfig_t config = { .kp = 1.0f, .ki = 1.0f, .kd = 1.0f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 0.1f };

  struct PidController_t pid;

  pid_init(&pid, &config);

  float set_point = 10.0;
  float measurement = 10.0;
  float delta_time = 1.0;

  float output = pid_update(&pid, set_point, measurement, delta_time);

  TEST_ASSERT_EQUAL_FLOAT(0.0f, output);
}

void test_pid_kp_update_positive_error() {
  struct PidConfig_t config = { .kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 0.1f };

  struct PidController_t pid;

  pid_init(&pid, &config);

  float set_point = 10.0f;
  float measurement = 5.0f;
  float delta_time = 1.0f;

  float output = pid_update(&pid, set_point, measurement, delta_time);

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 5.0f, output);
}

void test_pid_kp_update_negative_error() {
  struct PidConfig_t config = { .kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 0.1f };

  struct PidController_t pid;

  pid_init(&pid, &config);

  float set_point = 10.0f;
  float measurement = 15.0f;
  float delta_time = 1.0f;

  float output = pid_update(&pid, set_point, measurement, delta_time);

  TEST_ASSERT_FLOAT_WITHIN(0.1f, -5.0f, output);
}

void test_pid_ki_update_positive_error() {
  struct PidConfig_t config = { .kp = 0.0f, .ki = 1.0f, .kd = 0.0f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 0.1f };

  struct PidController_t pid;

  pid_init(&pid, &config);

  float set_point = 10.0f;
  float measurement = 5.0f;
  float delta_time = 1.0f;

  float output = pid_update(&pid, set_point, measurement, delta_time);

  /* 2.5 because of trapezoidal rule for integrals */
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.5f, output);
}

void test_pid_ki_update_negative_error() {
  struct PidConfig_t config = { .kp = 0.0f, .ki = 1.0f, .kd = 0.0f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 0.1f };

  struct PidController_t pid;

  pid_init(&pid, &config);

  float set_point = 10.0f;
  float measurement = 15.0f;
  float delta_time = 1.0f;

  float output = pid_update(&pid, set_point, measurement, delta_time);

  /* 2.5 because of trapezoidal rule for integrals */
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -2.5f, output);
}

void test_pid_kd_update_positive_error() {
  struct PidConfig_t config = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .output_max = 100.0f,
    .output_min = -100.0f,
    .derivative_ema_alpha = 1.0f /* 1 for simpler testing. Disabling EMA */
  };

  struct PidController_t pid;
  pid_init(&pid, &config);

  float set_point = 10.0f;
  float measurement = 5.0f;
  float delta_time = 0.1f;

  float output = pid_update(&pid, set_point, measurement, delta_time);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, output); /* First derivative should be 0 since we don't have a previous error */

  measurement = 4.0f;
  output = pid_update(&pid, set_point, measurement, delta_time);

  /* Derivative should be (error - prev_error) / delta_time = (6 - 5) / 0.1 = 10 */
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, output);
}

void test_pid_kd_update_negative_error() {
  struct PidConfig_t config = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .output_max = 100.0f,
    .output_min = -100.0f,
    .derivative_ema_alpha = 1.0f /* 1 for simpler testing. Disabling EMA */
  };

  struct PidController_t pid;
  pid_init(&pid, &config);

  float set_point = 10.0f;
  float measurement = 15.0f;
  float delta_time = 0.1f;

  float output = pid_update(&pid, set_point, measurement, delta_time);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, output); /* First derivative should be 0 since we don't have a previous error */

  measurement = 16.0f;
  output = pid_update(&pid, set_point, measurement, delta_time);

  /* Derivative should be (error - prev_error) / delta_time = (-6 - (-5)) / 0.1 = -10 */
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -10.0f, output);
}

void test_pid_kd_with_ema_filter() {
  struct PidConfig_t config = { .kp = 0.0f, .ki = 0.0f, .kd = 1.0f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 0.5f };

  struct PidController_t pid;
  pid_init(&pid, &config);

  float set_point = 10.0f;
  float measurement = 5.0f;
  float delta_time = 0.1f;

  float output = pid_update(&pid, set_point, measurement, delta_time);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, output); /* First derivative should be 0 since we don't have a previous error */

  measurement = 4.0f;
  output = pid_update(&pid, set_point, measurement, delta_time);

  /* Raw derivative would be 10, but EMA filter with alpha=0.5 makes it 5 */
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 5.0f, output);

  measurement = 3.0f;
  output = pid_update(&pid, set_point, measurement, delta_time);
  /* Raw derivative still 10, combined with previous filtered value gives 7.5 */
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 7.5f, output);
}

void test_pid_integral_windup_positive() {
  struct PidConfig_t config = { .kp = 0.0f, .ki = 1.0f, .kd = 0.0f, .output_max = 10.0f, .output_min = -10.0f, .derivative_ema_alpha = 1.0f };

  struct PidController_t pid;
  pid_init(&pid, &config);

  /* Large error to trigger windup */
  float set_point = 20.0f;
  float measurement = 0.0f;
  float delta_time = 1.0f;

  /* Run multiple times to build up the integral */
  for (int i = 0; i < 5; i++) {
    float output = pid_update(&pid, set_point, measurement, delta_time);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, output); /* Should remain at max */
  }
}

void test_pid_integral_windup_negative() {
  struct PidConfig_t config = { .kp = 0.0f, .ki = 1.0f, .kd = 0.0f, .output_max = 10.0f, .output_min = -10.0f, .derivative_ema_alpha = 1.0f };

  struct PidController_t pid;
  pid_init(&pid, &config);

  /* Large error to trigger windup */
  float set_point = -20.0f;
  float measurement = 0.0f;
  float delta_time = 1.0f;

  /* Run multiple times to build up the integral */
  for (int i = 0; i < 5; i++) {
    float output = pid_update(&pid, set_point, measurement, delta_time);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -10.0f, output); /* Should remain at min */
  }
}

void test_pid_zero_delta_time() {
  struct PidConfig_t config = { .kp = 1.0f, .ki = 1.0f, .kd = 1.0f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 1.0f };

  struct PidController_t pid;
  pid_init(&pid, &config);

  float set_point = 10.0f;
  float measurement = 5.0f;
  float delta_time = 0.0f;

  float output = pid_update(&pid, set_point, measurement, delta_time);

  /* Should only include proportional term since dt=0 */
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 5.0f, output);
}

void test_pid_large_delta_time() {
  struct PidConfig_t config = { .kp = 1.0f, .ki = 1.0f, .kd = 1.0f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 1.0f };

  struct PidController_t pid;
  pid_init(&pid, &config);

  float set_point = 10.0f;
  float measurement = 5.0f;
  float delta_time = 100.0f;

  float output = pid_update(&pid, set_point, measurement, delta_time);

  /* Should handle large dt gracefully without exploding */
  TEST_ASSERT_TRUE(output <= pid.config->output_max);
  TEST_ASSERT_TRUE(output >= pid.config->output_min);
}

void test_pid_changing_setpoint() {
  struct PidConfig_t config = { .kp = 1.0f, .ki = 0.1f, .kd = 0.1f, .output_max = 100.0f, .output_min = -100.0f, .derivative_ema_alpha = 1.0f };

  struct PidController_t pid;
  pid_init(&pid, &config);

  float measurement = 0.0f;
  float delta_time = 0.1f;

  /* First setpoint */
  float output = pid_update(&pid, 10.0f, measurement, delta_time);
  float first_output = output;

  /* Change setpoint */
  output = pid_update(&pid, 20.0f, measurement, delta_time);

  /* Output should be larger due to larger error */
  TEST_ASSERT_TRUE(output > first_output);
}

void run_pid_tests() {
  RUN_TEST(test_pid_init);
  RUN_TEST(test_pid_invalid_config);
  RUN_TEST(test_pid_update_no_error);
  RUN_TEST(test_pid_kp_update_positive_error);
  RUN_TEST(test_pid_kp_update_negative_error);
  RUN_TEST(test_pid_ki_update_positive_error);
  RUN_TEST(test_pid_ki_update_negative_error);
  RUN_TEST(test_pid_kd_update_positive_error);
  RUN_TEST(test_pid_kd_update_negative_error);
  RUN_TEST(test_pid_kd_with_ema_filter);
  RUN_TEST(test_pid_integral_windup_positive);
  RUN_TEST(test_pid_integral_windup_negative);
  RUN_TEST(test_pid_zero_delta_time);
  RUN_TEST(test_pid_large_delta_time);
  RUN_TEST(test_pid_changing_setpoint);
}
