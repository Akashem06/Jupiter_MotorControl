/*******************************************************************************************************************************
 * @file   test_main.c
 *
 * @brief  Source file for all unit tests
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */
#include "test_bldc_sensorless_driver.h"
#include "test_math_utils.h"
#include "test_pid.h"
#include "unity.h"

/* Intra-component Headers */

/* Setup before each test case */
void setUp() {
  bldc_sensorless_driver_test_set_up();
}

/* Cleanup after each test case */
void tearDown() {
  bldc_sensorless_driver_test_tear_down();
}

int main() {
  UNITY_BEGIN();
  run_pid_tests();
  run_math_utils_tests();
  run_bldc_sensorless_driver_tests();
  return UNITY_END();
}
