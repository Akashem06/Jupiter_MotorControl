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
#include "test_bldc_driver.h"
#include "test_bldc_model.h"
#include "test_math_utils.h"
#include "test_pid.h"
#include "unity.h"

/* Intra-component Headers */

/* Setup before each test case */
void setUp() {
  bldc_driver_test_set_up();
}

/* Cleanup after each test case */
void tearDown() {
  bldc_driver_test_tear_down();
}

int main() {
  UNITY_BEGIN();
  run_pid_tests();
  run_math_utils_tests();
  run_bldc_driver_tests();
  run_bldc_model_tests();
  return UNITY_END();
}
