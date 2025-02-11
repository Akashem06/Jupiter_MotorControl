/*******************************************************************************************************************************
 * @file   test_math_utils.c
 *
 * @brief  Source file for math unit tests
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */
#include "math_utils.h"
#include "unity.h"

/* Intra-component Headers */

void test_clamp_btwn() {
  float test_input = 7.0f;
  float test_max_output = 9.0f;
  float test_min_output = 1.0f;

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 7.0f, clamp(test_input, test_min_output, test_max_output));
}

void test_clamp_gtmax() {
  float test_input = 10.0f;
  float test_max_output = 9.0f;
  float test_min_output = 1.0f;

  TEST_ASSERT_FLOAT_WITHIN(0.1f, test_max_output,
                           clamp(test_input, test_min_output, test_max_output));
}

void test_clamp_lsmin() {
  float test_input = 0.5f;
  float test_max_output = 9.0f;
  float test_min_output = 1.0f;

  TEST_ASSERT_FLOAT_WITHIN(0.1f, test_min_output,
                           clamp(test_input, test_min_output, test_max_output));
}

void run_math_utils_tests() {
  RUN_TEST(test_clamp_btwn);
  RUN_TEST(test_clamp_gtmax);
  RUN_TEST(test_clamp_lsmin);
}
