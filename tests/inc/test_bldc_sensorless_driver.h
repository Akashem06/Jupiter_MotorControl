#pragma once

/*******************************************************************************************************************************
 * @file   test_bldc_sensorless_driver.h
 *
 * @brief  Header file for BLDC driver tests
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup TestHeaders Test files
 * @brief    Test headers for 3-phase inverters
 * @{
 */

/**
 * @brief   Run BLDC sensorless driver tests
 */
void run_bldc_sensorless_driver_tests();

void bldc_sensorless_driver_test_set_up();

void bldc_sensorless_driver_test_tear_down();

/** @} */
