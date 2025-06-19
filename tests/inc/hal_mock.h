/*******************************************************************************************************************************
 * @file   hal_mock.h
 *
 * @brief  Header file for the HAL mock for unit testing
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Inter-component Headers */
#include "hal.h"

/* Intra-component Headers */

/**
 * @defgroup TestHeaders Test files
 * @brief    Test headers for Motor control
 * @{
 */

void hal_mock_reset();

void hal_mock_set_test_micros(uint32_t micros);

uint16_t *hal_mock_get_test_pwm_duty_cycles();

uint8_t *hal_mock_get_test_gpio_states();

float *hal_mock_get_test_phase_voltages();

float *hal_mock_get_test_phase_currents();

void hal_mock_set_test_phase_voltage(MotorPhase_t phase, float voltage);

void hal_mock_set_test_phase_current(MotorPhase_t phase, float current);

/** @} */
