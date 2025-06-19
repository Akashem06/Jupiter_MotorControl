/*******************************************************************************************************************************
 * @file   hal_mock.c
 *
 * @brief  Source file for the HAL mock for unit testing
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

/* Global variables used by our HAL stubs */
static uint16_t test_pwm_duty[NUM_MOTOR_PHASES] = { 0 };
static uint8_t test_gpio_state[NUM_MOTOR_PHASES] = { 0 };

/* For our test: 0 = float, 1 = low, 2 = PWM (set via hal_pwm_set_duty) */
static float test_phase_voltages[NUM_MOTOR_PHASES] = { 0 };
static float test_phase_currents[NUM_MOTOR_PHASES] = { 0 };
static uint32_t test_micros = 0;

bool hal_pwm_init(struct PwmConfig_t *config) {
  (void)config;
  return true;
}

bool hal_adc_init(struct AdcConfig_t *config) {
  (void)config;
  return true;
}

bool hal_gpio_init() {
  return true;
}

void hal_pwm_set_duty(MotorPhase_t phase, uint16_t duty) {
  if (phase < NUM_MOTOR_PHASES) {
    test_pwm_duty[phase] = duty;
    test_gpio_state[phase] = 2U; /* 2 = PWM */
  }
}

void hal_gpio_set_phase_low(MotorPhase_t phase) {
  if (phase < NUM_MOTOR_PHASES) {
    test_gpio_state[phase] = 1U; /* 1 = LOW */
  }
}

void hal_gpio_set_phase_float(MotorPhase_t phase) {
  if (phase < NUM_MOTOR_PHASES) {
    test_gpio_state[phase] = 0U; /* 0 = float */
  }
}

void hal_adc_start_conversion() {
  /* In tests conversion is immediate */
}

void hal_adc_get_phase_voltages(float *voltages) {
  for (int i = 0; i < NUM_MOTOR_PHASES; i++) {
    voltages[i] = test_phase_voltages[i];
  }
}

void hal_adc_get_phase_currents(float *currents) {
  for (int i = 0; i < NUM_MOTOR_PHASES; i++) {
    currents[i] = test_phase_currents[i];
  }
}

/* Return a fixed temperature */
float hal_adc_get_temperature() {
  return 25.0f;
}

/* Return a fixed DC voltage */
float hal_adc_get_dc_voltage() {
  return 24.0f;
}

/* Return a simulated microsecond timestamp */
uint32_t hal_get_micros() {
  return test_micros;
}

void hal_delay_us(uint32_t delay_us) {}

void hal_delay_ms(uint32_t delay_ms) {}

bool hal_gpio_init_hall_sensors() {
  return true;
}

uint8_t hal_gpio_get_hall_state() {
  return 0;
}

/*******************************************************************************************************************************
 * HAL Mock Layer
 *******************************************************************************************************************************/

void hal_mock_reset() {
  memset(test_pwm_duty, 0, sizeof(test_pwm_duty));
  memset(test_gpio_state, 0, sizeof(test_gpio_state));
  memset(test_phase_voltages, 0, sizeof(test_phase_voltages));
  memset(test_phase_currents, 0, sizeof(test_phase_currents));
  test_micros = 1000; /* start time in microseconds */
}

void hal_mock_set_test_micros(uint32_t micros) {
  test_micros = micros;
}

uint16_t *hal_mock_get_test_pwm_duty_cycles() {
  return test_pwm_duty;
}

uint8_t *hal_mock_get_test_gpio_states() {
  return test_gpio_state;
}

float *hal_mock_get_test_phase_voltages() {
  return test_phase_voltages;
}

float *hal_mock_get_test_phase_currents() {
  return test_phase_currents;
}

void hal_mock_set_test_phase_voltage(MotorPhase_t phase, float voltage) {
  test_phase_voltages[phase] = voltage;
}

void hal_mock_set_test_phase_current(MotorPhase_t phase, float current) {
  test_phase_currents[phase] = current;
}
