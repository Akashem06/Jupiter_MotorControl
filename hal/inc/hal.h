#pragma once

/*******************************************************************************************************************************
 * @file   hal.h
 *
 * @brief  Header file for the Hardware Abstraction Layer
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup HAL Hardware abstraction layer
 * @brief    Hardware abstraction layer for the motor controller
 * @{
 */

/**
 * @brief   Motor phases
 */
typedef enum {
  MOTOR_PHASE_A, /**< Motor phase A */
  MOTOR_PHASE_B, /**< Motor phase B */
  MOTOR_PHASE_C, /**< Motor phase C */
  NUM_MOTOR_PHASES
} MotorPhase_t;

/**
 * @brief   PWM configuration structure
 */
struct PwmConfig_t {
  uint32_t frequency;        /**< PWM frequency in Hz */
  uint32_t dead_time_ns;     /**< Dead time in nanoseconds */
  uint16_t resolution;       /**< PWM resolution in bits */
  bool complementary_output; /**< Enable complementary output mode */
};

/**
 * @brief   ADC configuration structure
 */
struct AdcConfig_t {
  uint32_t sampling_freq; /**< ADC sampling frequency in Hz */
  uint16_t resolution;    /**< ADC resolution in bits */
  float v_ref;            /**< ADC reference voltage */
  float current_gain;     /**< Current sensor gain (V/A) */
  float voltage_gain;     /**< Voltage sensor gain (V/V) */
};

/**
 * @brief   Initialize the PWM interface
 * @param   config Pointer to the PWM config
 * @return  TRUE if initialization succeeds
 *          FALSE if initialization fails
 */
bool hal_pwm_init(struct PwmConfig_t *config);

/**
 * @brief   Initialize the ADC interface
 * @param   config Pointer to the ADC config
 * @return  TRUE if initialization succeeds
 *          FALSE if initialization fails
 */
bool hal_adc_init(struct AdcConfig_t *config);

/**
 * @brief   Initialize the GPIO interface
 * @return  TRUE if initialization succeeds
 *          FALSE if initialization fails
 */
bool hal_gpio_init();

void hal_gpio_set_phase_high(MotorPhase_t phase);

void hal_gpio_set_phase_low(MotorPhase_t phase);

void hal_gpio_set_phase_float(MotorPhase_t phase);

void hal_pwm_set_duty(MotorPhase_t phase, uint16_t duty);

uint32_t hal_get_micros();

void hal_delay_us(uint32_t delay_us);

void hal_delay_ms(uint32_t delay_ms);

void hal_adc_start_conversion();

void hal_adc_get_phase_voltages(float *voltages);

void hal_adc_get_phase_currents(float *currents);

float hal_adc_get_dc_voltage();

float hal_adc_get_temperature();

bool hal_gpio_init_hall_sensors();

uint8_t hal_gpio_get_hall_state();

/** @} */
