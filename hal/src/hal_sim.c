/*******************************************************************************************************************************
 * @file   hal_sim.c
 *
 * @brief  Source file for simulation Hardware Abstraction Layer
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <time.h>

/* Inter-component Headers */
#include "motor_model.h"

/* Intra-component Headers */
#include "hal.h"

static struct MotorModel_t *s_motor_model = NULL;
static struct PwmConfig_t *s_pwm_config;
static struct AdcConfig_t *s_adc_config;
static uint32_t s_last_update_time = 0;

static struct timespec start_time;

bool hal_pwm_init(struct PwmConfig_t *config) {
  if (config == NULL) {
    return false;
  }
  s_pwm_config = config;
  return true;
}

bool hal_adc_init(struct AdcConfig_t *config) {
  if (config == NULL) {
    return false;
  }
  s_adc_config = config;
  return true;
}

bool hal_gpio_init() {
  return true;
}

void hal_gpio_set_phase_high(MotorPhase_t phase) {
  if (s_motor_model != NULL && phase < NUM_MOTOR_PHASES) {
    motor_model_set_phase_voltage(s_motor_model, phase, s_motor_model->config->dc_voltage);
  }
}

void hal_gpio_set_phase_low(MotorPhase_t phase) {
  if (s_motor_model != NULL && phase < NUM_MOTOR_PHASES) {
    motor_model_set_phase_voltage(s_motor_model, phase, 0.0f);
  }
}

void hal_gpio_set_phase_float(MotorPhase_t phase) {
  if (s_motor_model != NULL && phase < NUM_MOTOR_PHASES) {
    motor_model_set_phase_float(s_motor_model, phase);
  }
}

void hal_pwm_set_duty(MotorPhase_t phase, uint16_t duty) {
  if (s_motor_model != NULL && phase < NUM_MOTOR_PHASES) {
    float duty_float = (float)duty / (float)((1U << s_pwm_config->resolution) - 1U);
    float voltage = duty_float * s_motor_model->config->dc_voltage;

    motor_model_set_phase_voltage(s_motor_model, phase, voltage);
  }
}

uint32_t hal_get_micros() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (ts.tv_sec * 1000000) + (ts.tv_nsec / 1000);
}

void hal_adc_start_conversion() {
  if (s_motor_model != NULL) {
    uint32_t current_time = hal_get_micros();

    float dt = (float)(current_time - s_last_update_time) / 1000000.0f;
    motor_model_step(s_motor_model, dt);
    s_last_update_time = current_time;
  }
}

void hal_adc_get_phase_voltages(float *voltages) {
  if (s_motor_model != NULL && voltages != NULL) {
    motor_model_get_phase_voltages(s_motor_model, voltages);
  }
}

void hal_adc_get_phase_currents(float *currents) {
  if (s_motor_model != NULL && currents != NULL) {
    motor_model_get_phase_currents(s_motor_model, currents);
  }
}

float hal_adc_get_dc_voltage() {
  if (s_motor_model != NULL) {
    return s_motor_model->config->dc_voltage;
  }

  return 0.0f;
}

float hal_adc_get_temperature() {
  /* Return default temperature */
  return 25.0f;
}

void hal_sim_set_model(struct MotorModel_t *model) {
  s_motor_model = model;
  s_last_update_time = hal_get_micros();
}
