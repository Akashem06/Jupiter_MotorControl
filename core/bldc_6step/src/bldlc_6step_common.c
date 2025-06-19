/*******************************************************************************************************************************
 * @file   bldc_6step_common.c
 *
 * @brief  Source file for the common functions in sensored/sensorless BLDC driver
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stddef.h>
#include <stdio.h>

/* Inter-component Headers */
#include "hal.h"

/* Intra-component Headers */
#include "bldc_6step_common.h"

/*******************************************************************************************************************************
 * Private Variables
 *******************************************************************************************************************************/

const uint8_t bldc_6step_commutation_table[NUM_COMMUTATION_STEPS][NUM_COMMUTATION_STEPS] = {
  /* High_A, Low_A, High_B, Low_B, High_C, Low_C */
  { 1U, 0U, 0U, 1U, 0U, 0U }, /* Step 1: A-High, B-Low */
  { 1U, 0U, 0U, 0U, 0U, 1U }, /* Step 2: A-High, C-Low */
  { 0U, 0U, 1U, 0U, 0U, 1U }, /* Step 3: B-High, C-Low */
  { 0U, 1U, 1U, 0U, 0U, 0U }, /* Step 4: B-High, A-Low */
  { 0U, 1U, 0U, 0U, 1U, 0U }, /* Step 5: C-High, A-Low */
  { 0U, 0U, 0U, 1U, 1U, 0U }, /* Step 6: C-High, B-Low */
};

/*******************************************************************************************************************************
 * Function Definitions
 *******************************************************************************************************************************/

void _6step_bldc_set_phase_outputs(const uint8_t commutation[NUM_COMMUTATION_STEPS], float pwm_duty) {
  /* Phase A */
  if (commutation[PHASE_A_HIGH_COMMUTATION_IDX] == 1U) {
    hal_pwm_set_duty(MOTOR_PHASE_A, pwm_duty);
  } else if (commutation[PHASE_A_LOW_COMMUTATION_IDX] == 1U) {
    hal_gpio_set_phase_low(MOTOR_PHASE_A);
  } else {
    hal_gpio_set_phase_float(MOTOR_PHASE_A);
  }

  /* Phase B */
  if (commutation[PHASE_B_HIGH_COMMUTATION_IDX] == 1U) {
    hal_pwm_set_duty(MOTOR_PHASE_B, pwm_duty);
  } else if (commutation[PHASE_B_LOW_COMMUTATION_IDX] == 1U) {
    hal_gpio_set_phase_low(MOTOR_PHASE_B);
  } else {
    hal_gpio_set_phase_float(MOTOR_PHASE_B);
  }

  /* Phase C */
  if (commutation[PHASE_C_HIGH_COMMUTATION_IDX] == 1U) {
    hal_pwm_set_duty(MOTOR_PHASE_C, pwm_duty);
  } else if (commutation[PHASE_C_LOW_COMMUTATION_IDX] == 1U) {
    hal_gpio_set_phase_low(MOTOR_PHASE_C);
  } else {
    hal_gpio_set_phase_float(MOTOR_PHASE_C);
  }
}

MotorPhase_t _6step_bldc_determine_floating_phase(uint8_t step) {
  switch (step) {
    case 0U:
      return MOTOR_PHASE_C; /**< A-High, B-Low -> C is floating */
    case 1U:
      return MOTOR_PHASE_B; /**< A-High, C-Low -> B is floating */
    case 2U:
      return MOTOR_PHASE_A; /**< B-High, C-Low -> A is floating */
    case 3U:
      return MOTOR_PHASE_C; /**< B-High, A-Low -> C is floating */
    case 4U:
      return MOTOR_PHASE_B; /**< C-High, A-Low -> B is floating */
    case 5U:
      return MOTOR_PHASE_A; /**< C-High, B-Low -> A is floating */
    default:
      return MOTOR_PHASE_C; /**< Invalid state */
  }
}

float _6step_bldc_get_conducting_current(struct Motor_t *motor, uint8_t step) {
  MotorPhase_t floating = _6step_bldc_determine_floating_phase(step);

  float sum = 0.0f;
  int count = 0;

  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; ++phase) {
    if (phase != floating) {
      sum += motor->state.phase_currents[phase];
      ++count;
    }
  }

  return (count > 0) ? (sum / (float)count) : 0.0f;
}

void _6step_bldc_stop_pwm_output() {
  hal_pwm_set_duty(MOTOR_PHASE_A, 0U);
  hal_pwm_set_duty(MOTOR_PHASE_B, 0U);
  hal_pwm_set_duty(MOTOR_PHASE_C, 0U);
  hal_gpio_set_phase_float(MOTOR_PHASE_A);
  hal_gpio_set_phase_float(MOTOR_PHASE_B);
  hal_gpio_set_phase_float(MOTOR_PHASE_C);
}
