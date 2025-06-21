#pragma once

/*******************************************************************************************************************************
 * @file   bldc_6step_sensorless.h
 *
 * @brief  Header file for the sensorless BLDC driver
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "bldc_6step_common.h"
#include "motor.h"

/**
 * @defgroup BLDCMotor BLDC motor class
 * @brief    Brushless DC motor class
 * @{
 */

typedef enum { ZC_STATE_RISING, ZC_STATE_FALLING } ZeroCrossingState_t;

struct BLDC6StepSensorlessData_t {
  uint8_t step;                          /**< Current commutation step (0-5) */
  bool direction;                        /**< Motor rotation direction (true for forward, false for reverse) */
  float pwm_duty;                        /**< Current PWM duty cycle applied to the high side */
  ZeroCrossingState_t zc_state;          /**< Expected zero-crossing state (rising or falling) */
  float zc_threshold;                    /**< Back-EMF zero-crossing threshold */
  float bemf[NUM_MOTOR_PHASES];          /**< Raw Back-EMF readings for each phase */
  float bemf_filtered[NUM_MOTOR_PHASES]; /**< Filtered Back-EMF readings */
  float bemf_filter_alpha;               /**< Alpha value for the BEMF low-pass filter */
  uint32_t last_zc_time;                 /**< Timestamp of the last zero-crossing event (microseconds) */
  uint32_t commutation_period;           /**< Estimated time for one 60-degree commutation step (microseconds) */
  float estimated_speed;                 /**< Estimated motor speed (RPM) */
  BLDC6StepMotorMode_t mode;             /**< Current motor operational mode */
};

/**
 * @brief   Initializes and registers the 6-step sensorless BLDC driver functions
 *          into the provided Motor_t structure
 * @param   motor Pointer to the Motor_t structure to be populated
 */
void bldc_6step_sensorless_create_driver(struct Motor_t *motor);

/** @} */
