#pragma once

/*******************************************************************************************************************************
 * @file   bldc_6step_sensored.h
 *
 * @brief  Header file for the sensored BLDC driver
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

struct BLDC6StepSensoredData_t {
  uint8_t step;                   /**< Current commutation step (0-5) */
  bool direction;                 /**< Motor rotation direction (true for forward, false for reverse) */
  float pwm_duty;                 /**< Current PWM duty cycle applied to the high side */
  uint8_t last_hall_state;        /**< Last read Hall sensor state */
  uint32_t last_commutation_time; /**< Timestamp of the last commutation (microseconds) */
  uint32_t commutation_period;    /**< Estimated time for one 60-degree commutation step (microseconds) */
  float estimated_speed;          /**< Estimated motor speed (RPM) */
  MotorMode_t mode;               /**< Current motor operational mode */
};

/**
 * @brief   Initializes and registers the 6-step sensored BLDC driver functions
 *          into the provided Motor_t structure
 * @param   motor Pointer to the Motor_t structure to be populated
 */
void bldc_6step_sensored_create_driver(struct Motor_t *motor);

/** @} */
