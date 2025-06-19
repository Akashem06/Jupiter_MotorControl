#pragma once

/*******************************************************************************************************************************
 * @file   bldc_6step_driver.h
 *
 * @brief  Header file for the BLDC driver
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "motor.h"

/**
 * @defgroup BLDCMotor BLDC motor class
 * @brief    Brushless DC motor class
 * @{
 */

/**
 * @brief   Zero-crossing detection states
 */
typedef enum {
  ZC_STATE_RISING,  /**< Rising edge expected */
  ZC_STATE_FALLING, /**< Falling edge expected */
  ZC_STATE_INVALID  /**< Invalid/unknown state */
} ZeroCrossingState_t;

/**
 * @brief   Motor mode definitions
 */
typedef enum {
  MOTOR_MODE_STOPPED,    /**< Motor is stopped */
  MOTOR_MODE_ALIGNING,   /**< Initial rotor alignment phase */
  MOTOR_MODE_OPEN_LOOP,  /**< Open-loop startup sequence */
  MOTOR_MODE_TRANSITION, /**< Transition from open to closed loop */
  MOTOR_MODE_RUNNING,    /**< Normal closed-loop operation */
  MOTOR_MODE_BRAKING,    /**< Active braking */
  MOTOR_MODE_ERROR       /**< Error state */
} MotorMode_t;

/**
 * @brief   BLDC Motor data storage
 */
struct BLDC_6Step_Data_t {
  uint8_t step;                 /**< Current commutation step (0-5) */
  float pwm_duty;               /**< Current PWM duty cycle */
  float bemf[NUM_MOTOR_PHASES]; /**< Back-EMF measurements for each phase */
  bool direction;               /**< Rotation direction (true = forward) */

  uint32_t last_zc_time;        /**< Last zero crossing timestamp */
  ZeroCrossingState_t zc_state; /**< Current zero-crossing state */
  float zc_threshold;           /**< Zero crossing detection threshold */

  float estimated_speed;       /**< Estimated motor speed */
  uint32_t commutation_period; /**< Time between commutations */

  float bemf_filtered[NUM_MOTOR_PHASES]; /**< Filtered Back-EMF values */
  float bemf_filter_alpha;               /**< Back-EMF filter coefficient */

  MotorMode_t mode; /**< Motor mode */
};

/**
 * @brief   Create BLDC motor driver
 * @param   motor Pointer to a motor instance
 */
void bldc_6step_create_driver(struct Motor_t *motor);

/** @} */
