#pragma once

/*******************************************************************************************************************************
 * @file   foc_observer.h
 *
 * @brief  Header file for FOC observer logic
 *
 * @date   2025-07-20
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>
#include <stdbool.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "motor_error.h"

/**
 * @defgroup FOC_PMSMMotor FOC control motor class
 * @brief    PMSM FOC control motor class
 * @{
 */

struct FOCObserver_t;

typedef enum {
  OBSERVER_TYPE_BACKEMF_PLL,
  OBSERVER_TYPE_SMO,
  OBSERVER_TYPE_EKF,
  /* TOOD: Add more observers */
} FOCObserverType_t;

struct FOCObserverDriver_t {
  MotorError_t (*init)(struct FOCObserver_t *observer);
  MotorError_t (*update)(struct FOCObserver_t *observer,
                         float v_alpha, float v_beta,
                         float i_alpha, float i_beta,
                         float dt,
                         float *theta_out, float *omega_out);
  MotorError_t (*reset)(struct FOCObserver_t *observer);
};

struct FOCObserver_t {
  float estimated_theta;  /**< Current angle estimate [rad] */
  float estimated_omega;  /**< Current speed estimate [rad/s] */

  float prev_theta;       /**< For delta tracking, filtering, etc. */
  float prev_omega;       /**< Optional filtering */

  FOCObserverType_t type; /**< Type of observer */
  struct FOCObserverDriver_t driver; /**< Driver for the observer */

  void *private_data;    /**< Implementation-specific state/config */
};

/** @} */
