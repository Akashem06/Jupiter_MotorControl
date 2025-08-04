#pragma once

/*******************************************************************************************************************************
 * @file   foc_common.h
 *
 * @brief  Header file for FOC common library
 *
 * @date   2025-06-18
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */
#include "motor_error.h"

/**
 * @defgroup FOC_PMSMMotor FOC control motor class
 * @brief    PMSM FOC control motor class
 * @{
 */

/**
 * @brief   Motor mode definitions
 */
typedef enum {
  MOTOR_MODE_IDLE,       /**< Motor is in idle state (Just initialized) */
  MOTOR_MODE_STOPPED,    /**< Motor is stopped */
  MOTOR_MODE_ALIGNING,   /**< Initial rotor alignment phase */
  MOTOR_MODE_OPEN_LOOP,  /**< Open-loop startup sequence */
  MOTOR_MODE_TRANSITION, /**< Transition from open to closed loop */
  MOTOR_MODE_RUNNING,    /**< Normal closed-loop operation */
  MOTOR_MODE_BRAKING,    /**< Active braking */
  MOTOR_MODE_ERROR       /**< Error state */
} FOCMotorMode_t;

/** @} */
