#pragma once

/*******************************************************************************************************************************
 * @file   motor_error.h
 *
 * @brief  Header file for the Motor errors
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup MotorClass Motor storage class
 * @brief    Motor agonistic storage class
 * @{
 */

/**
 * @brief   Motor error class
 */
typedef enum {
  MOTOR_OK,
  MOTOR_INVALID_ARGS,
  MOTOR_INIT_ERROR,
  MOTOR_OVERVOLTAGE_ERROR,
  MOTOR_OVERCURRENT_ERROR,
  MOTOR_HAL_ERROR,
  MOTOR_INTERNAL_ERROR,
} MotorError_t;

/** @} */
