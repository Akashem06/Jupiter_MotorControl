#pragma once

/*******************************************************************************************************************************
 * @file   stepper_motor_driver.h
 *
 * @brief  Header file for the Stepper motor driver
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
 * @defgroup StepperMotor Stepper motor class
 * @brief    Brushless DC motor class
 * @{
 */

/**
 * @brief   Stepper Motor data storage
 */
struct StepperMotorData_t {};

/**
 * @brief   Create Stepper motor driver
 * @param   motor Pointer to a motor instance
 */
void stepper_motor_create_driver(struct Motor_t *motor);

/** @} */
