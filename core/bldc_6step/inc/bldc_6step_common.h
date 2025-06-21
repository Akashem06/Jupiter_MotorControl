#pragma once

/*******************************************************************************************************************************
 * @file   bldc_6step_common.h
 *
 * @brief  Header file for the common functions in sensored/sensorless BLDC driver
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

/*******************************************************************************************************************************
 * Private defines and enums
 *******************************************************************************************************************************/

#define PHASE_A_HIGH_COMMUTATION_IDX 0U
#define PHASE_A_LOW_COMMUTATION_IDX 1U
#define PHASE_B_HIGH_COMMUTATION_IDX 2U
#define PHASE_B_LOW_COMMUTATION_IDX 3U
#define PHASE_C_HIGH_COMMUTATION_IDX 4U
#define PHASE_C_LOW_COMMUTATION_IDX 5U
#define NUM_COMMUTATION_STEPS 6U

#define DEFAULT_STARTUP_DUTY 0.20f             // Default startup PWM duty cycle (in decimal)
#define DEFAULT_ALIGNMENT_TIME_MS 500U         // Default time for initial rotor alignment (in ms)
#define STARTUP_MIN_PERIOD_US 50000U           // Slowest commutation period during startup (20Hz)
#define STARTUP_MAX_PERIOD_US 5000U            // Fastest commutation period during startup (200Hz)
#define DEFAULT_STARTUP_STEPS 12U              // Number of open-loop commutation steps during startup
#define STARTUP_ACCELERATION_FACTOR 0.8f       // Acceleration factor for each step (smaller = faster)
#define STARTUP_DUTY_INCREMENT_PER_STEP 0.05f  // PWM duty cycle increment per step during startup
#define MAX_STALL_TIME_MS 500U                 // Maximum time without movement before stall detection

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
} BLDC6StepMotorMode_t;

/*******************************************************************************************************************************
 * Variables
 *******************************************************************************************************************************/

extern const uint8_t bldc_6step_commutation_table[NUM_COMMUTATION_STEPS][NUM_COMMUTATION_STEPS];

/*******************************************************************************************************************************
 * Function definitions
 *******************************************************************************************************************************/

/**
 * @brief   Sets the PWM duty cycle or low/float state for each motor phase based on the commutation
 * map
 * @param   commutation The 6-element array representing the current commutation step (1U for
 * active, 0U for inactive)
 * @param   pwm_duty The PWM duty cycle to apply to the HIGH side of the active phase
 */
void _6step_bldc_set_phase_outputs(const uint8_t commutation[NUM_COMMUTATION_STEPS], float pwm_duty);

/**
 * @brief   Determines the floating (un-driven) phase for a given 6-step commutation step
 * @param   step The current commutation step (0-5)
 * @return  MotorPhase_t corresponding to the floating phase
 */
MotorPhase_t _6step_bldc_determine_floating_phase(uint8_t step);

/**
 * @brief   Determines the conducting currents by summing the 2 conducting phase currents
 * @param   motor Pointer to the motor instance
 * @param   step The current commutation step (0-5)
 * @return  Calculated float value of the conducting current
 */
float _6step_bldc_get_conducting_current(struct Motor_t *motor, uint8_t step);

/**
 * @brief   Sets all phase currents to 0 and stops all PWM output
 */
void _6step_bldc_stop_pwm_output();

/** @} */
