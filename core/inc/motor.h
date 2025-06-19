#pragma once

/*******************************************************************************************************************************
 * @file   motor.h
 *
 * @brief  Header file for the all motor control loops
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */
#include "hal.h"
#include "pid.h"

/* Intra-component Headers */
#include "motor_error.h"

/**
 * @defgroup MotorClass Motor storage class
 * @brief    Motor agonistic storage class
 * @{
 */

struct Motor_t;

/**
 * @brief   Control modes
 */
typedef enum {
  CONTROL_MODE_VOLTAGE,  /**< Voltage control */
  CONTROL_MODE_CURRENT,  /**< Current control */
  CONTROL_MODE_VELOCITY, /**< Velocity control */
  CONTROL_MODE_POSITION, /**< Position control */
  CONTROL_MODE_TORQUE    /**< Torque control */
} ControlMode_t;

/**
 * @brief   Motor types
 */
typedef enum {
  MOTOR_TYPE_BLDC,   /**< Brushless DC */
  MOTOR_TYPE_PMSM,   /**< Permanent Magnet Synchronous */
  MOTOR_TYPE_STEPPER /**< Stepper Motor */
} MotorType_t;

/**
 * @brief   Control methods
 */
typedef enum {
  CONTROL_METHOD_SIX_STEP,   /**< Basic 6-step commutation */
  CONTROL_METHOD_FOC,        /**< Field Oriented Control */
  CONTROL_METHOD_DTC,        /**< Direct Torque Control */
  CONTROL_METHOD_SENSORLESS, /**< Sensorless (Back-EMF based) */
  CONTROL_METHOD_VF          /**< V/f Control */
} ControlMethod_t;

/**
 * @brief   Motor state storage class
 */
struct MotorState_t {
  float phase_voltages[NUM_MOTOR_PHASES]; /**< Phase voltages */
  float phase_currents[NUM_MOTOR_PHASES]; /**< Phase currents */
  float dc_voltage;                       /**< DC Voltage */
  float position;                         /**< Motor position */
  float velocity;                         /**< Motor velocity */
  float temperature;                      /**< Motor temperature*/
  bool is_initialized;                    /**< Initialization state */
  uint32_t last_update_time;              /**< Timestamp of last state update */
};

/**
 * @brief   Motor configuration class
 */
struct MotorConfig_t {
  MotorType_t type;                       /**< Motor type selection */
  ControlMethod_t control_method;         /**< Motor control method selection */
  ControlMode_t control_mode;             /**< Motor control mode */
  uint8_t pole_pairs;                     /**< Number of pole pairs */
  float phase_resistance;                 /**< Phase resistance */
  float phase_inductance;                 /**< Phase inductance */
  float max_current;                      /**< Maximum current */
  float max_voltage;                      /**< Maximum voltage */
  float max_velocity;                     /**< Maximum velocity */
  float min_startup_speed;                /**< Minimum startup speed */
  float torque_constant;                  /**< Torque constant of the motor (Nm/A) */
  struct PidConfig_t current_pid_config;  /**< Current PID Configuration */
  struct PidConfig_t voltage_pid_config;  /**< Voltage PID Configuration */
  struct PidConfig_t velocity_pid_config; /**< Velocity PID Configuration */

  struct PwmConfig_t pwm_config;
  struct AdcConfig_t adc_config;
};

struct MotorDriver_t {
  MotorError_t (*init)(struct Motor_t *motor, struct MotorConfig_t *config); /**< Motor initializer */
  MotorError_t (*deinit)(struct Motor_t *motor);                             /**< Motor deinitializer */

  MotorError_t (*update_state)(struct Motor_t *motor); /**< Update motor state */
  MotorError_t (*commutate)(struct Motor_t *motor);    /**< Commutate motor */
  MotorError_t (*update_pwm)(struct Motor_t *motor);   /**< Update motor PWM */

  MotorError_t (*set_voltage)(struct Motor_t *motor, float voltage);   /**< Update voltage setpoint */
  MotorError_t (*set_current)(struct Motor_t *motor, float current);   /**< Update current setpoint */
  MotorError_t (*set_velocity)(struct Motor_t *motor, float velocity); /**< Update velocity setpoint */
  MotorError_t (*set_position)(struct Motor_t *motor, float position); /**< Update position setpoint */
  MotorError_t (*set_torque)(struct Motor_t *motor, float torque);     /**< Update torque setpoint */

  MotorError_t (*foc_transform)(struct Motor_t *motor);     /**< Clarke/Park transforms */
  MotorError_t (*estimate_position)(struct Motor_t *motor); /**< For sensorless */
  MotorError_t (*estimate_speed)(struct Motor_t *motor);    /**< For sensorless */
};

/**
 * @brief   Motor storage class
 */
struct Motor_t {
  struct MotorConfig_t *config; /**< Pointer to the motor configuration class */
  struct MotorDriver_t driver;  /**< Motor driver */
  struct MotorState_t state;    /**< Motor state */
  MotorError_t motor_error;     /**< Motor error tracker */

  /**
   * @brief   Control loop setpoints
   */
  struct {
    float voltage;  /**< Setpoint voltage */
    float current;  /**< Setpoint current */
    float velocity; /**< Setpoint velocity */
    float position; /**< Setpoint position */
    float torque;   /**< Setpoint torque */
  } setpoint;

  /**
   * @brief   Control loop PID controllers
   */
  struct {
    struct PidController_t current;  /**< Current PID controller */
    struct PidController_t velocity; /**< Velocity PID controller */
    struct PidController_t position; /**< Position PID controller */
  } control;

  void *private_data; /**< Private data for motor-specific data */
};

/**
 * @brief   Main control loop for the motor
 */
MotorError_t motor_run(struct Motor_t *motor);

/** @} */
