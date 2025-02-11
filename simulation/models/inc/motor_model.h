#pragma once

/*******************************************************************************************************************************
 * @file   motor_model.h
 *
 * @brief  Header file for the simulation motor model
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */
#include "motor.h"

/* Intra-component Headers */

/**
 * @defgroup SimulationMotorModel Motor Model for Simulation
 * @brief    Simulation class to model Motor characteristics
 * @{
 */

/**
 * @brief   Base configuration shared by all motor types
 */
struct MotorModelConfig_t {
  MotorType_t type;           /**< Motor type identifier */
  float moment_of_inertia;    /**< Rotor moment of inertia (kg⋅m²) */
  float friction_coefficient; /**< Friction coefficient (N⋅m⋅s) */
  float dc_voltage;           /**< DC bus voltage (V) */
  uint8_t pole_pairs;         /**< Number of pole pairs */

  float phase_resistance; /**< Phase resistance (Ω) */
  float phase_inductance; /**< Phase inductance (H) */

  float max_current; /**< Maximum current (A) */
  float max_voltage; /**< Maximum voltage (V) */

  /**
   * @brief   Motor specific configuration parameters
   */
  union {
    /**
     * @brief   BLDC/PMSM specific parameters
     */
    struct {
      float back_emf_constant; /**< Back EMF constant (V/rad/s) */
      float torque_constant;   /**< Torque constant (N⋅m/A) */
    } ac;

    /**
     * @brief   DC motor specific parameters
     */
    struct {
      float armature_resistance; /**< Armature resistance (Ω) */
      float armature_inductance; /**< Armature inductance (H) */
      float field_constant;      /**< Field constant (V/rad/s) */
    } dc;

    /**
     * @brief   Stepper motor specific parameters
     */
    struct {
      float detent_torque;    /**< Detent torque (N⋅m) */
      float holding_torque;   /**< Holding torque (N⋅m) */
      uint16_t steps_per_rev; /**< Steps per revolution */
    } stepper;
  } params;
};

/**
 * @brief   Motor model state storage
 */
struct MotorModelState_t {
  float phase_voltages[NUM_MOTOR_PHASES]; /**< Phase voltages (V) */
  float phase_currents[NUM_MOTOR_PHASES]; /**< Phase currents (A) */
  float rotor_angle;                      /**< Mechanical angle (rad) */
  float rotor_speed;                      /**< Mechanical speed (rad/s) */
  float electrical_angle;                 /**< Electrical angle (rad) */
  float load_torque;                      /**< External load torque (N⋅m) */

  /**
   * @brief   Motor specific state traits
   */
  union {
    /**
     * @brief   AC motor specific state traits
     */
    struct {
      float d_current; /**< d-axis current for FOC */
      float q_current; /**< q-axis current for FOC */
      float back_emf[NUM_MOTOR_PHASES];
    } ac;

    /**
     * @brief   DC motor specific state traits
     */
    struct {
      float armature_current; /**< Armature current for DC */
      float field_current;    /**< Field current for DC */
    } dc;

    /**
     * @brief   Stepper motor specific state traits
     */
    struct {
      uint16_t current_step; /**< Current step position */
      float cogging_torque;  /**< Current cogging torque */
    } stepper;
  } traits;
};

/**
 * @brief Motor model class
 */
struct MotorModel_t {
  struct MotorModelConfig_t *config;
  struct MotorModelState_t state;

  /* Function pointers for motor-type specific calculations */
  void (*calculate_back_emf)(struct MotorModel_t *model);
  void (*calculate_torque)(struct MotorModel_t *model, float *torque);
  void (*update_electrical)(struct MotorModel_t *model, float dt);
};

void motor_model_init(struct MotorModel_t *model, struct MotorModelConfig_t *config);
void motor_model_step(struct MotorModel_t *model, float dt);
void motor_model_set_pwm_duty(struct MotorModel_t *model, float duty);
void motor_model_set_phase_voltage(struct MotorModel_t *model, uint8_t phase, float voltage);
void motor_model_set_phase_float(struct MotorModel_t *model, uint8_t phase);
void motor_model_get_phase_voltages(struct MotorModel_t *model, float *voltages);
void motor_model_get_phase_currents(struct MotorModel_t *model, float *currents);

void hal_sim_set_model(struct MotorModel_t *model);

/** @} */
