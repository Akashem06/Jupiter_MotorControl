#pragma once

/*******************************************************************************************************************************
 * @file   sim_manager.h
 *
 * @brief  Header file for the simulation manager
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "motor_model.h"

/**
 * @brief   Simulation Manager class
 */
struct SimManager_t {
  struct Motor_t motor;
  struct MotorModel_t model;
  struct MotorConfig_t motor_config;
  struct MotorModelConfig_t model_config;
  bool is_running;
  uint32_t sim_time_us;
  uint32_t last_print_time;
};

bool sim_manager_init(struct SimManager_t *sim);
void sim_manager_run(struct SimManager_t *sim);
void sim_manager_stop(struct SimManager_t *sim);

/** @} */
