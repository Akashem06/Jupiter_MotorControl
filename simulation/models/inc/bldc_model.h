#pragma once

/*******************************************************************************************************************************
 * @file   bldc_model.h
 *
 * @brief  Header file for the simulation BLDC motor model
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
 * @defgroup SimulationBLDCMotorModel BLDC Motor Model for Simulation
 * @brief    Simulation class to model BLDC Motor characteristics
 * @{
 */

/**
 * @brief   Create BLDC motor model
 * @param   model Pointer to a model instance
 */
void bldc_create_model(struct MotorModel_t *model);

/** @} */
