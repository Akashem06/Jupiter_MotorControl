#pragma once

/*******************************************************************************************************************************
 * @file   pmsm_model.h
 *
 * @brief  Header file for the simulation PMSM motor model
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
 * @defgroup SimulationPMSMMotorModel PMSM Motor Model for Simulation
 * @brief    Simulation class to model PMSM Motor characteristics
 * @{
 */

/**
 * @brief   Create PMSM motor model
 * @param   model Pointer to a model instance
 */
void pmsm_create_model(struct MotorModel_t *model);

/** @} */
