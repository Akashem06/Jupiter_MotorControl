#pragma once

/*******************************************************************************************************************************
 * @file   pmsm_driver.h
 *
 * @brief  Header file for the PMSM driver
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
 * @defgroup PMSM PMSM class
 * @brief    Permanent magnet synchronous motor class
 * @{
 */

/**
 * @brief   PMSM data storage
 */
struct PMSMData_t {};

/**
 * @brief   Create PMSM motor driver
 * @param   motor Pointer to a motor instance
 */
void pmsm_create_driver(struct Motor_t *motor);

/** @} */
