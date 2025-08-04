#pragma once

/*******************************************************************************************************************************
 * @file   pll.h
 *
 * @brief  Header file for Phase Lock Loop
 *
 * @date   2025-08-03
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "utils_error.h"

/**
 * @defgroup PLLControl_Utils Phase Lock Loop Controller
 * @brief    Phase Lock Loop Control utilities for 3-phase inverters
 * @{
 */

struct PLLConfig_t {
    float kp;
    float ki;
    float max_omega;
    float filter_alpha;
    bool enable_filtering;
};

struct PLLState_t {
    float integrator;
    float prev_error;
    float theta;
    float omega;
    float max_error;
    bool is_converged;
    struct PLLConfig_t *cfg;
};

UtilsError_t pll_init(struct PLLState_t *state, const struct PLLConfig_t *cfg);
UtilsError_t pll_update(struct PLLState_t *state,
                        float phase_error, float dt,
                        float *theta_out, float *omega_out);

/** @} */
