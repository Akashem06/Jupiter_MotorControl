/*******************************************************************************************************************************
 * @file   pll.c
 *
 * @brief  Source file for Phase Lock Loop
 *
 * @date   2025-08-03
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stddef.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "pll.h"
#include "math_utils.h"

#define CONVERGENCE_THRESHOLD   0.05f
#define MAX_PHASE_ERROR         MATH_TWO_PI
#define MAX_INTEGRATOR          50.0f

UtilsError_t pll_init(struct PLLState_t *state, const struct PLLConfig_t *cfg) {
    if (state == NULL || cfg == NULL) {
        return UTILS_INVALID_ARGS;
    }

    state->theta = 0.0f;
    state->omega = 0.0f;
    state->integrator = 0.0f;
    state->max_error = 0.0f;
    state->is_converged = false;
    state->prev_error = 0.0f;
    state->cfg = cfg;

    return UTILS_OK;
}


UtilsError_t pll_update(struct PLLState_t *state,
                        float phase_error, float dt,
                        float *theta_out, float *omega_out) {
    if (state == NULL || theta_out == NULL || omega_out == NULL) {
        return UTILS_INVALID_ARGS;
    }

    /* Clamp error */
    phase_error = clamp(phase_error, -MAX_PHASE_ERROR, MAX_PHASE_ERROR);

    float abs_error = fabsf(phase_error);
    if (abs_error > state->max_error) {
        state->max_error = abs_error;
    }

    /* Check if phase is converged/aligned */
    state->is_converged = abs_error < CONVERGENCE_THRESHOLD;

    /* PI controller (Output is angular velocity) */
    state->integrator += state->cfg->ki * phase_error * dt;
    state->integrator = clamp(state->integrator, -MAX_INTEGRATOR, MAX_INTEGRATOR);

    float omega = state->cfg->kp * phase_error + state->integrator;

    if (omega > state->cfg->max_omega) omega = state->cfg->max_omega;
    if (omega < -state->cfg->max_omega) omega = -state->cfg->max_omega;

    /* Predict new theta (angular_vel * time = rotational distance) */
    float theta = state->theta + omega * dt;

    /* Apply filtering */
    if (state->cfg->enable_filtering) {
        state->theta = state->cfg->filter_alpha * state->theta + (1.0f - state->cfg->filter_alpha) * theta;
        state->omega = state->cfg->filter_alpha * state->omega + (1.0f - state->cfg->filter_alpha) * omega;
    } else {
        state->theta = theta;
        state->omega = omega;
    }

    if (theta_out) *theta_out = state->theta;
    if (omega_out) *omega_out = state->omega;

    return UTILS_OK;
}
