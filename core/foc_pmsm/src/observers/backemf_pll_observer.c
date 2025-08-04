/*******************************************************************************************************************************
 * @file   backemf_pll_observer.c
 *
 * @brief  Source file for FOC back-EMF and PLL observer
 *
 * @date   2025-07-20
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stddef.h>

/* Inter-component Headers */
#include "math_utils.h"
#include "hal.h"

/* Intra-component Headers */
#include "foc_observer.h"
#include "backemf_pll_observer.h"

/* Static instance for private data */
static struct BackEMFPLLData_t s_backemf_pll_data = {0};

#define MIN_BEMF_MAGNITUDE (0.01f)

static void calculate_back_emf(struct BackEMFPLLData_t *bemf_pll_data,
                              float v_alpha, float v_beta,
                              float i_alpha, float i_beta,
                              float dt) {
    struct BackEMFPLLConfig_t *cfg = &bemf_pll_data->config;
    
    /* Calculate back-EMF using a motor model: e = v - Rs*i - Ls*di/dt */
    /* TODO: Handle di/dt term by storing previous current reading */
    bemf_pll_data->bemf_alpha = v_alpha - cfg->Rs * i_alpha;
    bemf_pll_data->bemf_beta = v_beta - cfg->Rs * i_beta;
    
    /* Calculate magnitude */
    bemf_pll_data->bemf_magnitude = sqrtf(bemf_pll_data->bemf_alpha * bemf_pll_data->bemf_alpha + 
                                          bemf_pll_data->bemf_beta * bemf_pll_data->bemf_beta);
}

static void run_pll(struct BackEMFPLLData_t *bemf_pll_data, float dt, 
                   float *theta_out, float *omega_out) {
    const struct BackEMFPLLConfig_t *cfg = &bemf_pll_data->config;
    
    /* Skip PLL if back-EMF magnitude is too small. We can assume it didn't move much */
    if (bemf_pll_data->bemf_magnitude < MIN_BEMF_MAGNITUDE) {
        *theta_out = bemf_pll_data->pll_state.theta;
        *omega_out = bemf_pll_data->pll_state.omega;
        return;
    }
    
    /* Calculate estimated back-EMF based on current theta */
    float cos_theta = cosf(bemf_pll_data->pll_state.theta);
    float sin_theta = sinf(bemf_pll_data->pll_state.theta);

    /* Expected back-EMF direction */
    float expected_bemf_alpha = -bemf_pll_data->bemf_magnitude * sin_theta;
    float expected_bemf_beta = bemf_pll_data->bemf_magnitude * cos_theta;

    /* Phase error calculation (cross product) */
    float phase_error = (bemf_pll_data->bemf_alpha * expected_bemf_beta - 
                        bemf_pll_data->bemf_beta * expected_bemf_alpha) / 
                       (bemf_pll_data->bemf_magnitude * bemf_pll_data->bemf_magnitude + 1e-6f);
                       
    pll_update(&bemf_pll_data->pll_state, phase_error, hal_get_micros() - bemf_pll_data->last_pll_estimate_time, &bemf_pll_data->position_radians, &bemf_pll_data->angular_velocity);

    bemf_pll_data->last_pll_estimate_time = hal_get_micros();
}

static MotorError_t foc_observer_backemf_pll_init(struct FOCObserver_t *observer) {
    if (observer == NULL) {
        return MOTOR_INVALID_ARGS;
    }
    
    struct BackEMFPLLData_t *bemf_pll_data = (struct BackEMFPLLData_t *)observer->private_data;
    
    /* Reset all state variables */
    bemf_pll_data->bemf_alpha = 0.0f;
    bemf_pll_data->bemf_beta = 0.0f;
    bemf_pll_data->bemf_magnitude = 0.0f;
    bemf_pll_data->update_count = 0;
    bemf_pll_data->position_radians = 0.0f;
    bemf_pll_data->angular_velocity = 0.0f;
    bemf_pll_data->is_initialized = true;
    bemf_pll_data->last_pll_estimate_time = hal_get_micros();

    pll_init(&bemf_pll_data->pll_state, &bemf_pll_data->config->pll_cfg);
    
    return MOTOR_OK;
}

static MotorError_t foc_observer_backemf_pll_update(struct FOCObserver_t *observer,
                                        float v_alpha, float v_beta,
                                        float i_alpha, float i_beta,
                                        float dt,
                                        float *theta_out, float *omega_out) {
    if (observer == NULL || theta_out == NULL || omega_out == NULL) {
        return MOTOR_INVALID_ARGS;
    }
    
    if (dt <= 0.0f) {
        return MOTOR_INVALID_ARGS;
    }
    
    struct BackEMFPLLData_t *bemf_pll_data = (struct BackEMFPLLData_t *)observer->private_data;
    
    if (!bemf_pll_data->is_initialized) {
        return MOTOR_UNINITIALIZED;
    }
    
    /* Calculate back-EMF */
    calculate_back_emf(bemf_pll_data, v_alpha, v_beta, i_alpha, i_beta, dt);
    
    /* Run PLL algorithm */
    run_pll(bemf_pll_data, dt, theta_out, omega_out);
    
    /* Increment update counter */
    bemf_pll_data->update_count++;
    
    return MOTOR_OK;
}

static MotorError_t foc_observer_backemf_pll_reset(struct FOCObserver_t *observer) {
    if (observer == NULL) {
        return MOTOR_INVALID_ARGS;
    }
    
    struct BackEMFPLLData_t *bemf_pll_data = (struct BackEMFPLLData_t *)observer->private_data;
    
    /* Reset dynamic state variables but keep configuration */
    bemf_pll_data->pll_state.integrator = 0.0f;
    bemf_pll_data->pll_state.prev_error = 0.0f;
    bemf_pll_data->bemf_alpha = 0.0f;
    bemf_pll_data->bemf_beta = 0.0f;
    bemf_pll_data->bemf_magnitude = 0.0f;
    bemf_pll_data->pll_state.theta = 0.0f;
    bemf_pll_data->pll_state.omega = 0.0f;
    bemf_pll_data->update_count = 0;
    
    return MOTOR_OK;
}

MotorError_t foc_observer_backemf_pll_create_driver(struct FOCObserver_t *observer, struct BackEMFPLLConfig_t *config) {
    if (observer == NULL || config == NULL) {
        return MOTOR_INVALID_ARGS;
    }

    /* Set up driver function pointers */
    observer->driver.init = foc_observer_backemf_pll_init;
    observer->driver.update = foc_observer_backemf_pll_update;
    observer->driver.reset = foc_observer_backemf_pll_reset;

    /* Set observer type */
    observer->type = OBSERVER_TYPE_BACKEMF_PLL;

    /* Point private data to static instance */
    observer->private_data = &s_backemf_pll_data;

    /* Store configuration */
    s_backemf_pll_data.config = config;

    /* Initialize state */
    s_backemf_pll_data.is_initialized = false;

    return MOTOR_OK;
}
