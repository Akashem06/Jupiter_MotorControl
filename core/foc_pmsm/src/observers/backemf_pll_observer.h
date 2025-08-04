#pragma once

/*******************************************************************************************************************************
 * @file   backemf_pll_observer.h
 *
 * @brief  Header file for FOC back-EMF and PLL observer
 *
 * @date   2025-07-20
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>
#include <stdbool.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "motor_error.h"
#include "pll.h"

/**
 * @defgroup FOC_Observers FOC observers
 * @brief    FOC observers
 * @{
 */


/**
 * @brief Back-EMF PLL observer configuration parameters
 */
struct BackEMFPLLConfig_t {
    struct PLLConfig_t pll_cfg; /**< PLL Config */
    float Rs;                   /**< Stator resistance [Ohm] */
    float Ls;                   /**< Stator inductance [H] */
    float lambda_pm;            /**< Permanent magnet flux linkage [Wb] */
    float min_speed;            /**< Minimum speed for observer operation [rad/s] */
    float max_speed;            /**< Maximum speed limit [rad/s] */
};

/**
 * @brief Back-EMF PLL observer internal state
 */
struct BackEMFPLLData_t {
    struct PLLState_t pll_state;
    struct BackEMFPLLConfig_t *config; /**< Configuration parameters */

    /* Back-EMF estimation */
    float bemf_alpha;          /**< Alpha-axis back-EMF [V] */
    float bemf_beta;           /**< Beta-axis back-EMF [V] */
    float bemf_magnitude;      /**< Back-EMF magnitude [V] */
    
    float position_radians;
    float angular_velocity;

    /* Status flags */
    bool is_initialized;       /**< Initialization status */

    /* Statistics/debugging */
    uint32_t update_count;     /**< Update cycle counter */
    uint32_t last_pll_estimate_time;
};

/**
 * @brief Create and initialize a Back-EMF PLL observer driver
 * 
 * This function sets up the observer driver function pointers and allocates
 * the internal state structure with default configuration.
 * 
 * @param[in,out] observer Pointer to FOC observer structure
 * @param[in] config       Pointer to configuration parameters (NULL for defaults)
 * 
 * @return MotorError_t
 * @retval MOTOR_ERROR_NONE           Success
 * @retval MOTOR_ERROR_NULL_POINTER   Invalid observer pointer
 * @retval MOTOR_ERROR_INIT_FAILED    Initialization failed
 */
MotorError_t foc_observer_backemf_pll_create_driver(struct FOCObserver_t *observer, struct BackEMFPLLConfig_t *config);

/**
 * @brief Get observer status and statistics
 * 
 * @param[in] observer     Pointer to FOC observer structure
 * @param[out] is_converged Convergence status flag
 * @param[out] update_count Number of updates performed
 * @param[out] max_error   Maximum phase error observed
 * 
 * @return MotorError_t
 * @retval MOTOR_ERROR_NONE           Success
 * @retval MOTOR_ERROR_NULL_POINTER   Invalid pointer
 * @retval MOTOR_ERROR_NOT_INITIALIZED Observer not initialized
 */
MotorError_t foc_observer_backemf_pll_get_status(const struct FOCObserver_t *observer,
                                                 bool *is_converged,
                                                 uint32_t *update_count,
                                                 float *max_error);

/**
 * @brief Get estimated back-EMF components
 * 
 * @param[in] observer    Pointer to FOC observer structure
 * @param[out] bemf_alpha Alpha-axis back-EMF [V]
 * @param[out] bemf_beta  Beta-axis back-EMF [V]
 * @param[out] bemf_mag   Back-EMF magnitude [V]
 * 
 * @return MotorError_t
 * @retval MOTOR_ERROR_NONE           Success
 * @retval MOTOR_ERROR_NULL_POINTER   Invalid pointer
 * @retval MOTOR_ERROR_NOT_INITIALIZED Observer not initialized
 */
MotorError_t foc_observer_backemf_pll_get_bemf(const struct FOCObserver_t *observer,
                                               float *bemf_alpha,
                                               float *bemf_beta, 
                                               float *bemf_mag);

/** @} */
