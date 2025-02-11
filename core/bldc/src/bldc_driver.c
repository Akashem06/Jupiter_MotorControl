/*******************************************************************************************************************************
 * @file   bldc_driver.c
 *
 * @brief  Source file for the BLDC driver
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stddef.h>
#include <stdio.h>

/* Inter-component Headers */
#include "hal.h"

/* Intra-component Headers */
#include "bldc_driver.h"

/*******************************************************************************************************************************
 * Private Defines
 *******************************************************************************************************************************/

#define MIN_COMMUTATION_PERIOD_US 100U   /**< Minimum time between commutations (10kHz max) */
#define MAX_COMMUTATION_PERIOD_US 50000U /**< Maximum time between commutations (20Hz min) */
#define BEMF_FILTER_ALPHA_MIN 0.0f       /**< Back-EMF filter minimum alpha */
#define BEMF_FILTER_ALPHA_MAX 1.0f       /**< Back-EMF filter maximum alpha */

#define PHASE_A_HIGH_COMMUTATION_IDX 0U /**< Phase A-HI Commutation table index */
#define PHASE_A_LOW_COMMUTATION_IDX 1U  /**< Phase A-LO Commutation table index */
#define PHASE_B_HIGH_COMMUTATION_IDX 2U /**< Phase B-HI Commutation table index */
#define PHASE_B_LOW_COMMUTATION_IDX 3U  /**< Phase B-LO Commutation table index */
#define PHASE_C_HIGH_COMMUTATION_IDX 4U /**< Phase C-HI Commutation table index */
#define PHASE_C_LOW_COMMUTATION_IDX 5U  /**< Phase C-LO Commutation table index */
#define NUM_COMMUTATION_STEPS 6U        /**< Number of commutation steps */

/*******************************************************************************************************************************
 * Private Variables
 *******************************************************************************************************************************/

static struct BLDCData_t s_bldc_data = { 0U };

static uint8_t s_commutation_table[NUM_COMMUTATION_STEPS][NUM_COMMUTATION_STEPS] = {
  /* High_A, Low_A, High_B, Low_B, High_C, Low_C */
  { 1U, 0U, 0U, 1U, 0U, 0U }, /* Step 1: A-High, B-Low */
  { 0U, 1U, 1U, 0U, 0U, 0U }, /* Step 2: B-High, A-Low */
  { 0U, 1U, 0U, 0U, 1U, 0U }, /* Step 3: C-High, A-Low */
  { 0U, 0U, 0U, 1U, 1U, 0U }, /* Step 4: C-High, B-Low */
  { 1U, 0U, 0U, 0U, 0U, 1U }, /* Step 5: A-High, C-Low */
  { 0U, 0U, 1U, 0U, 0U, 1U }  /* Step 6: B-High, C-Low */
};

/*******************************************************************************************************************************
 * Helper Functions
 *******************************************************************************************************************************/

static void set_phase_outputs(const uint8_t commutation[NUM_COMMUTATION_STEPS]) {
  /* Phase A */
  if (commutation[PHASE_A_HIGH_COMMUTATION_IDX] == 1U) {
    hal_pwm_set_duty(MOTOR_PHASE_A, s_bldc_data.pwm_duty);
  } else if (commutation[PHASE_A_LOW_COMMUTATION_IDX] == 1U) {
    hal_gpio_set_phase_low(MOTOR_PHASE_A);
  } else {
    hal_gpio_set_phase_float(MOTOR_PHASE_A);
  }

  /* Phase B */
  if (commutation[PHASE_B_HIGH_COMMUTATION_IDX] == 1U) {
    hal_pwm_set_duty(MOTOR_PHASE_B, s_bldc_data.pwm_duty);
  } else if (commutation[PHASE_B_LOW_COMMUTATION_IDX] == 1U) {
    hal_gpio_set_phase_low(MOTOR_PHASE_B);
  } else {
    hal_gpio_set_phase_float(MOTOR_PHASE_B);
  }

  /* Phase C */
  if (commutation[PHASE_C_HIGH_COMMUTATION_IDX] == 1U) {
    hal_pwm_set_duty(MOTOR_PHASE_C, s_bldc_data.pwm_duty);
  } else if (commutation[PHASE_C_LOW_COMMUTATION_IDX] == 1U) {
    hal_gpio_set_phase_low(MOTOR_PHASE_C);
  } else {
    hal_gpio_set_phase_float(MOTOR_PHASE_C);
  }
}

static uint8_t hall_state_to_commutation_index(uint8_t hall_state) {
  switch (hall_state) {
    default:
      return 0U;
  }
}

static uint8_t determine_floating_phase(uint8_t step) {
  switch (step) {
    case 0U:
      return MOTOR_PHASE_C;
    case 1U:
      return MOTOR_PHASE_C;
    case 2U:
      return MOTOR_PHASE_B;
    case 3U:
      return MOTOR_PHASE_A;
    case 4U:
      return MOTOR_PHASE_B;
    case 5U:
      return MOTOR_PHASE_A;
    default:
      return MOTOR_PHASE_C;
  }
}

static bool has_zero_crossed(float bemf_value, float threshold, ZeroCrossingState_t zc_state) {
  /* If the back-emf value has risen past the Back-EMF threshold, and the zero crossing is rising */
  /* Or if the back-emf value has fallen below the Back-EMF threshold, and the zero crossing is
   * falling */
  if (zc_state == ZC_STATE_RISING && bemf_value > threshold) {
    return true;
  } else if (zc_state == ZC_STATE_FALLING && bemf_value < -threshold) {
    return true;
  }

  return false;
}

static ZeroCrossingState_t update_zc_state(ZeroCrossingState_t current_state) {
  /* In trapezoidal commutation of a BLDC motor, the zero crossing flips between rising and falling
   * edge */
  return (current_state == ZC_STATE_RISING) ? ZC_STATE_FALLING : ZC_STATE_RISING;
}

/*******************************************************************************************************************************
 * Private Driver Implementation
 *******************************************************************************************************************************/

static MotorError_t bldc_init(struct Motor_t *motor, struct MotorConfig_t *config) {
  if (motor == NULL || config == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  motor->config = config;

  s_bldc_data.step = 0U;
  s_bldc_data.direction = true;
  s_bldc_data.pwm_duty = 0U;
  s_bldc_data.zc_state = ZC_STATE_RISING;
  s_bldc_data.zc_threshold = 0.1f;
  s_bldc_data.bemf_filter_alpha = 0.1f;
  s_bldc_data.estimated_speed = 0.0f;
  s_bldc_data.commutation_period = MAX_COMMUTATION_PERIOD_US;

  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    s_bldc_data.bemf[phase] = 0.0f;
    s_bldc_data.bemf_filtered[phase] = 0.0f;
  }

  /* Initialize hardware */
  if (!hal_pwm_init(&config->pwm_config) || !hal_adc_init(&config->adc_config) ||
      !hal_gpio_init()) {
    return MOTOR_INIT_ERROR;
  }

  motor->private_data = &s_bldc_data;
  motor->state.is_initialized = true;
  motor->state.last_update_time = hal_get_micros();

  return MOTOR_OK;
}

static MotorError_t bldc_deinit(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  hal_pwm_set_duty(MOTOR_PHASE_A, 0U);
  hal_pwm_set_duty(MOTOR_PHASE_B, 0U);
  hal_pwm_set_duty(MOTOR_PHASE_C, 0U);

  hal_gpio_set_phase_float(MOTOR_PHASE_A);
  hal_gpio_set_phase_float(MOTOR_PHASE_B);
  hal_gpio_set_phase_float(MOTOR_PHASE_C);

  motor->state.is_initialized = false;

  return MOTOR_OK;
}

static MotorError_t bldc_update_state(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDCData_t *bldc = (struct BLDCData_t *)motor->private_data;

  /* This shall block the runnig thread until conversion is completed */
  hal_adc_start_conversion();

  hal_adc_get_phase_voltages(motor->state.phase_voltages);
  hal_adc_get_phase_currents(motor->state.phase_currents);
  motor->state.temperature = hal_adc_get_temperature();
  motor->state.dc_voltage = hal_adc_get_dc_voltage();

  uint32_t current_time = hal_get_micros();
  float delta_time = (current_time - motor->state.last_update_time) / 1000000.0f;
  motor->state.last_update_time = current_time;

  /* Check for overvoltage or undervoltage */
  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    if (motor->state.phase_voltages[phase] > motor->config->max_voltage) {
      return MOTOR_OVERVOLTAGE_ERROR;
    } else if (motor->state.phase_currents[phase] > motor->config->max_current) {
      return MOTOR_OVERCURRENT_ERROR;
    }
  }

  /* Update Back-EMF values */

  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    bldc->bemf[phase] = motor->state.phase_voltages[phase];

    /* Low-pass IIR filter on filtered Back-EMF */
    bldc->bemf_filtered[phase] = (bldc->bemf_filter_alpha * bldc->bemf[phase]) +
                                 ((1.0f - bldc->bemf_filter_alpha) * bldc->bemf_filtered[phase]);
  }

  switch (motor->control_mode) {
    case CONTROL_MODE_CURRENT:
      /* In current control mode, */
      bldc->pwm_duty =
          pid_update(&motor->control.current, motor->setpoint.current,
                     motor->state.phase_currents[determine_floating_phase(bldc->step)], delta_time);
      break;
    case CONTROL_MODE_VELOCITY:
      /* In velocity control mode, */
      bldc->pwm_duty = pid_update(&motor->control.velocity, motor->setpoint.velocity,
                                  motor->state.velocity, delta_time);
      break;
    default:
      break;
  }

  return MOTOR_OK;
}

static MotorError_t bldc_commutate(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDCData_t *bldc = (struct BLDCData_t *)motor->private_data;

  switch (motor->config->control_method) {
    case CONTROL_METHOD_SENSORLESS:
      /* Sensorless Commutation:
          1. Identify the floating phase for the current commutation step
          2. Check if the filtered back-EMF signal crosses the zero crossing threshold
          3. If so, update the commutation step with delay compensation (Based on estimated rotor
         speed)
      */
      float bemf_value = bldc->bemf_filtered[determine_floating_phase(bldc->step)];

      if (has_zero_crossed(bemf_value, bldc->zc_threshold, bldc->zc_state)) {
        if (bldc->direction) {
          bldc->step = (bldc->step + 1U) % NUM_COMMUTATION_STEPS;
        } else {
          bldc->step = (bldc->step + NUM_COMMUTATION_STEPS - 1U) % NUM_COMMUTATION_STEPS;
        }

        set_phase_outputs(s_commutation_table[bldc->step]);

        bldc->last_zc_time = hal_get_micros();
        bldc->zc_state = update_zc_state(bldc->zc_state);
      }

      break;
    default:
      /* Sensored Commutation:
          1. Read the hall sensors and map readings directly to a commutation step
      */
  }
  return MOTOR_OK;
}

static MotorError_t bldc_update_pwm(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDCData_t *bldc = (struct BLDCData_t *)motor->private_data;
  set_phase_outputs(s_commutation_table[bldc->step]);

  return MOTOR_OK;
}

static MotorError_t bldc_set_voltage(struct Motor_t *motor, float voltage) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  if (voltage > motor->config->max_voltage) {
    motor->setpoint.voltage = motor->config->max_voltage;
  } else {
    motor->setpoint.voltage = voltage;
  }
}

static MotorError_t bldc_set_current(struct Motor_t *motor, float current) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  if (current > motor->config->max_current) {
    motor->setpoint.current = motor->config->max_current;
  } else {
    motor->setpoint.current = current;
  }
}

static MotorError_t bldc_set_velocity(struct Motor_t *motor, float velocity) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  if (velocity > motor->config->max_velocity) {
    motor->setpoint.velocity = motor->config->max_velocity;
  } else {
    motor->setpoint.velocity = velocity;
  }
}

static MotorError_t bldc_set_position(struct Motor_t *motor, float position) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.position = position;
}

static MotorError_t bldc_set_torque(struct Motor_t *motor, float torque) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.torque = torque;
}

/*******************************************************************************************************************************
 * Global Driver Implementation
 *******************************************************************************************************************************/

void bldc_create_driver(struct Motor_t *motor) {
  if (motor != NULL) {
    motor->driver.init = bldc_init;
    motor->driver.deinit = bldc_deinit;
    motor->driver.update_state = bldc_update_state;
    motor->driver.commutate = bldc_commutate;
    motor->driver.update_pwm = bldc_update_pwm;
    motor->driver.set_voltage = bldc_set_voltage;
    motor->driver.set_current = bldc_set_current;
    motor->driver.set_velocity = bldc_set_velocity;
    motor->driver.set_position = bldc_set_position;
    motor->driver.set_torque = bldc_set_torque;
  }
}
