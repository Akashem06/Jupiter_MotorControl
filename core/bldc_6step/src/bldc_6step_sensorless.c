/*******************************************************************************************************************************
 * @file   bldc_6step_sensorless.c
 *
 * @brief  Source file for the sensorless BLDC driver
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stddef.h>
#include <stdio.h>

/* Inter-component Headers */
#include "hal.h"
#include "math_utils.h"

/* Intra-component Headers */
#include "bldc_6step_sensorless.h"

/*******************************************************************************************************************************
 * Private Defines
 *******************************************************************************************************************************/

#define MIN_COMMUTATION_PERIOD_US 10U    /**< Minimum time between commutations (100kHz max) */
#define MAX_COMMUTATION_PERIOD_US 50000U /**< Maximum time between commutations (20Hz min) */
#define BEMF_FILTER_ALPHA_MIN 0.0f       /**< Back-EMF filter minimum alpha */
#define BEMF_FILTER_ALPHA_MAX 1.0f       /**< Back-EMF filter maximum alpha */
#define ZERO_CROSSING_HYSTERSIS 0.5f     /**< Hysteresis value for zero crossing in Volts(V) */

/*******************************************************************************************************************************
 * Private Variables
 *******************************************************************************************************************************/

static struct BLDC6StepSensorlessData_t s_6step_sensorless_data = { 0U };

static const float s_startup_acceleration_lookup[DEFAULT_STARTUP_STEPS] = {
  1.0000f, /**< powf(0.8, 0) */
  0.8000f, /**< powf(0.8, 1) */
  0.6400f, /**< powf(0.8, 2) */
  0.5120f, /**< powf(0.8, 3) */
  0.4096f, /**< powf(0.8, 4) */
  0.3277f, /**< powf(0.8, 5) */
  0.2621f, /**< powf(0.8, 6) */
  0.2097f, /**< powf(0.8, 7) */
  0.1678f, /**< powf(0.8, 8) */
  0.1342f, /**< powf(0.8, 9) */
  0.1074f, /**< powf(0.8, 10) */
  0.0859f  /**< powf(0.8, 11) */
};

/*******************************************************************************************************************************
 * Helper Functions
 *******************************************************************************************************************************/

static bool _6step_sensorless_has_zero_crossed(float bemf_value, float threshold, ZeroCrossingState_t zc_state) {
  /* If the back-emf value has risen past the Back-EMF threshold, and the zero crossing is rising */
  /* Or if the back-emf value has fallen below the Back-EMF threshold, and the zero crossing is
   * falling */
  if (zc_state == ZC_STATE_RISING && bemf_value > (threshold + ZERO_CROSSING_HYSTERSIS)) {
    return true;
  } else if (zc_state == ZC_STATE_FALLING && bemf_value < -(threshold + ZERO_CROSSING_HYSTERSIS)) {
    return true;
  }
  return false;
}

static ZeroCrossingState_t _6step_sensorless_update_zc_state(ZeroCrossingState_t current_state) {
  /* In trapezoidal commutation of a BLDC motor, the zero crossing flips between rising and falling
   * edge */
  return (current_state == ZC_STATE_RISING) ? ZC_STATE_FALLING : ZC_STATE_RISING;
}

static uint32_t _6step_sensorless_calculate_startup_period(struct Motor_t *motor, uint8_t step) {
  if (motor == NULL || step >= DEFAULT_STARTUP_STEPS) {
    return STARTUP_MIN_PERIOD_US;
  }

  /* Calculate period with exponential decrease (faster acceleration) */
  float exponent = s_startup_acceleration_lookup[step];

  /* Interpolate between min and max period */
  uint32_t period = (uint32_t)(STARTUP_MIN_PERIOD_US * exponent);

  /* Ensure we don't exceed the limits */
  if (period < STARTUP_MAX_PERIOD_US) {
    period = STARTUP_MAX_PERIOD_US;
  }

  return period;
}

static float _6step_sensorless_calculate_startup_duty(struct Motor_t *motor, uint8_t step) {
  float duty = DEFAULT_STARTUP_DUTY + (step * STARTUP_DUTY_INCREMENT_PER_STEP);

  /* Cap at max duty cycle */
  if (duty > motor->config->current_pid_config.output_max) {
    duty = motor->config->current_pid_config.output_max;
  } else if (duty < motor->config->current_pid_config.output_min) {
    duty = motor->config->current_pid_config.output_min;
  }

  return duty;
}

static MotorError_t _6step_sensorless_startup_sequence(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDC6StepSensorlessData_t *bldc_data = (struct BLDC6StepSensorlessData_t *)motor->private_data;
  uint32_t start_time = hal_get_micros();

  /* Step 1: Initial alignment phase */
  bldc_data->mode = MOTOR_MODE_ALIGNING;
  bldc_data->step = 0;
  bldc_data->pwm_duty = DEFAULT_STARTUP_DUTY;
  _6step_bldc_set_phase_outputs(bldc_6step_commutation_table[bldc_data->step], bldc_data->pwm_duty);
  hal_delay_ms(DEFAULT_ALIGNMENT_TIME_MS);

  /* Step 2: Open-loop acceleration phase */
  bldc_data->mode = MOTOR_MODE_OPEN_LOOP;
  for (uint8_t i = 0; i < DEFAULT_STARTUP_STEPS; i++) {
    /* Calculate next commutation time and duty based on acceleration */
    uint32_t commutation_period = _6step_sensorless_calculate_startup_period(motor, i);
    bldc_data->pwm_duty = _6step_sensorless_calculate_startup_duty(motor, i);

    /* Next commutation step */
    bldc_data->step = (bldc_data->step + (bldc_data->direction ? 1 : NUM_COMMUTATION_STEPS - 1)) % NUM_COMMUTATION_STEPS;
    _6step_bldc_set_phase_outputs(bldc_6step_commutation_table[bldc_data->step], bldc_data->pwm_duty);

    /* Wait for calculated time */
    hal_delay_us(commutation_period);

    /* Check for stall condition */
    if ((hal_get_micros() - start_time) > (MAX_STALL_TIME_MS * 1000U)) {
      if (bldc_data->estimated_speed < motor->config->min_startup_speed) {
        // bldc_emergency_stop(motor); // TODO
        bldc_data->mode = MOTOR_MODE_ERROR;
        return MOTOR_INIT_ERROR;
      }
    }
  }

  /* Step 3: Transition to closed-loop */
  bldc_data->mode = MOTOR_MODE_TRANSITION;

  /* Pre-calculate expected zero-crossing time based on last open-loop timing */
  bldc_data->last_zc_time = hal_get_micros();
  bldc_data->commutation_period = _6step_sensorless_calculate_startup_period(motor, DEFAULT_STARTUP_STEPS - 1);

  /* Initialize BEMF filters with expected values to ensure smoother transition */
  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    bldc_data->bemf_filtered[phase] = 0.0f;
  }

  /* Now fully running in closed-loop mode */
  bldc_data->mode = MOTOR_MODE_RUNNING;

  return MOTOR_OK;
}

/*******************************************************************************************************************************
 * Interface Functions
 *******************************************************************************************************************************/

static MotorError_t _6step_sensorless_init(struct Motor_t *motor, struct MotorConfig_t *config) {
  if (motor == NULL || config == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  motor->config = config;
  motor->private_data = &s_6step_sensorless_data;

  s_6step_sensorless_data.step = 0U;
  s_6step_sensorless_data.direction = true;
  s_6step_sensorless_data.pwm_duty = 0U;
  s_6step_sensorless_data.zc_state = ZC_STATE_RISING;
  s_6step_sensorless_data.zc_threshold = 0.1f;
  s_6step_sensorless_data.bemf_filter_alpha = 0.1f;
  s_6step_sensorless_data.estimated_speed = 0.0f;
  s_6step_sensorless_data.commutation_period = MAX_COMMUTATION_PERIOD_US;
  s_6step_sensorless_data.mode = MOTOR_MODE_IDLE;

  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    s_6step_sensorless_data.bemf[phase] = 0.0f;
    s_6step_sensorless_data.bemf_filtered[phase] = 0.0f;
  }

  /* Initialize PID */
  pid_init(&motor->control.current, &motor->config->current_pid_config);
  pid_init(&motor->control.velocity, &motor->config->velocity_pid_config);

  /* Initialize hardware */
  if (!hal_pwm_init(&config->pwm_config) || !hal_adc_init(&config->adc_config) || !hal_gpio_init()) {
    return MOTOR_INIT_ERROR;
  }

  /* Startup sequence */
  MotorError_t startup_res = _6step_sensorless_startup_sequence(motor);
  if (startup_res != MOTOR_OK) {
    return startup_res;
  }

  motor->state.is_initialized = true;

  return MOTOR_OK;
}

static MotorError_t _6step_sensorless_deinit(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDC6StepSensorlessData_t *bldc_data = (struct BLDC6StepSensorlessData_t *)motor->private_data;

  _6step_bldc_stop_pwm_output();

  motor->state.is_initialized = false;
  bldc_data->mode = MOTOR_MODE_STOPPED;
  return MOTOR_OK;
}

static MotorError_t _6step_sensorless_update_state(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDC6StepSensorlessData_t *bldc_data = (struct BLDC6StepSensorlessData_t *)motor->private_data;

  if (bldc_data->mode == MOTOR_MODE_STOPPED || bldc_data->mode == MOTOR_MODE_ERROR) {
    bldc_data->pwm_duty = 0.0f;

    _6step_bldc_stop_pwm_output();
    return MOTOR_OK;
  }

  /* This shall block the running thread until conversion is completed */
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
      bldc_data->mode = MOTOR_MODE_ERROR;
      return MOTOR_OVERVOLTAGE_ERROR;
    } else if (motor->state.phase_currents[phase] > motor->config->max_current) {
      bldc_data->mode = MOTOR_MODE_ERROR;
      return MOTOR_OVERCURRENT_ERROR;
    }
  }

  MotorPhase_t floating_phase = _6step_bldc_determine_floating_phase(bldc_data->step);

  /* Sample BEMF on the floating phase */
  bldc_data->bemf[floating_phase] = motor->state.phase_voltages[floating_phase];

  /* Apply low-pass filter */
  bldc_data->bemf_filtered[floating_phase] = (bldc_data->bemf_filter_alpha * bldc_data->bemf[floating_phase]) +
                                             ((1.0f - bldc_data->bemf_filter_alpha) * bldc_data->bemf_filtered[floating_phase]);

  switch (motor->config->control_mode) {
    case CONTROL_MODE_TORQUE:
    case CONTROL_MODE_CURRENT:
      bldc_data->pwm_duty = pid_update(&motor->control.current, motor->setpoint.current, motor->state.phase_currents[floating_phase], delta_time);
      break;
    case CONTROL_MODE_VELOCITY:
      bldc_data->pwm_duty = pid_update(&motor->control.velocity, motor->setpoint.velocity, bldc_data->estimated_speed, delta_time);
      break;
    case CONTROL_MODE_VOLTAGE:
      bldc_data->pwm_duty = motor->setpoint.voltage / motor->config->max_voltage;
      break;
    case CONTROL_MODE_POSITION:
      // TODO: Complete position control
      break;
    default:
      break;
  }

  if (bldc_data->pwm_duty > 1.0f) bldc_data->pwm_duty = 1.0f;
  if (bldc_data->pwm_duty < 0.0f) bldc_data->pwm_duty = 0.0f;

  return MOTOR_OK;
}

static MotorError_t _6step_sensorless_commutate(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDC6StepSensorlessData_t *bldc_data = (struct BLDC6StepSensorlessData_t *)motor->private_data;

  if (bldc_data->mode != MOTOR_MODE_RUNNING) {
    _6step_bldc_stop_pwm_output();
    return MOTOR_OK;
  }

  uint32_t current_time = hal_get_micros();
  float bemf_value = bldc_data->bemf_filtered[_6step_bldc_determine_floating_phase(bldc_data->step)];

  /**
   * Only check for zero crossing if enough time has passed since last commutation
   * This prevents very high-frequency false zero-crossings at high speed or noise
   */
  if ((current_time - bldc_data->last_zc_time) >= MIN_COMMUTATION_PERIOD_US) {
    if (_6step_sensorless_has_zero_crossed(bemf_value, bldc_data->zc_threshold, bldc_data->zc_state)) {
      bldc_data->commutation_period = current_time - bldc_data->last_zc_time;

      /* RPM = Revolutions / Minute */
      /* Commutation period is time for 1/6th of a full revolution, thus RPM = 1/6 * 1/T_us * 60 *
       * 10^6 */
      bldc_data->estimated_speed = 60.0f * (1000000.0f / ((float)bldc_data->commutation_period * 6.0f));
      motor->state.velocity = bldc_data->estimated_speed;

      // printf("Zero cross %d, Period: %lu us, Speed: %.2f RPM\n", bldc_data->step,
      // bldc_data->commutation_period, bldc_data->estimated_speed);

      if (bldc_data->direction) {
        bldc_data->step = (bldc_data->step + 1U) % NUM_COMMUTATION_STEPS;
      } else {
        bldc_data->step = (bldc_data->step + NUM_COMMUTATION_STEPS - 1U) % NUM_COMMUTATION_STEPS;
      }

      _6step_bldc_set_phase_outputs(bldc_6step_commutation_table[bldc_data->step], bldc_data->pwm_duty);

      bldc_data->last_zc_time = current_time;
      bldc_data->zc_state = _6step_sensorless_update_zc_state(bldc_data->zc_state);
    }
  }

  return MOTOR_OK;
}

static MotorError_t _6step_sensorless_update_pwm(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDC6StepSensorlessData_t *bldc_data = (struct BLDC6StepSensorlessData_t *)motor->private_data;

  if (bldc_data->mode != MOTOR_MODE_RUNNING) {
    _6step_bldc_stop_pwm_output();
    return MOTOR_OK;
  }

  _6step_bldc_set_phase_outputs(bldc_6step_commutation_table[bldc_data->step], bldc_data->pwm_duty);

  return MOTOR_OK;
}

static MotorError_t _6step_sensorless_set_voltage(struct Motor_t *motor, float voltage) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.voltage = fminf(voltage, motor->config->max_voltage);
  motor->config->control_mode = CONTROL_MODE_VOLTAGE;
  return MOTOR_OK;
}

static MotorError_t _6step_sensorless_set_current(struct Motor_t *motor, float current) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.current = fminf(current, motor->config->max_current);
  motor->config->control_mode = CONTROL_MODE_CURRENT;
  return MOTOR_OK;
}

static MotorError_t _6step_sensorless_set_velocity(struct Motor_t *motor, float velocity) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.velocity = fminf(velocity, motor->config->max_velocity);
  motor->config->control_mode = CONTROL_MODE_VELOCITY;
  return MOTOR_OK;
}

static MotorError_t _6step_sensorless_set_position(struct Motor_t *motor, float position) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.position = position;
  motor->config->control_mode = CONTROL_MODE_POSITION;
  return MOTOR_OK;
}

static MotorError_t _6step_sensorless_set_torque(struct Motor_t *motor, float torque) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.torque = torque;
  motor->setpoint.current = motor->setpoint.torque / motor->config->torque_constant;
  motor->config->control_mode = CONTROL_MODE_TORQUE;
  return MOTOR_OK;
}

/*******************************************************************************************************************************
 * Global Driver Implementation
 *******************************************************************************************************************************/

void bldc_6step_sensorless_create_driver(struct Motor_t *motor) {
  if (motor != NULL) {
    motor->driver.init = _6step_sensorless_init;
    motor->driver.deinit = _6step_sensorless_deinit;
    motor->driver.update_state = _6step_sensorless_update_state;
    motor->driver.commutate = _6step_sensorless_commutate;
    motor->driver.update_pwm = _6step_sensorless_update_pwm;
    motor->driver.set_voltage = _6step_sensorless_set_voltage;
    motor->driver.set_current = _6step_sensorless_set_current;
    motor->driver.set_velocity = _6step_sensorless_set_velocity;
    motor->driver.set_position = _6step_sensorless_set_position;
    motor->driver.set_torque = _6step_sensorless_set_torque;
  }
}