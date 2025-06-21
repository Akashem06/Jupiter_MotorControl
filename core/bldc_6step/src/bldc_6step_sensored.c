/*******************************************************************************************************************************
 * @file   bldc_6step_sensored.c
 *
 * @brief  Source file for the sensored BLDC driver
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
#include "bldc_6step_sensored.h"

/*******************************************************************************************************************************
 * Private Variables
 *******************************************************************************************************************************/

static struct BLDC6StepSensoredData_t s_6step_sensored_data = { 0U };

/*******************************************************************************************************************************
 * Helper Functions
 *******************************************************************************************************************************/

static uint8_t _6step_sensored_hall_state_to_commutation_index(uint8_t hall_state, bool direction) {
  if (direction) {
    /*
     * Forward rotation commutation table mapping.
     * Hall state is read as 0bHallA_MSB HallB_MID HallC_LSB.
     * Commutation steps refer to 0-based indices in the 'bldc_6step_commutation_table'.
     */
    switch (hall_state) {
      case 0b011U:
        /*
         * Hall A=L, Hall B=H, Hall C=H (Diagram Step 1)
         * Corresponds to bldc_6step_commutation_table[0]: Phase A-High, Phase B-Low (Phase C-Float)
         */
        return 0U;
      case 0b001U:
        /*
         * Hall A=L, Hall B=L, Hall C=H (Diagram Step 2)
         * Corresponds to bldc_6step_commutation_table[1]: Phase A-High, Phase C-Low (Phase B-Float)
         */
        return 1U;
      case 0b101U:
        /*
         * Hall A=H, Hall B=L, Hall C=H (Diagram Step 3)
         * Corresponds to bldc_6step_commutation_table[2]: Phase B-High, Phase C-Low (Phase A-Float)
         */
        return 2U;
      case 0b100U:
        /*
         * Hall A=H, Hall B=L, Hall C=L (Diagram Step 4)
         * Corresponds to bldc_6step_commutation_table[3]: Phase B-High, Phase A-Low (Phase C-Float)
         */
        return 3U;
      case 0b110U:
        /*
         * Hall A=H, Hall B=H, Hall C=L (Diagram Step 5)
         * Corresponds to bldc_6step_commutation_table[4]: Phase C-High, Phase A-Low (Phase B-Float)
         */
        return 4U;
      case 0b010U:
        /*
         * Hall A=L, Hall B=H, Hall C=L (Diagram Step 6)
         * Corresponds to bldc_6step_commutation_table[5]: Phase C-High, Phase B-Low (Phase A-Float)
         */
        return 5U;
      default:
        return 0xFF;
    }
  } else {
    /*
     * Reverse rotation commutation table mapping.
     * This sequence is the reverse of the forward rotation to maintain continuous motion
     * Hall state is read as 0bHallA_MSB HallB_MID HallC_LSB
     * Commutation steps refer to 0-based indices in the 'bldc_6step_commutation_table'
     */
    switch (hall_state) {
      case 0b011U:
        /*
         * Hall A=L, Hall B=H, Hall C=H (Corresponds to Diagram Step 1 in forward sequence)
         * This is bldc_6step_commutation_table[5]: Phase C-High, Phase B-Low (Phase A-Float)
         */
        return 5U;
      case 0b001U:
        /*
         * Hall A=L, Hall B=L, Hall C=H (Corresponds to Diagram Step 2 in forward sequence)
         * This is bldc_6step_commutation_table[0]: Phase A-High, Phase B-Low (Phase C-Float)
         */
        return 0U;
      case 0b101U:
        /*
         * Hall A=H, Hall B=L, Hall C=H (Corresponds to Diagram Step 3 in forward sequence)
         * This is bldc_6step_commutation_table[1]: Phase A-High, Phase C-Low (Phase B-Float)
         */
        return 1U;
      case 0b100U:
        /*
         * Hall A=H, Hall B=L, Hall C=L (Corresponds to Diagram Step 4 in forward sequence)
         * This is bldc_6step_commutation_table[2]: Phase B-High, Phase C-Low (Phase A-Float)
         */
        return 2U;
      case 0b110U:
        /*
         * Hall A=H, Hall B=H, Hall C=L (Corresponds to Diagram Step 5 in forward sequence)
         * This is bldc_6step_commutation_table[3]: Phase B-High, Phase A-Low (Phase C-Float)
         */
        return 3U;
      case 0b010U:
        /*
         * Hall A=L, Hall B=H, Hall C=L (Corresponds to Diagram Step 6 in forward sequence)
         * This is bldc_6step_commutation_table[4]: Phase C-High, Phase A-Low (Phase B-Float)
         */
        return 4U;
      default:
        return 0xFF;
    }
  }
}

static MotorError_t _6step_sensored_startup_sequence(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDC6StepSensoredData_t *bldc_data = (struct BLDC6StepSensoredData_t *)motor->private_data;

  /* Step 1: Initial alignment phase */
  bldc_data->step = 0U;
  bldc_data->pwm_duty = DEFAULT_STARTUP_DUTY;
  bldc_data->mode = MOTOR_MODE_ALIGNING;
  _6step_bldc_set_phase_outputs(bldc_6step_commutation_table[bldc_data->step], bldc_data->pwm_duty);
  hal_delay_ms(DEFAULT_ALIGNMENT_TIME_MS);

  uint8_t current_hall_state = hal_gpio_get_hall_state();
  uint8_t initial_commutation_step = _6step_sensored_hall_state_to_commutation_index(current_hall_state, bldc_data->direction);

  if (initial_commutation_step == 0xFF) {
    /* Could not determine initial position */
    return MOTOR_INIT_ERROR;
  }

  bldc_data->step = initial_commutation_step;
  _6step_bldc_set_phase_outputs(bldc_6step_commutation_table[bldc_data->step], bldc_data->pwm_duty);
  bldc_data->last_commutation_time = hal_get_micros();
  bldc_data->last_hall_state = current_hall_state;

  motor->state.is_initialized = true;
  bldc_data->mode = MOTOR_MODE_RUNNING;
  return MOTOR_OK;
}

/*******************************************************************************************************************************
 * Interface Functions
 *******************************************************************************************************************************/

static MotorError_t _6step_sensored_init(struct Motor_t *motor, struct MotorConfig_t *config) {
  if (motor == NULL || config == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  motor->config = config;
  motor->private_data = &s_6step_sensored_data;

  s_6step_sensored_data.step = 0U;
  s_6step_sensored_data.direction = true;
  s_6step_sensored_data.pwm_duty = 0U;
  s_6step_sensored_data.last_hall_state = 0U;
  s_6step_sensored_data.last_commutation_time = 0U;
  s_6step_sensored_data.estimated_speed = 0.0f;
  s_6step_sensored_data.mode = MOTOR_MODE_IDLE;

  /* Initialize PID */
  pid_init(&motor->control.current, &motor->config->current_pid_config);
  pid_init(&motor->control.velocity, &motor->config->velocity_pid_config);

  /* Initialize hardware */
  if (!hal_pwm_init(&config->pwm_config) || !hal_adc_init(&config->adc_config) || !hal_gpio_init() || !hal_gpio_init_hall_sensors()) {
    return MOTOR_INIT_ERROR;
  }

  /* Startup sequence */
  MotorError_t startup_res = _6step_sensored_startup_sequence(motor);
  if (startup_res != MOTOR_OK) {
    return startup_res;
  }

  motor->state.is_initialized = true;

  return MOTOR_OK;
}

static MotorError_t _6step_sensored_deinit(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDC6StepSensoredData_t *bldc_data = (struct BLDC6StepSensoredData_t *)motor->private_data;

  _6step_bldc_stop_pwm_output();

  motor->state.is_initialized = false;
  bldc_data->mode = MOTOR_MODE_STOPPED;
  return MOTOR_OK;
}

static MotorError_t _6step_sensored_update_state(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct BLDC6StepSensoredData_t *bldc_data = (struct BLDC6StepSensoredData_t *)motor->private_data;

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
  float delta_time = (float)(current_time - motor->state.last_update_time) / 1000000.0f;
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

  /* Sample hall sensors */
  uint8_t current_hall_state = hal_gpio_get_hall_state();

  /* Calculate an estimated speed if the hall sensor state has changed */
  if (current_hall_state != bldc_data->last_hall_state) {
    if (bldc_data->last_commutation_time != 0) {
      uint32_t commutation_time_diff = current_time - bldc_data->last_commutation_time;
      if (commutation_time_diff > 0) {
        /* 60 degrees * the time it to for comutation * 6 sectors */
        bldc_data->estimated_speed = 60.0f * (1000000.0f / ((float)commutation_time_diff * 6.0f));
      }
    }
    bldc_data->last_commutation_time = current_time;
    bldc_data->last_hall_state = current_hall_state;
  }

  motor->state.velocity = bldc_data->estimated_speed;

  switch (motor->config->control_mode) {
    case CONTROL_MODE_TORQUE:
    case CONTROL_MODE_CURRENT:
      float conducting_current = _6step_bldc_get_conducting_current(motor, bldc_data->step);
      bldc_data->pwm_duty = pid_update(&motor->control.current, motor->setpoint.current, conducting_current, delta_time);
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
      bldc_data->pwm_duty = 0.0f;
      break;
  }

  if (bldc_data->pwm_duty > 1.0f) bldc_data->pwm_duty = 1.0f;
  if (bldc_data->pwm_duty < 0.0f) bldc_data->pwm_duty = 0.0f;

  return MOTOR_OK;
}

static MotorError_t _6step_sensored_commutate(struct Motor_t *motor) {
  struct BLDC6StepSensoredData_t *bldc_data;
  uint8_t current_hall_state;
  uint8_t next_step;

  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  bldc_data = (struct BLDC6StepSensoredData_t *)motor->private_data;

  if (bldc_data->mode != MOTOR_MODE_RUNNING) {
    _6step_bldc_stop_pwm_output();
    return MOTOR_OK;
  }

  current_hall_state = hal_gpio_get_hall_state();

  if (current_hall_state != bldc_data->last_hall_state) {
    next_step = _6step_sensored_hall_state_to_commutation_index(current_hall_state, bldc_data->direction);

    if (next_step == 0xFFU) {
      bldc_data->mode = MOTOR_MODE_ERROR;
      return MOTOR_HAL_ERROR;
    }

    bldc_data->step = next_step;
    _6step_bldc_set_phase_outputs(bldc_6step_commutation_table[bldc_data->step], bldc_data->pwm_duty);

    bldc_data->last_hall_state = current_hall_state;
    bldc_data->last_commutation_time = hal_get_micros();
  }

  return MOTOR_OK;
}

static MotorError_t _6step_sensored_update_pwm(struct Motor_t *motor) {
  struct BLDC6StepSensoredData_t *bldc_data;

  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  bldc_data = (struct BLDC6StepSensoredData_t *)motor->private_data;

  if (bldc_data->mode != MOTOR_MODE_RUNNING) {
    _6step_bldc_stop_pwm_output();
    return MOTOR_OK;
  }

  _6step_bldc_set_phase_outputs(bldc_6step_commutation_table[bldc_data->step], bldc_data->pwm_duty);
  return MOTOR_OK;
}

static MotorError_t _6step_sensored_set_voltage(struct Motor_t *motor, float voltage) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.voltage = fminf(voltage, motor->config->max_voltage);
  motor->config->control_mode = CONTROL_MODE_VOLTAGE;
  return MOTOR_OK;
}

static MotorError_t _6step_sensored_set_current(struct Motor_t *motor, float current) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.current = fminf(current, motor->config->max_current);
  motor->config->control_mode = CONTROL_MODE_CURRENT;
  return MOTOR_OK;
}

static MotorError_t _6step_sensored_set_velocity(struct Motor_t *motor, float velocity) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.velocity = fminf(velocity, motor->config->max_velocity);
  motor->config->control_mode = CONTROL_MODE_VELOCITY;
  return MOTOR_OK;
}

static MotorError_t _6step_sensored_set_position(struct Motor_t *motor, float position) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.position = position;
  motor->config->control_mode = CONTROL_MODE_POSITION;
  return MOTOR_OK;
}

static MotorError_t _6step_sensored_set_torque(struct Motor_t *motor, float torque) {
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

void bldc_6step_sensored_create_driver(struct Motor_t *motor) {
  if (motor != NULL) {
    motor->driver.init = _6step_sensored_init;
    motor->driver.deinit = _6step_sensored_deinit;
    motor->driver.update_state = _6step_sensored_update_state;
    motor->driver.commutate = _6step_sensored_commutate;
    motor->driver.update_pwm = _6step_sensored_update_pwm;
    motor->driver.set_voltage = _6step_sensored_set_voltage;
    motor->driver.set_current = _6step_sensored_set_current;
    motor->driver.set_velocity = _6step_sensored_set_velocity;
    motor->driver.set_position = _6step_sensored_set_position;
    motor->driver.set_torque = _6step_sensored_set_torque;
  }
}