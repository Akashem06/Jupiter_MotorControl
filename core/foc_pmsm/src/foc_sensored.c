/*******************************************************************************************************************************
 * @file   foc_sensored.c
 *
 * @brief  Source file for the sensored FOC driver
 *
 * @date   2025-06-19
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <math.h>
#include <stddef.h>

/* Inter-component Headers */
#include "foc_common.h"
#include "hal.h"
#include "math_utils.h"

/* Intra-component Headers */
#include "foc_sensored.h"

/*******************************************************************************************************************************
 * Private Data Structure
 *******************************************************************************************************************************/

static struct FOCSensoredData_t s_foc_data = {
  .current_d_pid_config = {
    .kd                   = FOC_PID_DEFAULT_D_KP,
    .ki                   = FOC_PID_DEFAULT_D_KI,
    .kp                   = FOC_PID_DEFAULT_D_KD,
    .output_max           = FOC_PID_DEFAULT_D_OUTPUT_MAX,
    .output_min           = FOC_PID_DEFAULT_D_OUTPUT_MIN,
    .derivative_ema_alpha = FOC_PID_DEFAULT_D_DERIV_EMA_ALPHA,
  },

  .current_q_pid_config = {
    .kd                   = FOC_PID_DEFAULT_Q_KP,
    .ki                   = FOC_PID_DEFAULT_Q_KI,
    .kp                   = FOC_PID_DEFAULT_Q_KD,
    .output_max           = FOC_PID_DEFAULT_Q_OUTPUT_MAX,
    .output_min           = FOC_PID_DEFAULT_Q_OUTPUT_MIN,
    .derivative_ema_alpha = FOC_PID_DEFAULT_Q_DERIV_EMA_ALPHA,
  },
};

/*******************************************************************************************************************************
 * Interface Functions
 *******************************************************************************************************************************/

static MotorError_t foc_sensored_init(struct Motor_t *motor, struct MotorConfig_t *config) {
  if (motor == NULL || config == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  motor->config = config;
  motor->private_data = &s_foc_data;

  /* Initialize pid controllers */
  pid_init(&motor->control.current, &motor->config->current_pid_config);
  pid_init(&motor->control.velocity, &motor->config->velocity_pid_config);

  pid_init(&s_foc_data.current_d, &s_foc_data.current_d_pid_config);
  pid_init(&s_foc_data.current_q, &s_foc_data.current_q_pid_config);

  if (!hal_pwm_init(&config->pwm_config) || !hal_adc_init(&config->adc_config) || !hal_encoder_init()) {
    return MOTOR_INIT_ERROR;
  }

  motor->state.is_initialized = true;
  return MOTOR_OK;
}

static MotorError_t foc_sensored_deinit(struct Motor_t *motor) {
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

static MotorError_t foc_sensored_update_state(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct FOCSensoredData_t *foc_data = (struct FOCSensoredData_t *)motor->private_data;

  hal_adc_start_conversion();
  hal_adc_get_phase_voltages(motor->state.phase_voltages);
  hal_adc_get_phase_currents(motor->state.phase_currents);
  motor->state.temperature = hal_adc_get_temperature();
  motor->state.dc_voltage = hal_adc_get_dc_voltage();

  motor->state.position = hal_encoder_get_position();
  motor->state.velocity = hal_encoder_get_velocity();

  /* Check for overvoltage or undervoltage */
  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    if (motor->state.phase_voltages[phase] > motor->config->max_voltage) {
      foc_data->mode = MOTOR_MODE_ERROR;
      return MOTOR_OVERVOLTAGE_ERROR;
    } else if (motor->state.phase_currents[phase] > motor->config->max_current) {
      foc_data->mode = MOTOR_MODE_ERROR;
      return MOTOR_OVERCURRENT_ERROR;
    }
  }

  return MOTOR_OK;
}

static MotorError_t foc_sensored_commutate(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct FOCSensoredData_t *foc_data = (struct FOCSensoredData_t *)motor->private_data;

  uint32_t current_time = hal_get_micros();
  float delta_time = (current_time - motor->state.last_update_time) / 1000000.0f;
  motor->state.last_update_time = current_time;

  /*
   * Step 1: Read phase currents
   */
  float ia = motor->state.phase_currents[MOTOR_PHASE_A];
  float ib = motor->state.phase_currents[MOTOR_PHASE_B];

  /*
   * Step 2: Calculate electrical angle
   */
  float mech_angle = motor->state.position;
  foc_data->electrical_angle = normalize_angle(mech_angle * motor->config->pole_pairs);

  /*
   * Step 3: Clarke transform
   */
  float alpha, beta;
  clarke_transform_2phase(ia, ib, &alpha, &beta);

  /*
   * Step 4: Park transform
   */
  park_transform(alpha, beta, foc_data->electrical_angle, &foc_data->id, &foc_data->iq);

  /*
   * Step 5: Determine VD and VQ (D-axis and Q-axis voltages) based on control method
   * Minimize ID to 0, maximizing motor torque generation
   */
  switch (motor->config->control_mode) {
    case CONTROL_MODE_CURRENT:
    case CONTROL_MODE_TORQUE: {
      float id_ref = 0.0f;
      float iq_ref =
          (motor->config->control_mode == CONTROL_MODE_TORQUE) ? (motor->setpoint.torque / motor->config->torque_constant) : motor->setpoint.current;

      foc_data->vd = pid_update(&foc_data->current_d, id_ref, foc_data->id, delta_time);
      foc_data->vq = pid_update(&foc_data->current_q, iq_ref, foc_data->iq, delta_time);
      break;
    }

    case CONTROL_MODE_VELOCITY: {
      float iq_ref = pid_update(&motor->control.velocity, motor->setpoint.velocity, motor->state.velocity, delta_time);
      float id_ref = 0.0f;

      foc_data->vd = pid_update(&foc_data->current_d, id_ref, foc_data->id, delta_time);
      foc_data->vq = pid_update(&foc_data->current_q, iq_ref, foc_data->iq, delta_time);
      break;
    }

    case CONTROL_MODE_VOLTAGE:
    default:
      foc_data->vd = motor->setpoint.voltage;
      foc_data->vq = 0.0f;
      break;
  }

  /*
   * Step 6: Inverse park transform
   */
  inverse_park_transform(foc_data->vd, foc_data->vq, foc_data->electrical_angle, &alpha, &beta);
  return MOTOR_OK;
}

static MotorError_t foc_sensored_update_pwm(struct Motor_t *motor) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }

  struct FOCSensoredData_t *foc_data = (struct FOCSensoredData_t *)motor->private_data;

  /*
   * Step 7: Space vector modulation generation
   */
  float vref_mag = sqrtf(foc_data->vd * foc_data->vd + foc_data->vq * foc_data->vq);
  float duty_A, duty_B, duty_C;

  svpwm_generate(foc_data->electrical_angle, vref_mag, &duty_A, &duty_B, &duty_C);
  hal_set_pwm(&motor->config->pwm_config, duty_A, duty_B, duty_C);
  return MOTOR_OK;
}

static MotorError_t foc_sensored_set_voltage(struct Motor_t *motor, float voltage) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.voltage = voltage;
  motor->config->control_mode = CONTROL_MODE_VOLTAGE;
  return MOTOR_OK;
}

static MotorError_t foc_sensored_set_current(struct Motor_t *motor, float current) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.current = current;
  motor->config->control_mode = CONTROL_MODE_CURRENT;
  return MOTOR_OK;
}

static MotorError_t foc_sensored_set_velocity(struct Motor_t *motor, float velocity) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.velocity = velocity;
  motor->config->control_mode = CONTROL_MODE_VELOCITY;
  return MOTOR_OK;
}

static MotorError_t foc_sensored_set_position(struct Motor_t *motor, float position) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.position = position;
  motor->config->control_mode = CONTROL_MODE_POSITION;
  return MOTOR_OK;
}

static MotorError_t foc_sensored_set_torque(struct Motor_t *motor, float torque) {
  if (motor == NULL) {
    return MOTOR_INVALID_ARGS;
  }
  motor->setpoint.torque = torque;
  motor->config->control_mode = CONTROL_MODE_TORQUE;
  return MOTOR_OK;
}

/*******************************************************************************************************************************
 * Global Driver Registration
 *******************************************************************************************************************************/

void foc_sensored_create_driver(struct Motor_t *motor) {
  if (motor != NULL) {
    motor->driver.init = foc_sensored_init;
    motor->driver.deinit = foc_sensored_deinit;
    motor->driver.update_state = foc_sensored_update_state;
    motor->driver.commutate = foc_sensored_commutate;
    motor->driver.update_pwm = foc_sensored_update_pwm;
    motor->driver.set_voltage = foc_sensored_set_voltage;
    motor->driver.set_current = foc_sensored_set_current;
    motor->driver.set_velocity = foc_sensored_set_velocity;
    motor->driver.set_position = foc_sensored_set_position;
    motor->driver.set_torque = foc_sensored_set_torque;
  }
}
