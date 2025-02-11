/*******************************************************************************************************************************
 * @file   motor_model.c
 *
 * @brief  Source file for the simulation motor model
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Inter-component Headers */
#include "math_utils.h"
#include "motor.h"

/* Intra-component Headers */
#include "bldc_model.h"
#include "motor_model.h"

/*******************************************************************************************************************************
 * Global Model Implementation
 *******************************************************************************************************************************/

void motor_model_init(struct MotorModel_t *model, struct MotorModelConfig_t *config) {
  if (model != NULL && config != NULL) {
    model->config = config;
    memset(&model->state, 0U, sizeof(struct MotorModelState_t));

    switch (config->type) {
      case MOTOR_TYPE_BLDC:
        bldc_create_model(model);
        break;
      case MOTOR_TYPE_PMSM:
        break;
      case MOTOR_TYPE_STEPPER:
        break;
      default:
        break;
    }
  }
}

void motor_model_step(struct MotorModel_t *model, float dt) {
  if (model != NULL && dt > 0.0f) {
    model->update_electrical(model, dt);

    float torque;
    model->calculate_torque(model, &torque);

    /* Moment of Inertia * dw_dt = torque */
    float dw_dt = torque / model->config->moment_of_inertia;

    /* Angular velocity is integrated */
    model->state.rotor_speed += dw_dt * dt;

    /* new angle = Old angle + Angular Velocity * time */
    model->state.rotor_angle += model->state.rotor_speed * dt;
    /* Normalize from [0, 2PI] */
    model->state.rotor_angle = fmodf(model->state.rotor_angle, 2.0f * MATH_PI);

    /* Calculate the electrical angle based on the mechanical angle and number of pole-pairs */
    model->state.electrical_angle = model->state.rotor_angle * model->config->pole_pairs;
  }
}

void motor_model_set_pwm_duty(struct MotorModel_t *model, float duty) {
  if (model != NULL) {
    /* Convert PWM duty cycle to voltage */
    float voltage = duty * model->config->dc_voltage;

    /* Apply to all phases */
    for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
      motor_model_set_phase_voltage(model, phase, voltage);
    }
  }
}

void motor_model_set_phase_voltage(struct MotorModel_t *model, uint8_t phase, float voltage) {
  if (model != NULL && phase < NUM_MOTOR_PHASES) {
    if (voltage > model->config->dc_voltage) {
      voltage = model->config->dc_voltage;
    } else if (voltage < -model->config->dc_voltage) {
      voltage = -model->config->dc_voltage;
    }

    model->state.phase_voltages[phase] = voltage;
  }
}

void motor_model_set_phase_float(struct MotorModel_t *model, uint8_t phase) {
  if (model != NULL && phase < NUM_MOTOR_PHASES) {
    /* When floating, the phase voltage will be determined by back-EMF. We set it to half the DC bus
     * voltage as a guess */
    model->state.phase_voltages[phase] = model->config->dc_voltage / 2.0f;
  }
}

void motor_model_get_phase_voltages(struct MotorModel_t *model, float *voltages) {
  if (model != NULL && voltages != NULL) {
    memcpy(voltages, model->state.phase_voltages, sizeof(float) * NUM_MOTOR_PHASES);
  }
}

void motor_model_get_phase_currents(struct MotorModel_t *model, float *currents) {
  if (model != NULL && currents != NULL) {
    memcpy(currents, model->state.phase_currents, sizeof(float) * NUM_MOTOR_PHASES);
  }
}
