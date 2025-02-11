/*******************************************************************************************************************************
 * @file   bldc_model.c
 *
 * @brief  Source file for the simulation BLDC motor model
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Inter-component Headers */
#include "math_utils.h"

/* Intra-component Headers */
#include "bldc_model.h"

/*******************************************************************************************************************************
 * Private Driver Implementation
 *******************************************************************************************************************************/

static void bldc_calculate_back_emf(struct MotorModel_t *model) {
  float electrical_angle = model->state.electrical_angle;
  float speed = model->state.rotor_speed;
  float ke = model->config->params.ac.back_emf_constant;

  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    /* Electrical angle refers to the position of the rotor within the magnetic field */
    /* The phase angle will be the electrical angle (Relative to the current phase) + the Phase
     * offset */
    float phase_angle = electrical_angle + (phase * ((2.0f / 3.0f) * MATH_PI));

    /* The phase angle will always be contained between 0 and 2PI */
    float normalized_angle = fmodf(phase_angle, 2.0f * MATH_PI);

    /* For trapezoidal commutation, the Back-EMF remains constant for 60 degree intervals */
    /* Polarity flips every 120 degrees. The phase shift accounts for all phase angles */
    if (normalized_angle < MATH_PI / 3.0f) {
      /* 0 degrees - 60 degrees */
      model->state.traits.ac.back_emf[phase] = ke * speed;
    } else if (normalized_angle < 2.0f * MATH_PI / 3.0f) {
      /* 60 degrees - 120 degrees */
      model->state.traits.ac.back_emf[phase] = ke * speed;
    } else if (normalized_angle < MATH_PI) {
      /* 120 degrees - 180 degrees */
      model->state.traits.ac.back_emf[phase] = -ke * speed;
    } else if (normalized_angle < 4.0f * MATH_PI / 3.0f) {
      /* 180 degrees - 240 degrees */
      model->state.traits.ac.back_emf[phase] = -ke * speed;
    } else if (normalized_angle < 5.0f * MATH_PI / 3.0f) {
      /* 240 degrees - 300 degrees */
      model->state.traits.ac.back_emf[phase] = ke * speed;
    } else {
      /* 300 degrees - 360 degrees */
      model->state.traits.ac.back_emf[phase] = ke * speed;
    }
  }
}

static void bldc_calculate_torque(struct MotorModel_t *model, float *torque) {
  /* Power = Torque * Angular velocity */
  /* For BLDC motors, the torque is the sum of all phase contributions, which shall be */
  /* Torque = (Back-EMF * Phase Current) / Angular velocity */
  /* Back-EMF is */
  float total_torque = 0.0f;

  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    /* Back-EMF instead of DC Voltage because it is representative of the 'useful' electromagnetic
     * interaction */
    total_torque += (model->state.phase_currents[phase] * model->state.traits.ac.back_emf[phase]) /
                    model->state.rotor_speed;
  }

  /* Apply mechanical losses */
  total_torque -= model->config->friction_coefficient * model->state.rotor_speed;
  total_torque -= model->state.load_torque;

  *torque = total_torque;
}

static void bldc_update_electrical(struct MotorModel_t *model, float dt) {
  /* Calculate back-EMF for current rotor position */
  bldc_calculate_back_emf(model);

  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    float voltage = model->state.phase_voltages[phase];
    float current = model->state.phase_currents[phase];
    float back_emf = model->state.traits.ac.back_emf[phase];

    /* di/dt = Voltage / Inductance, where voltage is the sum of the ohmic loss, Back-emf loss and
     * phase voltage */
    float di_dt = (voltage - (current * model->config->phase_resistance) - back_emf) /
                  model->config->phase_inductance;

    /* Integrate current */
    model->state.phase_currents[phase] += di_dt * dt;

    /* Apply current limiting */
    if (model->state.phase_currents[phase] > model->config->max_current) {
      model->state.phase_currents[phase] = model->config->max_current;
    } else if (model->state.phase_currents[phase] < -model->config->max_current) {
      model->state.phase_currents[phase] = -model->config->max_current;
    }
  }
}

/*******************************************************************************************************************************
 * Global Model Implementation
 *******************************************************************************************************************************/

void bldc_create_model(struct MotorModel_t *model) {
  if (model != NULL) {
    model->calculate_back_emf = bldc_calculate_back_emf;
    model->calculate_torque = bldc_calculate_torque;
    model->update_electrical = bldc_update_electrical;
  }
}
