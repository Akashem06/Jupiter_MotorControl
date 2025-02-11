/*******************************************************************************************************************************
 * @file   sim_manager.c
 *
 * @brief  Source file for the simulation manager
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdio.h>
#include <string.h>

/* Inter-component Headers */
#include "bldc_driver.h"
#include "motor.h"

/* Intra-component Headers */
#include "sim_manager.h"

bool sim_manager_init(struct SimManager_t *sim) {
  if (sim == NULL) {
    return false;
  }

  sim->motor_config = (struct MotorConfig_t){ .type = MOTOR_TYPE_BLDC,
                                              .control_method = CONTROL_METHOD_SENSORLESS,
                                              .pole_pairs = 7U,
                                              .phase_resistance = 0.1f,
                                              .phase_inductance = 0.0001f,
                                              .max_current = 20.0f,
                                              .max_voltage = 24.0f,
                                              .max_velocity = 1000.0f,
                                              .pwm_config = { .frequency = 20000,
                                                              .dead_time_ns = 1000,
                                                              .resolution = 12,
                                                              .complementary_output = true },
                                              .adc_config = { .sampling_freq = 20000,
                                                              .resolution = 12,
                                                              .v_ref = 3.3f,
                                                              .current_gain = 0.1f,
                                                              .voltage_gain = 0.1f } };

  sim->model_config = (struct MotorModelConfig_t){
    .pole_pairs = 7U,
    .phase_inductance = 0.0001f,
    .phase_resistance = 0.1f,
    .dc_voltage = 24.0f,
    .max_current = 20.0f,
    .max_voltage = 24.0f,
    .moment_of_inertia = 0.0001f,
    .friction_coefficient = 0.0001f,
    .params.ac.back_emf_constant = 0.01f,
    .params.ac.torque_constant = 0.01f,
  };

  motor_model_init(&sim->model, &sim->model_config);

  memset(&sim->motor, 0, sizeof(struct Motor_t));

  bldc_create_driver(&sim->motor);

  if (sim->motor.driver.init(&sim->motor, &sim->motor_config) != MOTOR_OK) {
    printf("Motor initialization failed!\n");
    return false;
  }

  hal_sim_set_model(&sim->model);

  sim->is_running = false;
  sim->sim_time_us = 0;
  sim->last_print_time = 0;

  return true;
}

void sim_manager_stop(struct SimManager_t *sim) {
  if (sim != NULL) {
    sim->is_running = false;
  }
}

void print_motor_info(struct Motor_t *motor) {
  printf("Motor Info:\n");
  printf("-------------------------------------------------\n");
  printf("  Type: %d\n", motor->config->type);
  printf("  Control Method: %d\n", motor->config->control_method);
  printf("  Pole Pairs: %d\n", motor->config->pole_pairs);
  printf("  Phase Resistance: %.3f Ohms\n", motor->config->phase_resistance);
  printf("  Phase Inductance: %.6f H\n", motor->config->phase_inductance);
  printf("  Max Current: %.2f A\n", motor->config->max_current);
  printf("  Max Voltage: %.2f V\n", motor->config->max_voltage);
  printf("  Max Velocity: %.2f rad/s\n", motor->config->max_velocity);

  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    printf("  Motor phase %c voltage: %.3f", (char)(phase + 'A'),
           motor->state.phase_voltages[phase]);
  }
  printf("\n");
  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    printf("  Motor phase %c current: %.3f", (char)(phase + 'A'),
           motor->state.phase_currents[phase]);
  }
  printf("\n");
}

void print_motor_model_info(struct MotorModel_t *model) {
  printf("Motor Model Info:\n");
  printf("-------------------------------------------------\n");
  printf("  Type: %d\n", model->config->type);
  printf("  Moment of Inertia: %.6f kg*m^2\n", model->config->moment_of_inertia);
  printf("  Friction Coefficient: %.6f N*m*s\n", model->config->friction_coefficient);
  printf("  DC Voltage: %.2f V\n", model->config->dc_voltage);
  printf("  Pole Pairs: %d\n", model->config->pole_pairs);
  printf("  Phase Resistance: %.3f Ohms\n", model->config->phase_resistance);
  printf("  Phase Inductance: %.6f H\n", model->config->phase_inductance);
  printf("  Max Current: %.2f A\n", model->config->max_current);
  printf("  Max Voltage: %.2f V\n", model->config->max_voltage);

  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    printf("  Motor phase %c voltage: %.3f", (char)(phase + 'A'),
           model->state.phase_voltages[phase]);
  }
  printf("\n");
  for (MotorPhase_t phase = MOTOR_PHASE_A; phase < NUM_MOTOR_PHASES; phase++) {
    printf("  Motor phase %c current: %.3f", (char)(phase + 'A'),
           model->state.phase_currents[phase]);
  }
  printf("\n");
}

void sim_manager_run(struct SimManager_t *sim) {
  if (sim == NULL) {
    return;
  }

  sim->is_running = true;

  while (sim->is_running) {
    /* Run motor control loop */
    motor_run(&sim->motor);

    /* Update simulation time */
    sim->sim_time_us = hal_get_micros();

    /* Print status every 100ms */
    if (sim->sim_time_us - sim->last_print_time >= 100000) {
      print_motor_model_info(&sim->model);
      print_motor_info(&sim->motor);
      fflush(stdout);
      sim->last_print_time = sim->sim_time_us;
    }
  }
}
