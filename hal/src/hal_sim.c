/*******************************************************************************************************************************
 * @file   hal_sim.c
 *
 * @brief  Simulation HAL with realistic motor model
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "hal.h"

/*******************************************************************************************************************************
 * Simulation Parameters and Constants
 *******************************************************************************************************************************/

#define SIM_MOTOR_POLES 14U               /**< Number of motor poles */
#define SIM_MOTOR_KE 0.1f                 /**< Back-EMF constant (V/rad/s) */
#define SIM_MOTOR_KT 0.1f                 /**< Torque constant (Nm/A) */
#define SIM_MOTOR_RESISTANCE 0.5f         /**< Phase resistance (Ohm) */
#define SIM_MOTOR_INDUCTANCE 0.001f       /**< Phase inductance (H) */
#define SIM_MOTOR_INERTIA 0.0001f         /**< Rotor inertia (kg⋅m²) */
#define SIM_MOTOR_FRICTION 0.00001f       /**< Friction coefficient (Nm⋅s/rad) */
#define SIM_MOTOR_COGGING_AMPLITUDE 0.05f /**< Cogging torque amplitude (Nm) */

#define SIM_DC_VOLTAGE 24.0f           /**< Simulated DC bus voltage */
#define SIM_AMBIENT_TEMPERATURE 25.0f  /**< Ambient temperature (°C) */
#define SIM_THERMAL_RESISTANCE 10.0f   /**< Thermal resistance (°C/W) */
#define SIM_ADC_NOISE_LEVEL 0.01f      /**< ADC noise level (% of full scale) */
#define SIM_CURRENT_SENSOR_GAIN 0.1f   /**< Current sensor gain (V/A) */
#define SIM_VOLTAGE_DIVIDER_RATIO 0.1f /**< Voltage divider ratio */

#define SIM_CONTROL_FREQUENCY 10000.0f        /**< Control loop frequency (Hz) */
#define SIM_DT (1.0f / SIM_CONTROL_FREQUENCY) /**< Time step (s) */

#define PI 3.14159265f

/*******************************************************************************************************************************
 * Simulation State Structure
 *******************************************************************************************************************************/

typedef struct {
  /* Motor electrical state */
  float rotor_angle;       /**< Rotor electrical angle (rad) */
  float rotor_velocity;    /**< Rotor velocity (rad/s) */
  float phase_currents[3]; /**< Phase currents (A) */
  float phase_voltages[3]; /**< Phase voltages (V) */
  float bemf_voltages[3];  /**< Back-EMF voltages (V) */

  /* Motor mechanical state */
  float torque_electrical; /**< Electrical torque (Nm) */
  float torque_load;       /**< Load torque (Nm) */
  float torque_cogging;    /**< Cogging torque (Nm) */

  /* PWM state */
  float pwm_duty[3];  /**< PWM duty cycles (0-1) */
  bool phase_high[3]; /**< Phase high-side state */
  bool phase_low[3];  /**< Phase low-side state */

  /* Thermal state */
  float temperature;       /**< Motor temperature (°C) */
  float power_dissipation; /**< Power dissipation (W) */

  /* Simulation control */
  uint32_t simulation_time;  /**< Simulation time (us) */
  uint32_t last_update_time; /**< Last update time (us) */
  bool simulation_running;   /**< Simulation running flag */

  /* Test/fault injection */
  bool inject_overcurrent;    /**< Inject overcurrent fault */
  bool inject_overvoltage;    /**< Inject overvoltage fault */
  bool inject_overtemp;       /**< Inject overtemperature fault */
  float injected_load_torque; /**< Injected load torque */

} SimulationState_t;

/*******************************************************************************************************************************
 * Static Variables
 *******************************************************************************************************************************/

static struct PwmConfig_t *s_pwm_config = NULL;
static struct AdcConfig_t *s_adc_config = NULL;
static SimulationState_t s_sim_state = { 0 };
static struct timespec s_start_time = { 0 };
static bool s_hal_initialized = false;

/*******************************************************************************************************************************
 * Private Helper Functions
 *******************************************************************************************************************************/

/**
 * @brief Add Gaussian noise to a signal
 */
static float add_noise(float signal, float noise_level) {
  static bool has_spare = false;
  static float spare;

  if (has_spare) {
    has_spare = false;
    return signal + spare * noise_level * signal;
  }

  has_spare = true;
  float u = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
  float v = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
  float mag = sqrt(u * u + v * v);

  if (mag > 1.0f) {
    return add_noise(signal, noise_level);  // Retry
  }

  spare = v * sqrt(-2.0f * log(mag) / mag);
  return signal + u * sqrt(-2.0f * log(mag) / mag) * noise_level * signal;
}

/**
 * @brief Calculate back-EMF voltages based on rotor position and velocity
 */
static void calculate_bemf(void) {
  float electrical_angle = s_sim_state.rotor_angle * (SIM_MOTOR_POLES / 2.0f);

  s_sim_state.bemf_voltages[0] = SIM_MOTOR_KE * s_sim_state.rotor_velocity * sinf(electrical_angle);
  s_sim_state.bemf_voltages[1] = SIM_MOTOR_KE * s_sim_state.rotor_velocity * sinf(electrical_angle - 2.0f * PI / 3.0f);
  s_sim_state.bemf_voltages[2] = SIM_MOTOR_KE * s_sim_state.rotor_velocity * sinf(electrical_angle - 4.0f * PI / 3.0f);
}

/**
 * @brief Calculate cogging torque
 */
static float calculate_cogging_torque(void) {
  float electrical_angle = s_sim_state.rotor_angle * (SIM_MOTOR_POLES / 2.0f);
  return SIM_MOTOR_COGGING_AMPLITUDE * sinf(electrical_angle * 6.0f);  // 6 cogging periods per electrical revolution
}

/**
 * @brief Update motor electrical dynamics
 */
static void update_electrical_dynamics(void) {
  float dt = SIM_DT;

  /* Calculate phase voltages based on PWM and switching states */
  for (int phase = 0; phase < 3; phase++) {
    if (s_sim_state.phase_high[phase]) {
      s_sim_state.phase_voltages[phase] = s_sim_state.pwm_duty[phase] * SIM_DC_VOLTAGE;
    } else if (s_sim_state.phase_low[phase]) {
      s_sim_state.phase_voltages[phase] = 0.0f;
    } else {
      /* Floating phase - voltage determined by back-EMF and current flow */
      s_sim_state.phase_voltages[phase] = s_sim_state.bemf_voltages[phase];
    }
  }

  /* Update phase currents using simplified electrical model */
  for (int phase = 0; phase < 3; phase++) {
    float voltage_drop = s_sim_state.phase_voltages[phase] - s_sim_state.bemf_voltages[phase];
    float target_current = voltage_drop / SIM_MOTOR_RESISTANCE;

    /* Simple first-order current dynamics */
    float tau = SIM_MOTOR_INDUCTANCE / SIM_MOTOR_RESISTANCE;
    s_sim_state.phase_currents[phase] += (target_current - s_sim_state.phase_currents[phase]) * dt / tau;

    /* Limit current to realistic values */
    if (s_sim_state.phase_currents[phase] > 20.0f) s_sim_state.phase_currents[phase] = 20.0f;
    if (s_sim_state.phase_currents[phase] < -20.0f) s_sim_state.phase_currents[phase] = -20.0f;
  }

  /* Calculate electrical torque */
  float electrical_angle = s_sim_state.rotor_angle * (SIM_MOTOR_POLES / 2.0f);
  s_sim_state.torque_electrical = SIM_MOTOR_KT * (s_sim_state.phase_currents[0] * sinf(electrical_angle) +
                                                  s_sim_state.phase_currents[1] * sinf(electrical_angle - 2.0f * PI / 3.0f) +
                                                  s_sim_state.phase_currents[2] * sinf(electrical_angle - 4.0f * PI / 3.0f));
}

/**
 * @brief Update motor mechanical dynamics
 */
static void update_mechanical_dynamics(void) {
  float dt = SIM_DT;

  /* Calculate cogging torque */
  s_sim_state.torque_cogging = calculate_cogging_torque();

  /* Total torque = electrical torque - load torque - friction - cogging */
  float total_torque = s_sim_state.torque_electrical - s_sim_state.torque_load - s_sim_state.injected_load_torque -
                       (SIM_MOTOR_FRICTION * s_sim_state.rotor_velocity) - s_sim_state.torque_cogging;

  /* Update rotor velocity: J * dω/dt = T_total */
  s_sim_state.rotor_velocity += (total_torque / SIM_MOTOR_INERTIA) * dt;

  /* Update rotor angle */
  s_sim_state.rotor_angle += s_sim_state.rotor_velocity * dt;

  /* Wrap angle to [0, 2π] */
  while (s_sim_state.rotor_angle >= 2.0f * PI) {
    s_sim_state.rotor_angle -= 2.0f * PI;
  }
  while (s_sim_state.rotor_angle < 0.0f) {
    s_sim_state.rotor_angle += 2.0f * PI;
  }
}

/**
 * @brief Update thermal dynamics
 */
static void update_thermal_dynamics(void) {
  float dt = SIM_DT;

  /* Calculate power dissipation */
  s_sim_state.power_dissipation = 0.0f;
  for (int phase = 0; phase < 3; phase++) {
    s_sim_state.power_dissipation += s_sim_state.phase_currents[phase] * s_sim_state.phase_currents[phase] * SIM_MOTOR_RESISTANCE;
  }

  /* Simple thermal model: C * dT/dt = P - (T - T_ambient) / R_th */
  float thermal_capacitance = 100.0f;  // J/°C
  float temp_rise = (s_sim_state.temperature - SIM_AMBIENT_TEMPERATURE) / SIM_THERMAL_RESISTANCE;
  s_sim_state.temperature += (s_sim_state.power_dissipation - temp_rise) * dt / thermal_capacitance;

  /* Ensure temperature doesn't go below ambient */
  if (s_sim_state.temperature < SIM_AMBIENT_TEMPERATURE) {
    s_sim_state.temperature = SIM_AMBIENT_TEMPERATURE;
  }
}

/**
 * @brief Update complete simulation state
 */
static void update_simulation_state(void) {
  if (!s_sim_state.simulation_running) {
    return;
  }

  uint32_t current_time = hal_get_micros();
  if ((current_time - s_sim_state.last_update_time) < (1000000 / SIM_CONTROL_FREQUENCY)) {
    return;  // Not time for update yet
  }

  calculate_bemf();
  update_electrical_dynamics();
  update_mechanical_dynamics();
  update_thermal_dynamics();

  s_sim_state.last_update_time = current_time;
  s_sim_state.simulation_time += (uint32_t)(SIM_DT * 1000000);
}

/*******************************************************************************************************************************
 * HAL Implementation
 *******************************************************************************************************************************/

bool hal_pwm_init(struct PwmConfig_t *config) {
  if (config == NULL) {
    return false;
  }

  s_pwm_config = config;

  /* Initialize PWM state */
  for (int i = 0; i < 3; i++) {
    s_sim_state.pwm_duty[i] = 0.0f;
    s_sim_state.phase_high[i] = false;
    s_sim_state.phase_low[i] = false;
  }

  printf("[SIM] PWM initialized - Frequency: %d Hz\n", config->frequency);
  return true;
}

bool hal_adc_init(struct AdcConfig_t *config) {
  if (config == NULL) {
    return false;
  }

  s_adc_config = config;
  printf("[SIM] ADC initialized - Resolution: %d bits\n", config->resolution);
  return true;
}

bool hal_gpio_init(void) {
  /* Initialize simulation state */
  memset(&s_sim_state, 0, sizeof(s_sim_state));
  s_sim_state.temperature = SIM_AMBIENT_TEMPERATURE;
  s_sim_state.simulation_running = true;

  /* Initialize random seed for noise generation */
  srand((unsigned int)time(NULL));

  /* Record start time */
  clock_gettime(CLOCK_MONOTONIC, &s_start_time);

  s_hal_initialized = true;
  printf("[SIM] GPIO and simulation initialized\n");
  return true;
}

void hal_gpio_set_phase_high(MotorPhase_t phase) {
  if (phase < 3) {
    s_sim_state.phase_high[phase] = true;
    s_sim_state.phase_low[phase] = false;
    printf("[SIM] Phase %d set HIGH\n", phase);
  }
}

void hal_gpio_set_phase_low(MotorPhase_t phase) {
  if (phase < 3) {
    s_sim_state.phase_high[phase] = false;
    s_sim_state.phase_low[phase] = true;
    printf("[SIM] Phase %d set LOW\n", phase);
  }
}

void hal_gpio_set_phase_float(MotorPhase_t phase) {
  if (phase < 3) {
    s_sim_state.phase_high[phase] = false;
    s_sim_state.phase_low[phase] = false;
    printf("[SIM] Phase %d set FLOAT\n", phase);
  }
}

void hal_pwm_set_duty(MotorPhase_t phase, uint16_t duty) {
  if (phase < 3) {
    s_sim_state.pwm_duty[phase] = (float)duty / 100.0f;  // Assuming duty is in percent

    /* Automatically set phase high when PWM is applied */
    if (duty > 0) {
      s_sim_state.phase_high[phase] = true;
      s_sim_state.phase_low[phase] = false;
    }

    printf("[SIM] Phase %d PWM duty: %.1f%%\n", phase, s_sim_state.pwm_duty[phase] * 100.0f);
  }
}

uint32_t hal_get_micros(void) {
  if (!s_hal_initialized) {
    return 0;
  }

  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);

  uint32_t elapsed_sec = ts.tv_sec - s_start_time.tv_sec;
  uint32_t elapsed_nsec = ts.tv_nsec - s_start_time.tv_nsec;

  if (elapsed_nsec < 0) {
    elapsed_sec--;
    elapsed_nsec += 1000000000;
  }

  return (elapsed_sec * 1000000) + (elapsed_nsec / 1000);
}

void hal_delay_us(uint32_t delay_us) {
  struct timespec ts;
  ts.tv_sec = delay_us / 1000000;
  ts.tv_nsec = (delay_us % 1000000) * 1000;
  nanosleep(&ts, NULL);
}

void hal_delay_ms(uint32_t delay_ms) {
  hal_delay_us(delay_ms * 1000);
}

void hal_adc_start_conversion(void) {
  /* Update simulation state before ADC conversion */
  update_simulation_state();
  printf("[SIM] ADC conversion started\n");
}

void hal_adc_get_phase_voltages(float *voltages) {
  if (voltages == NULL) return;

  for (int phase = 0; phase < 3; phase++) {
    voltages[phase] = add_noise(s_sim_state.phase_voltages[phase] * SIM_VOLTAGE_DIVIDER_RATIO, SIM_ADC_NOISE_LEVEL);

    /* Apply overvoltage fault injection */
    if (s_sim_state.inject_overvoltage) {
      voltages[phase] *= 1.5f;  // 50% overvoltage
    }
  }

  printf("[SIM] Phase voltages: A=%.2fV, B=%.2fV, C=%.2fV\n", voltages[0], voltages[1], voltages[2]);
}

void hal_adc_get_phase_currents(float *currents) {
  if (currents == NULL) return;

  for (int phase = 0; phase < 3; phase++) {
    currents[phase] = add_noise(s_sim_state.phase_currents[phase], SIM_ADC_NOISE_LEVEL);

    /* Apply overcurrent fault injection */
    if (s_sim_state.inject_overcurrent) {
      currents[phase] += 15.0f;  // Add 15A to trigger overcurrent
    }
  }

  printf("[SIM] Phase currents: A=%.2fA, B=%.2fA, C=%.2fA\n", currents[0], currents[1], currents[2]);
}

float hal_adc_get_dc_voltage(void) {
  float voltage = add_noise(SIM_DC_VOLTAGE, SIM_ADC_NOISE_LEVEL);

  /* Apply overvoltage fault injection */
  if (s_sim_state.inject_overvoltage) {
    voltage *= 1.3f;  // 30% overvoltage
  }

  printf("[SIM] DC voltage: %.2fV\n", voltage);
  return voltage;
}

float hal_adc_get_temperature(void) {
  float temp = add_noise(s_sim_state.temperature, SIM_ADC_NOISE_LEVEL);

  /* Apply overtemperature fault injection */
  if (s_sim_state.inject_overtemp) {
    temp += 50.0f;  // Add 50°C to trigger overtemperature
  }

  printf("[SIM] Temperature: %.1f°C\n", temp);
  return temp;
}

/*******************************************************************************************************************************
 * Simulation Control Functions (for testing)
 *******************************************************************************************************************************/

/**
 * @brief Set load torque for testing
 */
void hal_sim_set_load_torque(float torque_nm) {
  s_sim_state.injected_load_torque = torque_nm;
  printf("[SIM] Load torque set to %.3f Nm\n", torque_nm);
}

/**
 * @brief Inject faults for testing
 */
void hal_sim_inject_fault(const char *fault_type, bool enable) {
  if (strcmp(fault_type, "overcurrent") == 0) {
    s_sim_state.inject_overcurrent = enable;
    printf("[SIM] Overcurrent fault injection %s\n", enable ? "ENABLED" : "DISABLED");
  } else if (strcmp(fault_type, "overvoltage") == 0) {
    s_sim_state.inject_overvoltage = enable;
    printf("[SIM] Overvoltage fault injection %s\n", enable ? "ENABLED" : "DISABLED");
  } else if (strcmp(fault_type, "overtemp") == 0) {
    s_sim_state.inject_overtemp = enable;
    printf("[SIM] Overtemperature fault injection %s\n", enable ? "ENABLED" : "DISABLED");
  } else if (strcmp(fault_type, "overtemp") == 0) {
    s_sim_state.inject_overtemp = enable;
    printf("[SIM] Overtemperature fault injection %s\n", enable ? "ENABLED" : "DISABLED");
  } else {
    printf("[SIM] Unknown fault type '%s'\n", fault_type);
  }
}

/**
 * @brief Stop the simulation cleanly
 */
void hal_sim_stop(void) {
  s_sim_state.simulation_running = false;
  printf("[SIM] Simulation stopped\n");
}

/**
 * @brief Restart the simulation from initial state
 */
void hal_sim_restart(void) {
  memset(&s_sim_state, 0, sizeof(s_sim_state));
  s_sim_state.temperature = SIM_AMBIENT_TEMPERATURE;
  s_sim_state.simulation_running = true;
  s_sim_state.last_update_time = hal_get_micros();
  s_sim_state.simulation_time = 0;
  printf("[SIM] Simulation restarted\n");
}