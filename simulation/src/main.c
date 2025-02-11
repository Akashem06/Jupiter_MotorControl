/*******************************************************************************************************************************
 * @file   main.c
 *
 * @brief  Main file for motor simulation
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */
#include <stdio.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "sim_manager.h"

int main(void) {
  struct SimManager_t sim;

  if (sim_manager_init(&sim) == false) {
    printf("Simulation initialization failed!\n");
    return -1;
  }

  printf("Simulation initialized!\n");

  sim_manager_run(&sim);

  return 0;
}