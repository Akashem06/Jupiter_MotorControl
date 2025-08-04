#pragma once

/*******************************************************************************************************************************
 * @file   utils_error.h
 *
 * @brief  Header file for the Utils errors
 *
 * @date   2025-02-07
 * @author Aryan Kashem
 *******************************************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup UtilsClass Utils storage class
 * @brief    Utils agonistic storage class
 * @{
 */

/**
 * @brief   Utils error class
 */
typedef enum {
  UTILS_OK,
  UTILS_INVALID_ARGS,
  UTILS_UNINITIALIZED,
  UTILS_INTERNAL_ERROR,
} UtilsError_t;

/** @} */
