/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#define FW_VERSION 0x0A



/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER DEFINES */
//#define GYRO_TS
#define MEMS_SR 10
#define MEMS_SR_SEC (float)(MEMS_SR / 1000)


void magnCalStart(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
