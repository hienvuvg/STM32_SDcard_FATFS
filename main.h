/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <stdio.h>
#include "stdbool.h"


#include <string.h>
#include <stdarg.h> //for va_list var arg functions


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void HF_SystemConfig(void);
void LF_SystemConfig(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LDO_EN_Pin GPIO_PIN_6
#define LDO_EN_GPIO_Port GPIOC
#define SD_SPI_CS_Pin GPIO_PIN_6
#define SD_SPI_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/********** Start of GPIO ******************/

#define BlinkPin        HAL_GPIO_TogglePin

#define SetPin          HAL_GPIO_WritePin
#define ReadPin         HAL_GPIO_ReadPin

#define OffIO         	GPIO_PIN_RESET
#define OnIO           	GPIO_PIN_SET

#define OffSPI         	GPIO_PIN_SET
#define OnSPI           GPIO_PIN_RESET

#define OnLED         	GPIO_PIN_RESET
#define OffLED       	GPIO_PIN_SET

#define OnGate         	GPIO_PIN_RESET
#define OffGate       	GPIO_PIN_SET

#define OnRFID         	GPIO_PIN_RESET
#define OffRFID       	GPIO_PIN_SET

#define OnLDO			GPIO_PIN_SET
#define OffLDO			GPIO_PIN_RESET


/********** End of GPIO ******************/


// For logging
#define IMU_SAMPLING	25	// Hz
#define T_TIMESTAMP		60 	// Seconds
#define N_TIMESTAMPS	9	// Samples
#define N_SAMPLES		IMU_SAMPLING * T_TIMESTAMP
#define N_TIME_BUFF		N_TIMESTAMPS * 4
#define N_ELEMENTS		N_SAMPLES * 3
#define BUFF_LENGTH		N_TIMESTAMPS * N_ELEMENTS + N_TIME_BUFF // 14000*3 // Elements (36)
//#define BUFF_LENGTH		1 * N_ELEMENTS + 15 // 14000*3 // Elements (36)

#define SD_SPI_HANDLE hspi1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
