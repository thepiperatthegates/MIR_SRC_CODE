/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"


#define DMA_ADC_BUFF_SIZE 4
#define MAX_TOTAL_TRANSMISSION_BUFFER 12*DMA_ADC_BUFF_SIZE
#define USB_PAYLOAD 1000 //1KB
#define TOTAL_RX_BUFFER 4
#define INCOMING_PACKET_UINT 6
#define INCOMING_PACKET_INT 2
#define INCOMING_PACKET_FLOAT 1

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Start_LED_Pin GPIO_PIN_4
#define Start_LED_GPIO_Port GPIOK
#define Filter_LED_Pin GPIO_PIN_3
#define Filter_LED_GPIO_Port GPIOK

/* USER CODE BEGIN Private defines */


void inital_setup(float32_t dac_frequency, uint32_t running_time, uint32_t amplitude_1,
  int offset_1, uint32_t amplitude_2, int offset_2, uint32_t direction,
uint32_t mode_button_flag, uint32_t reset_mcu_flag);

extern __IO uint8_t send_flag;



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
