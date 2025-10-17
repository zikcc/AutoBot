/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "mcp4017.h"
#include "ads8867.h"
#include "w25q128.h"
#include "ly68l6400.h"
#include "myusart.h"
#include "myfun.h"
#include "stdio.h"
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
#define Power_Ctr_Pin GPIO_PIN_13
#define Power_Ctr_GPIO_Port GPIOC
#define LY_CE_Pin GPIO_PIN_2
#define LY_CE_GPIO_Port GPIOA
#define LY_SCLK_Pin GPIO_PIN_3
#define LY_SCLK_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define LY_SIO3_Pin GPIO_PIN_6
#define LY_SIO3_GPIO_Port GPIOA
#define LY_SIO2_Pin GPIO_PIN_7
#define LY_SIO2_GPIO_Port GPIOA
#define LY_SIO1_Pin GPIO_PIN_0
#define LY_SIO1_GPIO_Port GPIOB
#define LY_SIO0_Pin GPIO_PIN_1
#define LY_SIO0_GPIO_Port GPIOB
#define SCL_MCP_Pin GPIO_PIN_10
#define SCL_MCP_GPIO_Port GPIOB
#define SDA_MCP_Pin GPIO_PIN_11
#define SDA_MCP_GPIO_Port GPIOB
#define W25_CS_Pin GPIO_PIN_12
#define W25_CS_GPIO_Port GPIOB
#define W25_CLK_Pin GPIO_PIN_13
#define W25_CLK_GPIO_Port GPIOB
#define W25_DO_Pin GPIO_PIN_14
#define W25_DO_GPIO_Port GPIOB
#define W25_DI_Pin GPIO_PIN_15
#define W25_DI_GPIO_Port GPIOB
#define DOUT_Pin GPIO_PIN_11
#define DOUT_GPIO_Port GPIOA
#define DIN_Pin GPIO_PIN_12
#define DIN_GPIO_Port GPIOA
#define CONVST_Pin GPIO_PIN_15
#define CONVST_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define IO1_Pin GPIO_PIN_4
#define IO1_GPIO_Port GPIOB
#define IO2_Pin GPIO_PIN_5
#define IO2_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern TIM_HandleTypeDef htim2;
extern uint8_t tim_endflag;
extern UART_HandleTypeDef huart1;
extern uint8_t adcData[800000] __attribute__((at(0x90000000)));
extern uint8_t calistatus;//采样状态
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
