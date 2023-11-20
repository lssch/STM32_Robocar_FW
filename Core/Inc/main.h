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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define ESP32_SPI_CS_Pin GPIO_PIN_4
#define ESP32_SPI_CS_GPIO_Port GPIOA
#define ESP32_SPI_SCK_Pin GPIO_PIN_5
#define ESP32_SPI_SCK_GPIO_Port GPIOA
#define ESP32_SPI_MISO_Pin GPIO_PIN_6
#define ESP32_SPI_MISO_GPIO_Port GPIOA
#define ESP32_SPI_MOSI_Pin GPIO_PIN_7
#define ESP32_SPI_MOSI_GPIO_Port GPIOA
#define STM32_DBG_TX_Pin GPIO_PIN_9
#define STM32_DBG_TX_GPIO_Port GPIOA
#define STM32_DBG_RX_Pin GPIO_PIN_10
#define STM32_DBG_RX_GPIO_Port GPIOA
#define STM32_DBG_SWDIO_Pin GPIO_PIN_13
#define STM32_DBG_SWDIO_GPIO_Port GPIOA
#define STM32_DBG_SWCLK_Pin GPIO_PIN_14
#define STM32_DBG_SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
