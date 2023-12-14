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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void delay_us (uint16_t us);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VFS_RESET_Pin GPIO_PIN_0
#define VFS_RESET_GPIO_Port GPIOC
#define VFS_SPI_MOSI_Pin GPIO_PIN_1
#define VFS_SPI_MOSI_GPIO_Port GPIOC
#define VFS_SPI_MISO_Pin GPIO_PIN_2
#define VFS_SPI_MISO_GPIO_Port GPIOC
#define MOTOR1_IN1_Pin GPIO_PIN_3
#define MOTOR1_IN1_GPIO_Port GPIOC
#define ESP32_OK_Pin GPIO_PIN_0
#define ESP32_OK_GPIO_Port GPIOA
#define ESP32_ERROR_Pin GPIO_PIN_1
#define ESP32_ERROR_GPIO_Port GPIOA
#define ESP32_COMM_START_Pin GPIO_PIN_2
#define ESP32_COMM_START_GPIO_Port GPIOA
#define ESP32_RES1_Pin GPIO_PIN_3
#define ESP32_RES1_GPIO_Port GPIOA
#define ESP32_SPI_CS_Pin GPIO_PIN_4
#define ESP32_SPI_CS_GPIO_Port GPIOA
#define ESP32_SPI_CS_EXTI_IRQn EXTI4_IRQn
#define ESP32_SPI_SCK_Pin GPIO_PIN_5
#define ESP32_SPI_SCK_GPIO_Port GPIOA
#define ESP32_SPI_MISO_Pin GPIO_PIN_6
#define ESP32_SPI_MISO_GPIO_Port GPIOA
#define ESP32_SPI_MOSI_Pin GPIO_PIN_7
#define ESP32_SPI_MOSI_GPIO_Port GPIOA
#define MOTOR1_IN2_Pin GPIO_PIN_4
#define MOTOR1_IN2_GPIO_Port GPIOC
#define MOTOR_STB_Pin GPIO_PIN_5
#define MOTOR_STB_GPIO_Port GPIOC
#define USR_CONFIG1_Pin GPIO_PIN_0
#define USR_CONFIG1_GPIO_Port GPIOB
#define USR_CONCIG2_Pin GPIO_PIN_1
#define USR_CONCIG2_GPIO_Port GPIOB
#define USR_CONGIG3_Pin GPIO_PIN_2
#define USR_CONGIG3_GPIO_Port GPIOB
#define LED_PIXEL_VAVIGATON_Pin GPIO_PIN_10
#define LED_PIXEL_VAVIGATON_GPIO_Port GPIOB
#define VFS_SPI_CS_Pin GPIO_PIN_12
#define VFS_SPI_CS_GPIO_Port GPIOB
#define VFS_SPI_SCK_Pin GPIO_PIN_13
#define VFS_SPI_SCK_GPIO_Port GPIOB
#define USR_CONFIG4_Pin GPIO_PIN_14
#define USR_CONFIG4_GPIO_Port GPIOB
#define USR_BUTTON_Pin GPIO_PIN_15
#define USR_BUTTON_GPIO_Port GPIOB
#define TOF_CAM_TX_Pin GPIO_PIN_6
#define TOF_CAM_TX_GPIO_Port GPIOC
#define TOF_CAM_RX_Pin GPIO_PIN_7
#define TOF_CAM_RX_GPIO_Port GPIOC
#define MOTOR2_IN1_Pin GPIO_PIN_8
#define MOTOR2_IN1_GPIO_Port GPIOC
#define MOTOR2_IN2_Pin GPIO_PIN_9
#define MOTOR2_IN2_GPIO_Port GPIOC
#define ESP32_RES3_Pin GPIO_PIN_8
#define ESP32_RES3_GPIO_Port GPIOA
#define STM32_DBG_TX_Pin GPIO_PIN_9
#define STM32_DBG_TX_GPIO_Port GPIOA
#define STM32_DBG_RX_Pin GPIO_PIN_10
#define STM32_DBG_RX_GPIO_Port GPIOA
#define ESP32_RESET_Pin GPIO_PIN_11
#define ESP32_RESET_GPIO_Port GPIOA
#define STM32_DBG_SWDIO_Pin GPIO_PIN_13
#define STM32_DBG_SWDIO_GPIO_Port GPIOA
#define STM32_DBG_SWCLK_Pin GPIO_PIN_14
#define STM32_DBG_SWCLK_GPIO_Port GPIOA
#define LED_PIXEL_STATE_VFS_Pin GPIO_PIN_15
#define LED_PIXEL_STATE_VFS_GPIO_Port GPIOA
#define TOF_SPOT_UART_TX_Pin GPIO_PIN_10
#define TOF_SPOT_UART_TX_GPIO_Port GPIOC
#define TOF_SPOT_UART_RX_Pin GPIO_PIN_11
#define TOF_SPOT_UART_RX_GPIO_Port GPIOC
#define VFS_EXTERNAL_LIGHT_Pin GPIO_PIN_12
#define VFS_EXTERNAL_LIGHT_GPIO_Port GPIOC
#define MOTOR1_PWM_Pin GPIO_PIN_4
#define MOTOR1_PWM_GPIO_Port GPIOB
#define MOTOR2_PWM_Pin GPIO_PIN_5
#define MOTOR2_PWM_GPIO_Port GPIOB
#define SERVO1_PWM_Pin GPIO_PIN_6
#define SERVO1_PWM_GPIO_Port GPIOB
#define SERVO2_PWM_Pin GPIO_PIN_7
#define SERVO2_PWM_GPIO_Port GPIOB
#define IMU_I2C_SCL_Pin GPIO_PIN_8
#define IMU_I2C_SCL_GPIO_Port GPIOB
#define IMU_I2C_SDA_Pin GPIO_PIN_9
#define IMU_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
