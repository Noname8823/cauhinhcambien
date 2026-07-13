/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>

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

/*
 * LED onboard.
 */
#define LED_Pin                         GPIO_PIN_13
#define LED_GPIO_Port                   GPIOC

/*
 * MAX485 số 2:
 * USART2 nối module LoRa E32 RS485.
 *
 * USART2_TX = PA2
 * USART2_RX = PA3
 * DE        = PA4
 * /RE       = PA5
 */
#define UART2_DE_Pin                    GPIO_PIN_4
#define UART2_DE_GPIO_Port              GPIOA

#define UART2_RE_Pin                    GPIO_PIN_5
#define UART2_RE_GPIO_Port              GPIOA

/*
 * MAX485 số 1:
 * USART1 đọc bus cảm biến RS485.
 *
 * USART1_TX = PA9
 * USART1_RX = PA10
 * DE        = PB0
 * /RE       = PB1
 */
#define UART1_DE_Pin                    GPIO_PIN_0
#define UART1_DE_GPIO_Port              GPIOB

#define UART1_RE_Pin                    GPIO_PIN_1
#define UART1_RE_GPIO_Port              GPIOB

/*
 * Chân điều khiển RF cũ của project.
 */
#define RF_MODE_Pin                     GPIO_PIN_13
#define RF_MODE_GPIO_Port               GPIOB

/*
 * Bốn digital input.
 */
#define IN1_Pin                         GPIO_PIN_5
#define IN1_GPIO_Port                   GPIOB

#define IN2_Pin                         GPIO_PIN_6
#define IN2_GPIO_Port                   GPIOB

#define IN3_Pin                         GPIO_PIN_7
#define IN3_GPIO_Port                   GPIOB

#define IN4_Pin                         GPIO_PIN_8
#define IN4_GPIO_Port                   GPIOB

/* USER CODE BEGIN Private defines */

/*
 * Tên sử dụng trong rs485_port.c, station_app.c và main.c.
 *
 * MAX485 số 1:
 * Đọc chung ba cảm biến:
 *   - Gió ID 1
 *   - Mưa ID 2
 *   - Đất ID 3
 */
#define RS485_SENSOR_DE_Pin             UART1_DE_Pin
#define RS485_SENSOR_DE_GPIO_Port       UART1_DE_GPIO_Port

#define RS485_SENSOR_RE_Pin             UART1_RE_Pin
#define RS485_SENSOR_RE_GPIO_Port       UART1_RE_GPIO_Port

/*
 * MAX485 số 2:
 * Gửi gói dữ liệu sang LoRa E32 RS485.
 */
#define RS485_E32_DE_Pin                UART2_DE_Pin
#define RS485_E32_DE_GPIO_Port          UART2_DE_GPIO_Port

#define RS485_E32_RE_Pin                UART2_RE_Pin
#define RS485_E32_RE_GPIO_Port          UART2_RE_GPIO_Port

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
