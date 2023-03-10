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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "eeprom.h"
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
#define SYS_STA_LED_Pin GPIO_PIN_0
#define SYS_STA_LED_GPIO_Port GPIOA
#define POWER_LED_Pin GPIO_PIN_1
#define POWER_LED_GPIO_Port GPIOA
#define POWER_CHRING_Pin GPIO_PIN_4
#define POWER_CHRING_GPIO_Port GPIOC
#define WT588_DAT_Pin GPIO_PIN_5
#define WT588_DAT_GPIO_Port GPIOC
#define WT588_CLK_Pin GPIO_PIN_0
#define WT588_CLK_GPIO_Port GPIOB
#define WT588_BUSY_Pin GPIO_PIN_1
#define WT588_BUSY_GPIO_Port GPIOB
#define Power_Key_Pin GPIO_PIN_2
#define Power_Key_GPIO_Port GPIOB
#define Power_IP5310Key_EN_Pin GPIO_PIN_10
#define Power_IP5310Key_EN_GPIO_Port GPIOB
#define BLE_State_Pin GPIO_PIN_13
#define BLE_State_GPIO_Port GPIOB
#define BLE_PWRC_Pin GPIO_PIN_14
#define BLE_PWRC_GPIO_Port GPIOB
#define BLE_Power_EN_Pin GPIO_PIN_15
#define BLE_Power_EN_GPIO_Port GPIOB
#define BLE_RST_Pin GPIO_PIN_7
#define BLE_RST_GPIO_Port GPIOC
#define SdDetect_Pin GPIO_PIN_15
#define SdDetect_GPIO_Port GPIOA
  /* USER CODE BEGIN Private defines */

#define MAXU2MAX 530
#define powerKeyInAct HAL_GPIO_ReadPin(Power_Key_GPIO_Port, Power_Key_Pin) == GPIO_PIN_RESET                  // 按键读取
#define powerKeyInRel HAL_GPIO_ReadPin(Power_Key_GPIO_Port, Power_Key_Pin) == GPIO_PIN_SET                    // 按键弹起
#define powerKeyPress HAL_GPIO_WritePin(Power_IP5310Key_EN_GPIO_Port, Power_IP5310Key_EN_Pin, GPIO_PIN_SET)   // 按下
#define powerKeyRelse HAL_GPIO_WritePin(Power_IP5310Key_EN_GPIO_Port, Power_IP5310Key_EN_Pin, GPIO_PIN_RESET) // 松开
#define ChongDianFlag HAL_GPIO_ReadPin(POWER_CHRING_GPIO_Port, POWER_CHRING_Pin) == GPIO_PIN_RESET            // 充电识别
#define ChongNoneFlag HAL_GPIO_ReadPin(POWER_CHRING_GPIO_Port, POWER_CHRING_Pin) == GPIO_PIN_SET              // 没有在充电
#define BlerstH HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_SET)
#define BlerstL HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_RESET)
#define BlePwrOn HAL_GPIO_WritePin(BLE_Power_EN_GPIO_Port, BLE_Power_EN_Pin, GPIO_PIN_RESET)
#define BlePwrOf HAL_GPIO_WritePin(BLE_Power_EN_GPIO_Port, BLE_Power_EN_Pin, GPIO_PIN_SET)
#define BlePwrcH HAL_GPIO_WritePin(BLE_PWRC_GPIO_Port, BLE_PWRC_Pin, GPIO_PIN_SET)
#define BlePwrcL HAL_GPIO_WritePin(BLE_PWRC_GPIO_Port, BLE_PWRC_Pin, GPIO_PIN_RESET)

  extern unsigned char TxBufU2[MAXU2MAX];
  extern unsigned char RxBufU2[MAXU2MAX];
  extern unsigned short RxLenU2;
  extern UART_HandleTypeDef huart2;
  extern TIM_HandleTypeDef htim2;
  extern void IIC_Send_Byte(unsigned char txd);
  extern void u2rxstrdecodeProcess(void);
  extern unsigned char gotoAppflag;
  extern void System_Jump2Bootloader(void);
  extern void shortKeyPress(void);
  extern void delay_ns(unsigned int ndelay);
  extern void delay_us(unsigned int udelay);
  extern void delay_ms(unsigned int nms);
  extern void I2C_Delay_us(uint16_t t);
  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
