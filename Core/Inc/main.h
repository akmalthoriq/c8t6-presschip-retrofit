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
#include "stm32f1xx_hal.h"

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
void System_VCP_Process(void);
void Send_VCP_Message(char* message);
void stop_all_actuators(void);
HAL_StatusTypeDef Flash_Save_All_Parameters(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEL_SW_1_Pin GPIO_PIN_13
#define SEL_SW_1_GPIO_Port GPIOC
#define SEL_SW_2_Pin GPIO_PIN_14
#define SEL_SW_2_GPIO_Port GPIOC
#define PRESS_GAUGE_Pin GPIO_PIN_0
#define PRESS_GAUGE_GPIO_Port GPIOA
#define PROX_HOMING_Pin GPIO_PIN_4
#define PROX_HOMING_GPIO_Port GPIOA
#define PROX_PRESS_Pin GPIO_PIN_5
#define PROX_PRESS_GPIO_Port GPIOA
#define PROX_EJECT_Pin GPIO_PIN_6
#define PROX_EJECT_GPIO_Port GPIOA
#define PROX_CLOSE_Pin GPIO_PIN_7
#define PROX_CLOSE_GPIO_Port GPIOA
#define PROX_OPEN_Pin GPIO_PIN_0
#define PROX_OPEN_GPIO_Port GPIOB
#define RELAY_AUGER_MAJU_Pin GPIO_PIN_1
#define RELAY_AUGER_MAJU_GPIO_Port GPIOB
#define RELAY_AUGER_MUNDUR_Pin GPIO_PIN_10
#define RELAY_AUGER_MUNDUR_GPIO_Port GPIOB
#define RELAY_CLOSE_Pin GPIO_PIN_12
#define RELAY_CLOSE_GPIO_Port GPIOB
#define RELAY_OPEN_Pin GPIO_PIN_13
#define RELAY_OPEN_GPIO_Port GPIOB
#define RELAY_DOWN_Pin GPIO_PIN_14
#define RELAY_DOWN_GPIO_Port GPIOB
#define RELAY_UP_Pin GPIO_PIN_15
#define RELAY_UP_GPIO_Port GPIOB
#define BTN_AUGER_MUNDUR_Pin GPIO_PIN_9
#define BTN_AUGER_MUNDUR_GPIO_Port GPIOA
#define BTN_AUGER_MAJU_Pin GPIO_PIN_10
#define BTN_AUGER_MAJU_GPIO_Port GPIOA
#define BTN_MUNDUR_Pin GPIO_PIN_3
#define BTN_MUNDUR_GPIO_Port GPIOB
#define BTN_MAJU_Pin GPIO_PIN_4
#define BTN_MAJU_GPIO_Port GPIOB
#define BTN_TURUN_Pin GPIO_PIN_5
#define BTN_TURUN_GPIO_Port GPIOB
#define BTN_NAIK_Pin GPIO_PIN_6
#define BTN_NAIK_GPIO_Port GPIOB
#define BTN_AUTO_CYCLE_Pin GPIO_PIN_7
#define BTN_AUTO_CYCLE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MAX_VCP_RX_BUFFER_SIZE 64

#define TEST_MODE 1 // 0 = mode test tidak aktif | 1 = MODE TEST AKTIF

// Konfigurasi Flash untuk Data Persistensi (F103)
#define FLASH_PAGE_TO_USE   FLASH_PAGE_SIZE * 31 // Menggunakan Page 31
#define FLASH_ADDRESS_DELAY      ((uint32_t)0x08007C00) // Alamat 1
#define FLASH_ADDRESS_VERT_UP    ((uint32_t)0x08007C02) // Alamat 2
#define FLASH_ADDRESS_PRESS_TIMER ((uint32_t)0x08007C04) // Alamat 3
#define FLASH_ADDRESS_TIMEOUT     ((uint32_t)0x08007C06) //Alamat 4
#define FLASH_ADDRESS_TARGET_BAR   ((uint32_t)0x08007C08) // Alamat 5



// Status Logika Sensor
#define PROXY_AKTIF  GPIO_PIN_RESET  // Sensor aktif ketika pin LOW (0)
#define PROXY_INAKTIF GPIO_PIN_SET   // Sensor inaktif ketika pin HIGH (1)

// Logika Kontrol Relay (Berdasarkan skema Optocoupler-Relay Board)
// Pin STM32 HIGH (SET) -> Optocoupler LED ON -> Transistor LOW (Collector) -> Relay Board LOW Trigger -> Relay ON/Aktuator ON
#define ACTUATOR_ON  GPIO_PIN_SET
// Pin STM32 LOW (RESET) -> Optocoupler LED OFF -> Transistor HIGH (Collector) -> Relay Board HIGH -> Relay OFF/Aktuator OFF
#define ACTUATOR_OFF GPIO_PIN_RESET

#define PRESS_TARGET_BAR 150.0f
#define PRESS_MAX_ADC    3723  // Nilai ADC untuk 300 Bar (~3.0V)
#define PRESS_MIN_ADC    410   // Nilai ADC untuk 0 Bar (~0.33V)


#define PRESS_GAUGE_TIMEOUT_MS 15000

#define GLOBAL_STATE_TIMEOUT_MS 60000

extern uint8_t VCP_RxBuffer[MAX_VCP_RX_BUFFER_SIZE];
extern uint32_t VCP_RxLength;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
