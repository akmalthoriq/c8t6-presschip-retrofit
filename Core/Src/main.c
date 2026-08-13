/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h" // untuk CDC_Transmit_FS
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STATE_EMERGENCY_STOP,
	    STATE_IDLE,
		STATE_HORIZ_CLOSE_FIRST,
	    STATE_HOMING,
	    STATE_VERT_PRESS_BY_GAUGE,
	    STATE_VERT_PRESS_BY_TIMER,
		STATE_AUGER_FILLING,
	    STATE_DELAY_MATERIAL_IN,
	    STATE_VERT_EJECT,
		STATE_UP_TO_PROX_VERT_MIDDLE,
		STATE_VERT_UP_BY_TIMER,
	    STATE_HORIZ_OPEN,
	    STATE_HORIZ_CLOSE,
//	    STATE_CYCLE_END_TO_HOME
} CycleState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
volatile CycleState_t CurrentState = STATE_IDLE;
volatile uint8_t auto_cycle_active = 0;   // 1 = Auto Cycle ON, 0 = Manual/Idle
// Parameter yang bisa diubah via USB VCP (Default 10 detik = 10000ms)
volatile uint32_t material_in_delay_ms = 10000;
volatile uint32_t delay_after_naik_ms = 1500;
volatile uint32_t delay_after_naik_timer_start = 0;

volatile uint32_t vert_up_timer_start = 0;
volatile uint32_t vert_up_duration_ms = 3000;

// Waktu mulai timer (menggunakan HAL_GetTick)
volatile uint32_t material_in_timer_start = 0;
// Variabel untuk Debounce & Latch Tombol Manual
uint8_t manual_btn_state[6] = {0}; // 0: Naik, 1: Turun, 2: Maju, 3: Mundur
const char* manual_action_names[] = {"NAIK", "TURUN", "MAJU", "MUNDUR"};

volatile uint16_t current_pressure_adc = 0; // Nilai ADC (0-4095)
volatile uint8_t auto_cycle_mode = 1;     // 1, 2, atau 3 (Dipilih oleh selector switch)
volatile uint8_t vert_press_counter = 0;  // Counter untuk siklus 2 & 3

// Timer untuk Press By Timer
volatile uint32_t press_by_timer_duration_ms = 4000; // Default 4 detik untuk press timer
volatile uint32_t press_by_timer_start = 0;

volatile uint32_t gauge_press_timer_start = 0;

volatile float target_pressure_bar = 150.0f;
volatile uint16_t target_pressure_adc = 2066;

volatile uint32_t state_timer_start = 0; //timer di setiap state
volatile uint32_t global_timeout_duration_ms = GLOBAL_STATE_TIMEOUT_MS;


// Buffer VCP untuk menerima data
uint8_t VCP_RxBuffer[MAX_VCP_RX_BUFFER_SIZE];
uint32_t VCP_RxLength = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void stop_all_actuators(void);
void vertikal_naik(void);
void vertikal_turun(void);
void horizontal_maju(void);
void horizontal_mundur(void);
void Handle_Manual_Mode(void);
void System_Run_Cycle(void);
void System_VCP_Process(void);
void Send_VCP_Message(char* message);
void remove_whitespace(char* s);
void auger_maju(void);
void auger_mundur(void);
void auger_stop(void);
uint8_t read_auto_cycle_mode(void);
uint16_t convert_bar_to_adc(float bar_val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  uint32_t saved_delay = *(volatile uint32_t*)FLASH_ADDRESS_DELAY;
  uint32_t saved_vert_up = *(volatile uint32_t*)FLASH_ADDRESS_VERT_UP;
  uint32_t saved_press_timer = *(volatile uint32_t*)FLASH_ADDRESS_PRESS_TIMER;
  uint32_t saved_global_timeout = *(volatile uint32_t*)FLASH_ADDRESS_TIMEOUT;
  uint32_t saved_target_bar = *(volatile uint32_t*)FLASH_ADDRESS_TARGET_BAR;

    // Cek jika nilai material_in_delay_ms valid
    if (saved_delay != 0xFFFFFFFF && saved_delay >= 500 && saved_delay <= 60000) {
        material_in_delay_ms = saved_delay;
    } else {
        material_in_delay_ms = 10000;
    }

    // Cek jika nilai vert_up_duration_ms valid
    if (saved_vert_up != 0xFFFFFFFF && saved_vert_up >= 500 && saved_vert_up <= 10000) {
        vert_up_duration_ms = saved_vert_up;
    } else {
        vert_up_duration_ms = 3000; // Default 3 detik
    }

    if (saved_press_timer != 0xFFFFFFFF && saved_press_timer >= 500 && saved_press_timer <= 10000) {
          press_by_timer_duration_ms = saved_press_timer;
      } else {
          press_by_timer_duration_ms = 5000; // Default 4 detik
      }

    if (saved_global_timeout != 0xFFFFFFFF && saved_global_timeout >= 5000 && saved_global_timeout <= 60000) {
        global_timeout_duration_ms = saved_global_timeout;
    } else {
        global_timeout_duration_ms = GLOBAL_STATE_TIMEOUT_MS;
    }

    if (saved_target_bar != 0xFFFFFFFF && saved_target_bar >= 50 && saved_target_bar <= 300) {
        target_pressure_bar = (float)saved_target_bar;
    } else {
        target_pressure_bar = 150.0f; // Default 150 Bar
    }

    target_pressure_adc = convert_bar_to_adc(target_pressure_bar);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&current_pressure_adc, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  System_Run_Cycle();
	  System_VCP_Process();
	  HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_AUGER_MAJU_Pin|RELAY_AUGER_MUNDUR_Pin|RELAY_CLOSE_Pin|RELAY_OPEN_Pin
                          |RELAY_DOWN_Pin|RELAY_UP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEL_SW_1_Pin SEL_SW_2_Pin */
  GPIO_InitStruct.Pin = SEL_SW_1_Pin|SEL_SW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PROX_HOMING_Pin PROX_PRESS_Pin PROX_EJECT_Pin PROX_CLOSE_Pin
                           BTN_AUGER_MUNDUR_Pin BTN_AUGER_MAJU_Pin */
  GPIO_InitStruct.Pin = PROX_HOMING_Pin|PROX_PRESS_Pin|PROX_EJECT_Pin|PROX_CLOSE_Pin
                          |BTN_AUGER_MUNDUR_Pin|BTN_AUGER_MAJU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PROX_OPEN_Pin BTN_MUNDUR_Pin BTN_MAJU_Pin BTN_TURUN_Pin
                           BTN_NAIK_Pin BTN_AUTO_CYCLE_Pin */
  GPIO_InitStruct.Pin = PROX_OPEN_Pin|BTN_MUNDUR_Pin|BTN_MAJU_Pin|BTN_TURUN_Pin
                          |BTN_NAIK_Pin|BTN_AUTO_CYCLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_AUGER_MAJU_Pin RELAY_AUGER_MUNDUR_Pin */
  GPIO_InitStruct.Pin = RELAY_AUGER_MAJU_Pin|RELAY_AUGER_MUNDUR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_CLOSE_Pin RELAY_OPEN_Pin RELAY_DOWN_Pin RELAY_UP_Pin */
  GPIO_InitStruct.Pin = RELAY_CLOSE_Pin|RELAY_OPEN_Pin|RELAY_DOWN_Pin|RELAY_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Mematikan semua aktuator (Relay OFF).
 */
void stop_all_actuators(void) {
    // ACTUATOR_OFF (GPIO_PIN_RESET) untuk mematikan relay
    HAL_GPIO_WritePin(RELAY_DOWN_GPIO_Port, RELAY_DOWN_Pin, ACTUATOR_OFF);
    HAL_GPIO_WritePin(RELAY_UP_GPIO_Port, RELAY_UP_Pin, ACTUATOR_OFF);
    HAL_GPIO_WritePin(RELAY_CLOSE_GPIO_Port, RELAY_CLOSE_Pin, ACTUATOR_OFF);
    HAL_GPIO_WritePin(RELAY_OPEN_GPIO_Port, RELAY_OPEN_Pin, ACTUATOR_OFF);
    auger_stop();
}

void vertikal_naik(void) {
    stop_all_actuators();
    HAL_GPIO_WritePin(RELAY_UP_GPIO_Port, RELAY_UP_Pin, ACTUATOR_ON);
}

void vertikal_turun(void) {
    stop_all_actuators();
    HAL_GPIO_WritePin(RELAY_DOWN_GPIO_Port, RELAY_DOWN_Pin, ACTUATOR_ON);

}

void horizontal_maju(void) { // Menutup Pembuangan
    stop_all_actuators();
    HAL_GPIO_WritePin(RELAY_CLOSE_GPIO_Port, RELAY_CLOSE_Pin, ACTUATOR_ON);

}

void horizontal_mundur(void) { // Membuka Pembuangan
    stop_all_actuators();
    HAL_GPIO_WritePin(RELAY_OPEN_GPIO_Port, RELAY_OPEN_Pin, ACTUATOR_ON);
}

void auger_maju(void) {
    HAL_GPIO_WritePin(RELAY_AUGER_MAJU_GPIO_Port, RELAY_AUGER_MAJU_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_AUGER_MUNDUR_GPIO_Port, RELAY_AUGER_MUNDUR_Pin, GPIO_PIN_RESET);
}

void auger_mundur(void) {
    HAL_GPIO_WritePin(RELAY_AUGER_MAJU_GPIO_Port, RELAY_AUGER_MAJU_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_AUGER_MUNDUR_GPIO_Port, RELAY_AUGER_MUNDUR_Pin, GPIO_PIN_SET);
}

void auger_stop(void) {
    HAL_GPIO_WritePin(RELAY_AUGER_MAJU_GPIO_Port, RELAY_AUGER_MAJU_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_AUGER_MUNDUR_GPIO_Port, RELAY_AUGER_MUNDUR_Pin, GPIO_PIN_RESET);
}

uint8_t read_auto_cycle_mode(void) {
    // Selector switch di Pull-Up, jadi yang LOW adalah yang terpilih.
    if (HAL_GPIO_ReadPin(SEL_SW_1_GPIO_Port, SEL_SW_1_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(SEL_SW_2_GPIO_Port, SEL_SW_2_Pin) == GPIO_PIN_SET) {
        return 1; // Siklus 1
    } else if (HAL_GPIO_ReadPin(SEL_SW_1_GPIO_Port, SEL_SW_1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(SEL_SW_2_GPIO_Port, SEL_SW_2_Pin) == GPIO_PIN_SET) {
        return 2; // Siklus 2
    } else if (HAL_GPIO_ReadPin(SEL_SW_1_GPIO_Port, SEL_SW_1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(SEL_SW_2_GPIO_Port, SEL_SW_2_Pin) == GPIO_PIN_RESET) {
        return 3; // Siklus 3
    }

    //DEFAULT menggunkaan siklus 1
    return 1;
}

/**
 * @brief Mengkonversi nilai Bar ke nilai ADC mentah (untuk threshold kontrol).
 * * Target: 0 Bar -> 410 ADC, 300 Bar -> 3723 ADC.
 */
uint16_t convert_bar_to_adc(float bar_val) {
    // ADC_MIN_ACTUAL: 410.0f, ADC_MAX_ACTUAL: 3723.0f, PRESSURE_MAX_RANGE: 300.0f

    //nilai slope
    const float SLOPE = (3723.0f - 410.0f) / 300.0f;

    //nilai intercept
    const float INTERCEPT_ADC = 410.0f;

    // Persamaan liniear: ADC = SLOPE * Bar + INTERCEPT_ADC
    float adc_val_float = (SLOPE * bar_val) + INTERCEPT_ADC;

    if (adc_val_float > 4095.0f) return 4095;
    if (adc_val_float < 0.0f) return 0;

    return (uint16_t)adc_val_float;
}


/**
 * @brief Menghapus karakter whitespace (termasuk \r, \n, spasi) dari akhir string.
 */
void remove_whitespace(char *s) {
    int len = strlen(s);
    // Hapus karakter whitespace dari belakang
    while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' ')) {
        s[len-1] = '\0';
        len--;
    }
}


/**
 * @brief Mengirim pesan string melalui USB VCP
 */
void Send_VCP_Message(char* message) {
    uint16_t len = strlen(message);
    // Tambahkan karakter baris baru untuk memudahkan pembacaan di terminal
    char final_message[len + 3];
    sprintf(final_message, "%s\r\n", message);
    CDC_Transmit_FS((uint8_t*)final_message, strlen(final_message));
}


/**
 * @brief Memproses perintah VCP yang diterima
 */
void System_VCP_Process(void) {
    if (VCP_RxLength == 0) return;

    // Pastikan string diakhiri null
    if (VCP_RxLength < MAX_VCP_RX_BUFFER_SIZE) {
        VCP_RxBuffer[VCP_RxLength] = '\0';
    } else {
        VCP_RxBuffer[MAX_VCP_RX_BUFFER_SIZE - 1] = '\0';
    }

    char response[150];
    char* cmd = (char*)VCP_RxBuffer;

    // Bersihin string dari whitespace
    remove_whitespace(cmd);

   // Proses perintah
    if (strncmp(cmd, "$DMI:", 9) == 0) {
            long new_delay = atol(cmd + 9);
            if (new_delay >= 500 && new_delay <= 60000) {

                material_in_delay_ms = (uint32_t)new_delay; // Update RAM

                // Panggil fungsi SAVE ALL
                if (Flash_Save_All_Parameters() == HAL_OK) {
                    sprintf(response, "OK: DELAY disetel & DISIMPAN ke %lu ms", material_in_delay_ms);
                } else {
                    sprintf(response, "ERROR: Gagal menyimpan ke Flash.");
                }
            } else {
                sprintf(response, "ERROR: Delay tidak valid (500-60000 ms).");
            }
            Send_VCP_Message(response);

        } else if (strncmp(cmd, "$VUD:", 8) == 0) {
                long new_duration = atol(cmd + 8);
                if (new_duration >= 500 && new_duration <= 10000) {

                    vert_up_duration_ms = (uint32_t)new_duration; // Update RAM

                    // Panggil fungsi SAVE ALL
                    if (Flash_Save_All_Parameters() == HAL_OK) {
                        sprintf(response, "OK: Vertikal Up Duration disetel & DISIMPAN ke %lu ms", vert_up_duration_ms);
                    } else {
                        sprintf(response, "ERROR: Gagal menyimpan Press Up Duration ke Flash.");
                    }
                } else {
                    sprintf(response, "ERROR: Vert Up Duration tidak valid (500-10000 ms).");
                }
                Send_VCP_Message(response);
        } else if (strncmp(cmd, "$PTD:", 5) == 0) {
                    long new_duration = atol(cmd + 5);
                    if (new_duration >= 500 && new_duration <= 10000) {

                        press_by_timer_duration_ms = (uint32_t)new_duration; // Update RAM

                        if (Flash_Save_All_Parameters() == HAL_OK) {
                            sprintf(response, "OK: Press Timer Duration disetel & DISIMPAN ke %lu ms", press_by_timer_duration_ms);
                        } else {
                            sprintf(response, "ERROR: Gagal menyimpan Press Timer Duration ke Flash.");
                        }
                    } else {
                        sprintf(response, "ERROR: Press Timer Duration tidak valid (500-10000 ms).");
                    }
                    Send_VCP_Message(response);

        } else if (strncmp(cmd, "$GTO:", 5) == 0) {
            long new_duration = atol(cmd + 5);
            if (new_duration >= 5000 && new_duration <= 60000) { // Batas 5s - 60s

                global_timeout_duration_ms = (uint32_t)new_duration; // Update RAM

                if (Flash_Save_All_Parameters() == HAL_OK) {
                    sprintf(response, "OK: Global Timeout disetel & DISIMPAN ke %lu ms", global_timeout_duration_ms);
                } else {
                    sprintf(response, "ERROR: Gagal menyimpan Global Timeout ke Flash.");
                }
            } else {
                sprintf(response, "ERROR: Global Timeout tidak valid (5000-60000 ms).");
            }
            Send_VCP_Message(response);

    } else if (strncmp(cmd, "$PTB:", 5) == 0) {
        float new_target = atof(cmd + 5); // atof untuk float

        // minimal 50 bar, maksimal 300 bar
        if (new_target >= 50.0f && new_target <= 300.0f) {

            target_pressure_bar = new_target; // Update RAM (Bar)

            // HITUNG NILAI ADC BARU SECARA OTOMATIS
            target_pressure_adc = convert_bar_to_adc(new_target);//konversi bar ke adc


            if (Flash_Save_All_Parameters() == HAL_OK) {
                sprintf(response, "OK: Target Press disetel & DISIMPAN ke %.1f Bar (%d ADC)", target_pressure_bar, target_pressure_adc);
            } else {
                sprintf(response, "ERROR: Gagal menyimpan Target Bar ke Flash.");
            }
        } else {
            sprintf(response, "ERROR: Target Bar tidak valid (50-300 Bar).");
        }
        Send_VCP_Message(response);

    } else if (strcmp(cmd, "GETSTATUS") == 0) {
		char state_str[35];

		switch (CurrentState) {
			case STATE_IDLE: strcpy(state_str, "IDLE"); break;
			case STATE_HOMING: strcpy(state_str, "HOMING"); break;
			case STATE_DELAY_MATERIAL_IN: strcpy(state_str, "DELAY, MATERIAL IN"); break;
			case STATE_VERT_PRESS_BY_GAUGE: strcpy(state_str, "VERT_PRESS_GAUGE"); break;
			case STATE_VERT_PRESS_BY_TIMER:strcpy(state_str, "VERT_PRESS_TIMER");break;
			case STATE_VERT_UP_BY_TIMER: strcpy(state_str, "VERT UP BY TIMER"); break;
			case STATE_HORIZ_OPEN: strcpy(state_str, "HORIZ_OPEN"); break;
			case STATE_VERT_EJECT: strcpy(state_str, "VERT_EJECT"); break;
			case STATE_UP_TO_PROX_VERT_MIDDLE: strcpy(state_str, "NAIK TO PROX_VERT_MIDDLE"); break;
			case STATE_HORIZ_CLOSE: strcpy(state_str, "HORIZ_CLOSE"); break;
			case STATE_EMERGENCY_STOP: strcpy(state_str, "E-STOP"); break;
			default: strcpy(state_str, "UNKNOWN"); break;
	}

        float pressure_bar = ((float)current_pressure_adc - PRESS_MIN_ADC) * (PRESS_TARGET_BAR / (PRESS_MAX_ADC - PRESS_MIN_ADC));
		if (pressure_bar < 0) pressure_bar = 0.0f;
		if(pressure_bar > 300) pressure_bar = 300.0f;

        uint8_t prox_vert_homing = HAL_GPIO_ReadPin(PROX_HOMING_GPIO_Port, PROX_HOMING_Pin) == PROXY_AKTIF;
        uint8_t prox_vert_middle = HAL_GPIO_ReadPin(PROX_PRESS_GPIO_Port, PROX_PRESS_Pin) == PROXY_AKTIF;
        uint8_t prox_vert_eject = HAL_GPIO_ReadPin(PROX_EJECT_GPIO_Port, PROX_EJECT_Pin) == PROXY_AKTIF;
        uint8_t prox_horiz_close = HAL_GPIO_ReadPin(PROX_CLOSE_GPIO_Port, PROX_CLOSE_Pin) == PROXY_AKTIF;
        uint8_t prox_horiz_open = HAL_GPIO_ReadPin(PROX_OPEN_GPIO_Port, PROX_OPEN_Pin) == PROXY_AKTIF;

        sprintf(response, "STATUS: %s | Auto:%d Mode Siklus: %d | Press: %.1f Bar | Delay:%lu ms | Proxies: H:%d M:%d E:%d C:%d O:%d",
                        state_str, auto_cycle_active, auto_cycle_mode, pressure_bar, material_in_delay_ms, prox_vert_homing, prox_vert_middle, prox_vert_eject, prox_horiz_close, prox_horiz_open);
		Send_VCP_Message(response);
    } else {
        sprintf(response, "ERROR: Perintah '%s' tidak dikenal.", cmd);
        Send_VCP_Message(response);
    }

    // Reset buffer
    VCP_RxLength = 0;
}

/**
 * @brief Menangani pergerakan silinder secara manual.
 */
void Handle_Manual_Mode(void) {
    // Pin LOW = Ditekan (Proxy/Button aktif)
    uint8_t btn_turun  = (HAL_GPIO_ReadPin(BTN_TURUN_GPIO_Port, BTN_TURUN_Pin) == GPIO_PIN_RESET);
    uint8_t btn_naik   = (HAL_GPIO_ReadPin(BTN_NAIK_GPIO_Port, BTN_NAIK_Pin) == GPIO_PIN_RESET);
    uint8_t btn_mundur = (HAL_GPIO_ReadPin(BTN_MUNDUR_GPIO_Port, BTN_MUNDUR_Pin) == GPIO_PIN_RESET);
    uint8_t btn_maju   = (HAL_GPIO_ReadPin(BTN_MAJU_GPIO_Port, BTN_MAJU_Pin) == GPIO_PIN_RESET);
    uint8_t btn_auger_maju   = (HAL_GPIO_ReadPin(BTN_AUGER_MAJU_GPIO_Port, BTN_AUGER_MAJU_Pin) == GPIO_PIN_RESET);
    uint8_t btn_auger_mundur = (HAL_GPIO_ReadPin(BTN_AUGER_MUNDUR_GPIO_Port, BTN_AUGER_MUNDUR_Pin) == GPIO_PIN_RESET);

    // Safety: Prioritas Tombol Berlawanan
    if ((btn_maju && btn_mundur) || (btn_naik && btn_turun) || (btn_auger_maju && btn_auger_mundur)) {
        stop_all_actuators();
        return;
    }

    // Eksekusi Gerakan
    if (btn_naik) {
        vertikal_naik();
        if (manual_btn_state[0] == 0) {
            Send_VCP_Message("MANUAL: Vertikal NAIK (Aktif)");
            manual_btn_state[0] = 1;
        }
    } else if (btn_turun) {
        vertikal_turun();
        if (manual_btn_state[1] == 0) {
            Send_VCP_Message("MANUAL: Vertikal TURUN (Aktif)");
            manual_btn_state[1] = 1;
        }
    } else if (btn_maju) {
        horizontal_maju();
        if (manual_btn_state[2] == 0) {
            Send_VCP_Message("MANUAL: Horizontal MAJU (Tutup) (Aktif)");
            manual_btn_state[2] = 1;
        }
    } else if (btn_mundur) {
        horizontal_mundur();
        if (manual_btn_state[3] == 0) {
            Send_VCP_Message("MANUAL: Horizontal MUNDUR (Buka) (Aktif)");
            manual_btn_state[3] = 1;
        }
    } else if (btn_auger_maju) {
        auger_maju();
        if (manual_btn_state[4] == 0) {
            Send_VCP_Message("MANUAL: Auger MAJU (Aktif)");
            manual_btn_state[4] = 1;
        }
    } else if (btn_auger_mundur) {
        auger_mundur();
        if (manual_btn_state[5] == 0) {
            Send_VCP_Message("MANUAL: Auger MUNDUR (Aktif)");
            manual_btn_state[5] = 1;
        }
    } else {
        // Jika TIDAK ADA tombol yang ditekan, baru matikan semua
        stop_all_actuators();

        // Kirim pesan stop hanya jika sebelumnya ada tombol yang aktif
        if (manual_btn_state[0] || manual_btn_state[1] || manual_btn_state[2] ||
            manual_btn_state[3] || manual_btn_state[4] || manual_btn_state[5]) {
            Send_VCP_Message("MANUAL: Semua gerakan dihentikan.");
        }

        // Reset semua state saat tidak ada tombol ditekan
        for(int i=0; i<6; i++) manual_btn_state[i] = 0;
    }
}


/**
 * @brief Logika utama State Machine untuk siklus otomatis.
 */
void System_Run_Cycle(void) {
    // Baca status Proximity Sensor (PROXY_AKTIF = LOW/0)
    uint8_t prox_vert_homing = HAL_GPIO_ReadPin(PROX_HOMING_GPIO_Port, PROX_HOMING_Pin);
    uint8_t prox_vert_middle = HAL_GPIO_ReadPin(PROX_PRESS_GPIO_Port, PROX_PRESS_Pin);
    uint8_t prox_vert_eject = HAL_GPIO_ReadPin(PROX_EJECT_GPIO_Port, PROX_EJECT_Pin);
    uint8_t prox_horiz_close = HAL_GPIO_ReadPin(PROX_CLOSE_GPIO_Port, PROX_CLOSE_Pin);
    uint8_t prox_horiz_open = HAL_GPIO_ReadPin(PROX_OPEN_GPIO_Port, PROX_OPEN_Pin);

    // Tombol Auto Cycle (Latching) - Pin LOW = Ditekan/ON
    uint8_t btn_auto = (HAL_GPIO_ReadPin(BTN_AUTO_CYCLE_GPIO_Port, BTN_AUTO_CYCLE_Pin) == GPIO_PIN_RESET);

    // Cek Safety/Error Handler - Prioritas tertinggi
#if TEST_MODE == 0
    if ((prox_vert_homing == PROXY_AKTIF && prox_vert_middle == PROXY_AKTIF) || (prox_vert_middle == PROXY_INAKTIF && prox_vert_eject == PROXY_INAKTIF)) {
        CurrentState = STATE_EMERGENCY_STOP;
        Send_VCP_Message("CRITICAL ERROR: Sensor Vertikal Tidak Konsisten!");
    }
    //logika sensor prox_vert_middle dibalik, jika PROXY_AKTIF (berarti aktualnya off), jika PROXY_INAKTIF (aktual nya aktif)
    // Jika waktu press melebihi batas aman (misal 2x delay)
    if (CurrentState == STATE_DELAY_MATERIAL_IN && (HAL_GetTick() - material_in_timer_start) > (material_in_delay_ms * 2)) {
         CurrentState = STATE_EMERGENCY_STOP;
         Send_VCP_Message("CRITICAL ERROR: Waktu Press Melebihi Batas Aman!");
    }

    if (prox_horiz_close == PROXY_AKTIF && prox_horiz_open == PROXY_AKTIF){
    	CurrentState = STATE_EMERGENCY_STOP; //kondisi ketika kedua sensor untuk buka/tutup pembuangan sama-sama aktif, langsung emergency
    	Send_VCP_Message("CRITICAL ERROR: Sensor Horizontal(buka dan tutup pembuangan) sama-sama AKTIF!");
    }

#endif

    //Transisi Mode (Auto ON/OFF)
    if (btn_auto) {
            if (auto_cycle_active == 0) {
                auto_cycle_mode = read_auto_cycle_mode();
                vert_press_counter = 0; //reset counter
                auto_cycle_active = 1;
                CurrentState = STATE_HORIZ_CLOSE_FIRST; // Selalu mulai dari m
                char msg_buffer[100];
                sprintf(msg_buffer, "AUTO CYCLE ON. Mode: %d. Menutup pintu pembuangan awal sebelum Homing.", auto_cycle_mode);
                Send_VCP_Message(msg_buffer);
            }
	} else {
        if (auto_cycle_active == 1) {
            auto_cycle_active = 0;
            CurrentState = STATE_IDLE; // Kembali ke IDLE/MANUAL
            stop_all_actuators();
            Send_VCP_Message("AUTO CYCLE OFF. Kembali ke Manual Mode.");
        }
    }

    //GLOBAL STATE TIMEOUT
    if (auto_cycle_active) {
        if (state_timer_start == 0) {
            // Inisialisasi saat pertama kali masuk auto mode
            state_timer_start = HAL_GetTick();
        } else if (HAL_GetTick() - state_timer_start > global_timeout_duration_ms) {
            // TIMEOUT GLOBAL! Pindah ke EMERGENCY STOP
            CurrentState = STATE_EMERGENCY_STOP;
            state_timer_start = 0; // Reset
            char msg_buffer_timeout[85];
			sprintf(msg_buffer_timeout, "CRITICAL ERROR: GLOBAL STATE TIMEOUT! Siklus terjebak selama %lu ms.", global_timeout_duration_ms);
			Send_VCP_Message(msg_buffer_timeout);
        }
    } else {
        // Reset timer saat dalam mode IDLE/Manual
        state_timer_start = 0;
    }

    //logika state mesin 
        switch (CurrentState) {
            case STATE_EMERGENCY_STOP:
                stop_all_actuators();
                break;

            case STATE_IDLE:
                stop_all_actuators();
                // Mode manual cuman boleh saat sedang di state IDLE
                Handle_Manual_Mode();
                break;

            // 1. Menutup pembuangan hingga proxy close terdeteksi di awal siklus
            case STATE_HORIZ_CLOSE_FIRST:
                if (!auto_cycle_active) break;
                horizontal_maju(); // Gerakkan pintu tertutup

                if (prox_horiz_close == PROXY_AKTIF && prox_horiz_open == PROXY_INAKTIF) {
                    CurrentState = STATE_HOMING;
                    stop_all_actuators();
                    Send_VCP_Message("Pintu Tertutup. Memulai Homing piston vertikal.");
                    state_timer_start = HAL_GetTick();
                }
                break;

            // 2. Homing hingga proxy homing terdeteksi
            case STATE_HOMING:
                if (!auto_cycle_active) break;
                vertikal_naik(); // Hanya fokus naik, pembuangan sudah diurus di awal

                // Cek jika semua sensor vertikal sudah nyala atau belum
                //Note: khusus logika prox_vert_middle, logikanya terbalik, PROXY_INAKTIF -> SENSOR ON
                if (prox_vert_homing == PROXY_AKTIF && prox_vert_middle == PROXY_INAKTIF && prox_vert_eject == PROXY_INAKTIF) {
                    vert_press_counter++;
                    CurrentState = STATE_AUGER_FILLING;
                    stop_all_actuators();
                    auger_maju(); // 3. Jalankan motor pengisi
                    material_in_timer_start = HAL_GetTick(); // Mulai hitung waktu pengisian
                    Send_VCP_Message("Homing Selesai. Auger mengisi chip...");
                }
                break;

            case STATE_AUGER_FILLING:
                if (!auto_cycle_active) { auger_stop(); break; }

                // Jika waktu pengisian (DMI) sudah tercapai
                if (HAL_GetTick() - material_in_timer_start >= material_in_delay_ms) {
                    auger_stop(); // Matikan auger

                    uint8_t total_press_count = auto_cycle_mode;
                    if (vert_press_counter < total_press_count) {
                        CurrentState = STATE_VERT_PRESS_BY_TIMER;
                        press_by_timer_start = HAL_GetTick();
                    } else {
                        CurrentState = STATE_VERT_PRESS_BY_GAUGE;

                        // >>> TAMBAHKAN BARIS INI <<<
                        gauge_press_timer_start = HAL_GetTick();
                    }
                    Send_VCP_Message("Pengisian Chip Selesai. Memulai Press.");
                }
                break;

            case STATE_DELAY_MATERIAL_IN:
    			if (!auto_cycle_active) break;
    			// Cek waktu
    			if (HAL_GetTick() - material_in_timer_start >= material_in_delay_ms) {

    				uint8_t total_press_count = auto_cycle_mode;

    				if (vert_press_counter < total_press_count){
    					//hanya untuk mode siklus 2 dan siklus 3
    					CurrentState = STATE_VERT_PRESS_BY_TIMER;
    					press_by_timer_start = HAL_GetTick();
    					char msg_buffer_2[70];
    					sprintf(msg_buffer_2, "Jeda Selesai. Menuju press by TIMER. (#%d)", vert_press_counter);
    					Send_VCP_Message(msg_buffer_2);

    				} else if (vert_press_counter  == total_press_count){
    					//Siklus 1 (1/1), Siklus 2 (2/2), Siklus 3 (3/3) : pake sensor gauge
    					CurrentState = STATE_VERT_PRESS_BY_GAUGE;
    					Send_VCP_Message("Jeda selesai. Menuju Press by GAUGE (Final Press).");
    				} else {
    					//kalo counter kelebihan
    					CurrentState = STATE_EMERGENCY_STOP;
    					Send_VCP_Message("CRITICAL ERROR: Counter Press Kelebihan");
    				}
    			}
    			break;

            case STATE_VERT_PRESS_BY_GAUGE:
                if (!auto_cycle_active) break;

                // >>> HAPUS BLOK IF (CurrentState != ...) INI KARENA SUDAH TIDAK PERLU <<<

                vertikal_turun();

                if (HAL_GetTick() - gauge_press_timer_start > PRESS_GAUGE_TIMEOUT_MS) {
                    CurrentState = STATE_EMERGENCY_STOP;
                    gauge_press_timer_start = 0; // Reset timer
                    Send_VCP_Message("CRITICAL ERROR: PRESS GAUGE TIMEOUT! Gagal mencapai target Bar dalam 30s.");
                    break;
                }

                //cek apakah tekanan target tercapai
                if (current_pressure_adc >= target_pressure_adc) {
                    CurrentState = STATE_VERT_UP_BY_TIMER;
                    stop_all_actuators();
                    Send_VCP_Message("PRESS GAUGE SELESAI: Target Bar tercapai. Naik By Timer");
                    vert_up_timer_start = HAL_GetTick();
                    gauge_press_timer_start = 0; // >>> OPSIONAL: Tambahkan ini untuk keamanan ekstra <<<
                }
                break;

            case STATE_VERT_PRESS_BY_TIMER:
            	if(!auto_cycle_active) break;
            	vertikal_turun();

            	if(HAL_GetTick() - press_by_timer_start >= press_by_timer_duration_ms){
            		//press by timer selesai. kembali homing untuk press selanjutnya
            		CurrentState = STATE_HOMING;
            		stop_all_actuators();
            		Send_VCP_Message("PRESS BY TIMER selesai. Kembali Homing untuk siklus press selanjutnya");
            		state_timer_start = HAL_GetTick();
            	}
            	break;

            case STATE_VERT_UP_BY_TIMER:
            	if(!auto_cycle_active) break;

            	vertikal_naik();

            	if (HAL_GetTick() - vert_up_timer_start >= vert_up_duration_ms){
            		CurrentState = STATE_HORIZ_OPEN;
            		stop_all_actuators();
            		Send_VCP_Message("Silinder vertikal naik sementara selesai. Menuju membuka pembuangan");
            		state_timer_start = HAL_GetTick();
            	}
            	break;

            case STATE_HORIZ_OPEN:
    			if (!auto_cycle_active) break;
    				horizontal_mundur(); // Buka Pembuangan

    			if (prox_horiz_close == PROXY_INAKTIF && prox_horiz_open == PROXY_AKTIF) {
    				CurrentState = STATE_VERT_EJECT;
    				stop_all_actuators();
    				Send_VCP_Message("Pembuangan Terbuka. Menunggu silinder vertikal turun mendorong material keluar.");
    				state_timer_start = HAL_GetTick();
    			}
    			break;

            // 6. Mengeluarkan material/ piston maju hingga proxy eject terdeteksi
            case STATE_VERT_EJECT:
            	if (!auto_cycle_active) break;

            	            vertikal_turun();

            	            // Konversi 50 Bar ke nilai ADC
            	            uint16_t eject_50bar_adc = convert_bar_to_adc(50.0f);

            	            // Deteksi 50 Bar pertama kali
            	            if (current_pressure_adc >= eject_50bar_adc) {
            	                CurrentState = STATE_HORIZ_CLOSE;

            	                // JANGAN panggil stop_all_actuators() agar gerakan tidak terputus
            	                delay_after_naik_timer_start = HAL_GetTick();
            	                state_timer_start = HAL_GetTick();
            	                Send_VCP_Message("Eject 50 Bar terdeteksi. Mulai menutup pintu sambil mempertahankan 50 Bar.");
            	            }
            	            break;

            // Tarik piston ke atas hingga batas aman agar tidak menabrak pintu
            case STATE_UP_TO_PROX_VERT_MIDDLE:
    			if(!auto_cycle_active) break;
    				vertikal_naik();

    				if (prox_vert_homing == PROXY_INAKTIF && prox_vert_middle == PROXY_INAKTIF && prox_vert_eject == PROXY_AKTIF){
    					CurrentState =  STATE_HORIZ_CLOSE;
    					stop_all_actuators();
    					Send_VCP_Message("Naik batas aman selesai. Delay 1,5 detik sebelum menutup pembuangan");
    					delay_after_naik_timer_start = HAL_GetTick();
    					state_timer_start = HAL_GetTick();
    				}
    			break;

            // 7. Jika proxy eject sudah terdeteksi (dan aman), pintu pembuangan tertutup lagi
            case STATE_HORIZ_CLOSE:
            	if (!auto_cycle_active) break;

            	            uint16_t target_50bar_adc = convert_bar_to_adc(50.0f);

            	            // --- A. KONTROL PINTU HORIZONTAL ---
            	            // Setelah delay terpenuhi, nyalakan relay pintu maju (tanpa mematikan relay vertikal)
            	            if (HAL_GetTick() - delay_after_naik_timer_start >= delay_after_naik_ms) {
            	                HAL_GPIO_WritePin(RELAY_CLOSE_GPIO_Port, RELAY_CLOSE_Pin, ACTUATOR_ON);
            	            }

            	            // --- B. KONTROL TEKANAN PISTON VERTIKAL (MEMPERTAHANKAN 50 BAR) ---
            	            // Jika tekanan turun < 50 Bar, dorong lagi. Jika sudah >= 50 Bar, matikan sementara agar tidak over-pressure.
            	            if (current_pressure_adc < target_50bar_adc) {
            	                HAL_GPIO_WritePin(RELAY_DOWN_GPIO_Port, RELAY_DOWN_Pin, ACTUATOR_ON);
            	            } else {
            	                HAL_GPIO_WritePin(RELAY_DOWN_GPIO_Port, RELAY_DOWN_Pin, ACTUATOR_OFF);
            	            }

            	            // --- C. CEK KONDISI PINTU SUDAH TERTUTUP RAPAT ---
            	            if (prox_horiz_close == PROXY_AKTIF && prox_horiz_open == PROXY_INAKTIF) {
            	                CurrentState = STATE_HOMING;

            	                // Matikan semua aktuator HANYA saat pintu sudah benar-benar tertutup rapat
            	                stop_all_actuators();

            	                Send_VCP_Message("Pembuangan berhasil ditutup rapat. Mengulang ke Homing.");
            	                state_timer_start = HAL_GetTick();
            	            }
            	            break;
        }

}

HAL_StatusTypeDef Flash_Save_All_Parameters(void)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    // 16-bit versions of the 32-bit parameters
    uint16_t delay_halfword = (uint16_t)material_in_delay_ms;
    uint16_t vert_up_halfword = (uint16_t)vert_up_duration_ms;
    uint16_t press_timer_halfword = (uint16_t)press_by_timer_duration_ms;
    uint16_t timeout_halfword = (uint16_t)global_timeout_duration_ms;
    uint16_t target_bar_halfword = (uint16_t)target_pressure_bar;

    // 1. Unlock Flash
    HAL_FLASH_Unlock();

    // 2. Setup Erase Operation
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_ADDRESS_DELAY; // Target Page
    EraseInitStruct.NbPages = 1;

    // 3. Erase Page
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // 4. Program Data 1 (material_in_delay_ms)
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ADDRESS_DELAY, delay_halfword);
    if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }

    // 5. Program Data 2 (vert_up_duration_ms)
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ADDRESS_VERT_UP, vert_up_halfword);
    if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }

    // 6. Program Data 3 (press_by_timer_duration_ms)
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ADDRESS_PRESS_TIMER, press_timer_halfword);
	if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }

	// 7. Program Data 4 (global_timeout_duration_ms)
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ADDRESS_TIMEOUT, timeout_halfword);
	if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }

	// 8. Program Data 5 (target_pressure_bar)
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ADDRESS_TARGET_BAR, target_bar_halfword);

	// 9. Lock Flash
	HAL_FLASH_Lock();

    return status;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
