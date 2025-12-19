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
#include "crc.h"
#include "dma2d.h"
#include "fdcan.h"
#include "i2c.h"
#include "ltdc.h"
#include "memorymap.h"
#include "quadspi.h"
#include "sdmmc.h"
#include "gpio.h"
#include "fmc.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8]; // Bufor na dane odebrane z VESC
uint8_t TxData[8]; // Bufor na dane do wysłania
FDCAN_TxHeaderTypeDef TxHeader;

// Zmienne do przechowywania odczytów z VESC
volatile int32_t vesc_rpmL = 0;
volatile int32_t vesc_rpmM = 0;
volatile int32_t vesc_rpmR = 0;
volatile float vesc_currentL = 0.0f;
volatile float vesc_currentM = 0.0f;
volatile float vesc_currentR = 0.0f;
volatile float vesc_dutyL = 0.0f;
volatile float vesc_dutyM = 0.0f;
volatile float vesc_dutyR = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_FMC_Init();
  MX_QUADSPI_Init();
  MX_LTDC_Init();
  MX_DMA2D_Init();
  MX_FDCAN1_Init();
  MX_I2C4_Init();
  MX_SDMMC2_SD_Init();
  MX_I2C2_Init();
  MX_CRC_Init();
  MX_TouchGFX_Init();
  /* USER CODE BEGIN 2 */
  // 1. Konfiguracja Filtra - akceptujemy WSZYSTKO (tryb otwarty dla testów)
    // VESC wysyła ramki z ID rozszerzonym (29-bit).
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0;     // Akceptuj wszystko
    sFilterConfig.FilterID2 = 0;     // Maska 0 = ignoruj bity ID (bierz wszystko jak leci)

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
      Error_Handler();
    }

    // 2. Start modułu FDCAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
      Error_Handler();
    }

    // 3. Włączenie przerwań (chcemy wiedzieć, kiedy przyjdzie nowa wiadomość)
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

  MX_TouchGFX_Process();
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_CSI
                              |RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    // Pobierz wiadomość z kolejki
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      return;
    }

    // Analiza danych z VESC
    // VESC ID mamy w dolnych 8 bitach Identifiera.
    // Ramka STATUS 1 ma Command ID = 9 (w bitach 8-15).
    // Pełne ID dla Status 1 od VESC 74 to: (9 << 8) | 74 = 0x94A

    // Sprawdzamy czy to ramka od naszego VESC (ID 74 = 0x4A)
    // Oraz czy to komenda STATUS 1 (0x09)
    uint32_t id = RxHeader.Identifier;
    uint8_t cmd_id = (id >> 8) & 0xFF;
    uint8_t controller_id = id & 0xFF;

    if (controller_id == 5 && cmd_id == 0x09) // 0x09 to CAN_PACKET_STATUS
    {
        // VESC wysyła dane w formacie Big Endian (MSB first)
        // Bajty 0-3: RPM (int32)
        // Bajty 4-5: Prąd (int16, skalowane x10)
        // Bajty 6-7: Duty Cycle (int16, skalowane x1000)

        vesc_rpmL = (int32_t)((RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3]);

        int16_t current_x10 = (int16_t)((RxData[4] << 8) | RxData[5]);
        vesc_currentL = (float)current_x10 / 10.0f;

        int16_t duty_x1000 = (int16_t)((RxData[6] << 8) | RxData[7]);
        vesc_dutyL = (float)duty_x1000 / 1000.0f;
    }
    if (controller_id == 47 && cmd_id == 0x09) // 0x09 to CAN_PACKET_STATUS
    {
        // VESC wysyła dane w formacie Big Endian (MSB first)
        // Bajty 0-3: RPM (int32)
        // Bajty 4-5: Prąd (int16, skalowane x10)
        // Bajty 6-7: Duty Cycle (int16, skalowane x1000)

        vesc_rpmM = (int32_t)((RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3]);

        int16_t current_x10 = (int16_t)((RxData[4] << 8) | RxData[5]);
        vesc_currentM = (float)current_x10 / 10.0f;

        int16_t duty_x1000 = (int16_t)((RxData[6] << 8) | RxData[7]);
        vesc_dutyM = (float)duty_x1000 / 1000.0f;
     }
    if (controller_id == 122 && cmd_id == 0x09) // 0x09 to CAN_PACKET_STATUS
     {
        // VESC wysyła dane w formacie Big Endian (MSB first)
        // Bajty 0-3: RPM (int32)
        // Bajty 4-5: Prąd (int16, skalowane x10)
        // Bajty 6-7: Duty Cycle (int16, skalowane x1000)

        vesc_rpmR = (int32_t)((RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3]);

        int16_t current_x10 = (int16_t)((RxData[4] << 8) | RxData[5]);
        vesc_currentR = (float)current_x10 / 10.0f;

        int16_t duty_x1000 = (int16_t)((RxData[6] << 8) | RxData[7]);
        vesc_dutyR
		= (float)duty_x1000 / 1000.0f;
     }


  }
}

//void VESC_SetDuty(float dutyCycle) // Zakres -1.0 do 1.0 (np. 0.5 to 50% mocy)
//{
//    // Zabezpieczenie zakresu
//    if(dutyCycle > 1.0f) dutyCycle = 1.0f;
//    if(dutyCycle < -1.0f) dutyCycle = -1.0f;
//
//    int32_t send_val = (int32_t)(dutyCycle * 100000.0f); // VESC oczekuje skali 100 000
//
//    // Konfiguracja nagłówka
//    TxHeader.Identifier = 74 | (0x00 << 8); // ID 74 + Komenda 0 (SET_DUTY)
//    TxHeader.IdType = FDCAN_EXTENDED_ID;
//    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//    TxHeader.DataLength = FDCAN_DLC_BYTES_4; // Wysyłamy 4 bajty
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//    TxHeader.FDFormat = FDCAN_CLASSIC_CAN; // VESC zazwyczaj używa klasycznego CAN, nie FD
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//    TxHeader.MessageMarker = 0;
//
//    // Pakowanie danych (Big Endian)
//    TxData[0] = (send_val >> 24) & 0xFF;
//    TxData[1] = (send_val >> 16) & 0xFF;
//    TxData[2] = (send_val >> 8) & 0xFF;
//    TxData[3] = (send_val) & 0xFF;
//
//    // Wysłanie
//    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
//}



/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

#ifdef  USE_FULL_ASSERT
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
