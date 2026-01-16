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
#include "cmsis_os.h"
#include "crc.h"
#include "dma2d.h"
#include "fdcan.h"
#include "i2c.h"
#include "ltdc.h"
#include "memorymap.h"
#include "quadspi.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"
#include "app_touchgfx.h"
#include "mpu6050.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"

extern uint8_t BSP_QSPI_Init(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILTER_SAMPLES 50 // Długość bufora uśredniającego (wygładzanie wykresu)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8]; // Bufor na dane odebrane z VESC
uint8_t TxData[8]; // Bufor na dane do wysłania
// --- 1. DEFINICJA STRUKTURY DLA WSZYSTKICH STATUSÓW ---
FDCAN_TxHeaderTypeDef TxHeader;

// --- 2. INSTANCJE DLA SILNIKÓW ---
// Ustawiamy ID Twoich VESC-ów
#define VESC_ID_L  1   // Lewy
#define VESC_ID_M  2   // Środkowy
#define VESC_ID_R  3   // Prawy (przykładowo)

VescData_t vescL; // Zmienna dla lewego
VescData_t vescM; // Zmienna dla środkowego
VescData_t vescR; // Zmienna dla prawego

MPU6050_t MPU6050;
float AngX, AngY;
volatile float vehicleSpeedKmh = 0.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void VESC_SetCurrent(uint8_t controller_id, float current_amps);
void VESC_SetBrakeCurrent(uint8_t controller_id, float current_amps);
void MPU_Config(void);
int32_t Get_Filtered_RPM(int32_t new_val);
void MotorControl_Task_Entry(void);
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
  MX_LTDC_Init();
  MX_DMA2D_Init();
  MX_FDCAN1_Init();
  MX_I2C4_Init();
  MX_I2C2_Init();
  MX_CRC_Init();
  MX_QUADSPI_Init();
  MX_USART1_UART_Init();
  MX_TouchGFX_Init();
  /* Call PreOsInit function */
  MX_TouchGFX_PreOSInit();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_Delay(120);
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

    /* Initialize QSPI Flash */
    if (BSP_QSPI_Init() != HAL_OK)
    {
      Error_Handler();
    }
    if (GT911_Init(&hi2c2) == 0) {
        // Obsługa błędu inicjalizacji dotyku
    }

    MPU6050_Init(&hi2c2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
    // Czy przyszła nowa wiadomość?
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        // Pobierz wiadomość
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 1. Rozpoznanie KTO wysłał (ID VESC)
            uint8_t id_nadawcy = (RxHeader.Identifier & 0xFF);

            // 2. Rozpoznanie CO wysłał (Komenda)
            uint8_t komenda = (RxHeader.Identifier >> 8) & 0xFF;

            // 3. Wybór odpowiedniej struktury (Wskaźnik)
            VescData_t *target = NULL;

            if      (id_nadawcy == VESC_ID_M) target = &vescM; // Środkowy
            else if (id_nadawcy == VESC_ID_L) target = &vescL; // Lewy
            else if (id_nadawcy == VESC_ID_R) target = &vescR; // Prawy

            // Jeśli znaleźliśmy pasujący silnik, dekodujemy dane
            if (target != NULL)
            {
                switch (komenda)
                {
                    // --- STATUS 1: RPM, Prąd Silnika, Duty ---
                    case 0x09:
                        target->rpm = (int32_t)((RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3]);
                        target->current_motor = (float)((int16_t)((RxData[4] << 8) | RxData[5])) / 10.0f;
                        target->duty_cycle = (float)((int16_t)((RxData[6] << 8) | RxData[7])) / 1000.0f;
                        break;

                        // --- STATUS 2: Ah zużyte, Ah odzyskane ---
                        case 0x0E:
                            target->amp_hours = (float)((int32_t)((RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3])) / 10000.0f;
                            // POPRAWKA PONIŻEJ: 24, 16, 8, 0
                            target->amp_hours_chg = (float)((int32_t)((RxData[4] << 24) | (RxData[5] << 16) | (RxData[6] << 8) | RxData[7])) / 10000.0f;
                            break;

                        // --- STATUS 3: Wh zużyte, Wh odzyskane ---
                        case 0x0F:
                             target->watt_hours = (float)((int32_t)((RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3])) / 10000.0f;
                             // POPRAWKA PONIŻEJ: 24, 16, 8, 0
                             target->watt_hours_chg = (float)((int32_t)((RxData[4] << 24) | (RxData[5] << 16) | (RxData[6] << 8) | RxData[7])) / 10000.0f;
                             break;

                    // --- STATUS 4: Temperatury, Prąd Baterii ---
                    case 0x10:
                        target->temp_fet = (float)((int16_t)((RxData[0] << 8) | RxData[1])) / 10.0f;
                        target->temp_motor = (float)((int16_t)((RxData[2] << 8) | RxData[3])) / 10.0f;
                        target->current_in = (float)((int16_t)((RxData[4] << 8) | RxData[5])) / 10.0f;
                        target->pid_pos = (float)((int16_t)((RxData[6] << 8) | RxData[7])) / 50.0f;
                        break;

                    // --- STATUS 5: Tachometr, Napięcie Baterii ---
                    case 0x1B:
                        target->tacho_value = (int32_t)((RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3]);
                        target->v_in = (float)((int16_t)((RxData[4] << 8) | RxData[5])) / 10.0f;
                        break;

                    default:
                        break;
                }
            }
        }
        // Ponowne włączenie nasłuchiwania
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}

// --- Funkcja ustawiania PRĄDU (Momentu) ---
// Komenda 0x01 w VESC to Set Current
void VESC_SetCurrent(uint8_t controller_id, float current_amps)
{
    // 1. Zabezpieczenie przed ujemnym prądem (chyba że chcesz hamować/wsteczny)
    // Zakładamy, że rower jedzie tylko do przodu.
    if (current_amps < 0.0f) current_amps = 0.0f;

    // 2. Zabezpieczenie maksymalnego prądu (bezpiecznik programowy)
    // Z tabelki wynika max 20A, ale dajmy bezpiecznik np. 30A
    if (current_amps > 30.0f) current_amps = 30.0f;

    // 3. Skalowanie: VESC oczekuje prądu * 1000 (miliampery)
    int32_t send_val = (int32_t)(current_amps * 1000.0f);

    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[4];

    // Budowa ID ramki: [KOMENDA 0x01] [ID_STEROWNIKA]
    TxHeader.Identifier = (uint32_t)controller_id | ((uint32_t)0x01 << 8);

    // Konfiguracja FDCAN (identyczna jak w działającym SetDuty)
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Pakowanie danych (Big Endian)
    TxData[0] = (uint8_t)((send_val >> 24) & 0xFF);
    TxData[1] = (uint8_t)((send_val >> 16) & 0xFF);
    TxData[2] = (uint8_t)((send_val >> 8)  & 0xFF);
    TxData[3] = (uint8_t)(send_val & 0xFF);

    // Wysłanie do kolejki
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    }
}

void VESC_SetBrakeCurrent(uint8_t controller_id, float current_amps)
{
    // Hamowanie zawsze podajemy jako wartość dodatnią (moduł prądu)
    if (current_amps < 0.0f) current_amps = -current_amps;
    if (current_amps > 40.0f) current_amps = 40.0f; // Bezpiecznik

    int32_t send_val = (int32_t)(current_amps * 1000.0f);

    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[4];

    // Budowa ID ramki: [KOMENDA 0x02] [ID_STEROWNIKA]
    TxHeader.Identifier = (uint32_t)controller_id | ((uint32_t)0x02 << 8);

    // Konfiguracja FDCAN
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxData[0] = (uint8_t)((send_val >> 24) & 0xFF);
    TxData[1] = (uint8_t)((send_val >> 16) & 0xFF);
    TxData[2] = (uint8_t)((send_val >> 8)  & 0xFF);
    TxData[3] = (uint8_t)(send_val & 0xFF);

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    }
}

int32_t Get_Filtered_RPM(int32_t new_val) {
    static int32_t buffer[FILTER_SAMPLES];
    static uint8_t index = 0;
    static int32_t sum = 0;
    static uint8_t count = 0;

    // Odejmij starą wartość i dodaj nową
    sum -= buffer[index];
    buffer[index] = new_val;
    sum += buffer[index];

    index++;
    if (index >= FILTER_SAMPLES) index = 0;
    if (count < FILTER_SAMPLES) count++;

    return sum / count;
}

void CalculateVehicleSpeed(void)
{
    // Zabezpieczenie przed dzieleniem przez zero (gdybyś zapomniał ustawić define)
    #if MOTOR_POLE_PAIRS == 0
        return;
    #endif

    // 1. Pobierz RPM (wartość bezwzględna)
    float rpmL = (float)vescL.rpm;
    float rpmR = (float)vescR.rpm;

    if (rpmL < 0) rpmL = -rpmL;
    if (rpmR < 0) rpmR = -rpmR;

    // 2. Średnia obrotów elektrycznych (ERPM)
    float avgErpm = (rpmL + rpmR) / 2.0f;

    // 3. Przeliczenie na obroty mechaniczne (RPM)
    float mechRpm = avgErpm / MOTOR_POLE_PAIRS;

    // 4. Przeliczenie na prędkość liniową (m/min) -> Obwód = PI * D
    float speedMetersPerMin = mechRpm * (M_PI * WHEEL_DIAMETER_M);

    // 5. Przeliczenie na km/h: (m/min * 60) / 1000
    vehicleSpeedKmh = (speedMetersPerMin * 60.0f) / 1000.0f;
}

void MotorControl_Task_Entry(void)
{
	 uint8_t current_gear = 1;
	        const uint8_t MAX_GEAR = 16;

	        // Parametry automatu
	        const int32_t UPSHIFT_RPM = 7000;
	        const int32_t DOWNSHIFT_RPM = 4000;
	        const uint32_t SHIFT_DELAY = 1000;

	        // Zmienne operacyjne
	        uint32_t timer_upshift = 0;
	        uint32_t timer_downshift = 0;
	        static float current_drive_filtered = 0.0f;
	        static float current_drag_filtered = 0.0f;


    // Pętla nieskończona zadania
    while (1)
    {
    	// 1. ODCZYT I FILTRACJA RPM
    	CalculateVehicleSpeed();
    	MPU6050_Read_All(&hi2c2, &MPU6050);
    		        int32_t rpm_raw = vescM.rpm;
    		        if (rpm_raw < 0) rpm_raw = 0;
    		        int32_t rpm_filtered = Get_Filtered_RPM(rpm_raw);

    		        // 2. LOGIKA ZMIANY BIEGÓW (Ulepszona o histerezę)
    		        static uint32_t last_tick = 0;
    		        uint32_t current_tick = HAL_GetTick();
    		        if(last_tick == 0) last_tick = current_tick;
    		        uint32_t dt_ms = current_tick - last_tick;
    		        last_tick = current_tick;

    		        // Zabezpieczenie przed dzieleniem przez zero
    		        if (dt_ms == 0) dt_ms = 1;

    		        if (rpm_filtered > 200)
    		        {
    		            // Automat biegów (Twój kod z poprzednich iteracji jest OK)
    		            if (rpm_filtered > UPSHIFT_RPM) {
    		                timer_upshift += dt_ms;
    		                timer_downshift = 0;
    		                if (timer_upshift > SHIFT_DELAY) {
    		                    if (current_gear < MAX_GEAR) current_gear++;
    		                    timer_upshift = 0;
    		                }
    		            }
    		            else if (rpm_filtered < 2000) {
    		               timer_upshift = 0; // Reset dopiero przy niskich obrotach
    		               if (rpm_filtered < DOWNSHIFT_RPM) {
    		                   timer_downshift += dt_ms;
    		                   if (timer_downshift > SHIFT_DELAY) {
    		                       if (current_gear > 1) current_gear--;
    		                       timer_downshift = 0;
    		                   }
    		               }
    		            }
    		        } else {
    		             current_gear = 1;
    		             timer_upshift = 0;
    		             timer_downshift = 0;
    		        }
    		        float target_drive = 0.0f;
    		              float target_drag = 0.0f;

    		              static float virtual_speed = 0.0f;

    		              // --- PARAMETRY ---
    		              const float CLUTCH_STIFFNESS = 0.003f;
    		              const float VIRTUAL_ACCEL_FACTOR = 0.15f;
    		              const float COASTING_DECAY = 0.996f;

    		              // Parametr Mocy: Ile Amperów na koła daje wciśnięty gaz (RPM) na danym biegu?
    		              // Zwiększ to, jeśli rower jest za słaby przy szybkiej jeździe.
    		              const float DRIVE_POWER_FACTOR = 2.5f;

    		              if (rpm_filtered > 100)
    		              {
    		                  float speed_delta = (float)rpm_filtered - virtual_speed;

    		                  if (speed_delta > 0)
    		                  {
    		                      // === SYTUACJA A: JAZDA (Aktywne pedałowanie) ===

    		                      // 1. Fizyka wirtualnego koła (goni nogi)
    		                      virtual_speed += (speed_delta * VIRTUAL_ACCEL_FACTOR);

    		                      // 2. OPÓR (Generator) - Zostawiamy miękki, bo to było dobre
    		                      float gear_base_drag = (current_gear - 1) * 0.5f;
    		                      target_drag = (speed_delta * CLUTCH_STIFFNESS) + gear_base_drag;

    		                      // 3. NAPĘD (KOŁA) - TU JEST POPRAWKA!
    		                      // Wcześniej zależało to tylko od 'speed_delta' (dlatego znikało przy stałej jeździe).
    		                      // Teraz zależy od FAKTYCZNEGO RPM i BIEGU.

    		                      // A. Baza: Im szybciej kręcisz (RPM) i wyższy bieg, tym więcej mocy.
    		                      // Normalizacja: (RPM / 7000) * (Bieg * MocNaBieg)
    		                      float rpm_ratio = (float)rpm_filtered / 7000.0f;
    		                      if(rpm_ratio > 1.0f) rpm_ratio = 1.0f;

    		                      float cruising_power = rpm_ratio * (current_gear * DRIVE_POWER_FACTOR);

    		                      // B. Boost: Dodatek za "deptanie" (speed_delta)
    		                      float acceleration_boost = speed_delta * 0.01f;

    		                      // Suma
    		                      target_drive = cruising_power + acceleration_boost;

    		                      // Start Assist (żeby ruszył z miejsca)
    		                      if (virtual_speed < 1000) target_drive += 5.0f;
    		                  }
    		                  else
    		                  {
    		                      // === SYTUACJA B: WOLNOBIEG (Luz) ===
    		                      target_drag = 0.0f;
    		                      target_drive = 0.0f;
    		                      virtual_speed *= COASTING_DECAY;
    		                  }

    		                  // --- LIMITY ---
    		                  // Generator (Opór)
    		                  if (target_drag > 15.0f) target_drag = 15.0f;

    		                  // Silniki (Napęd) - Tu masz 40A w VESC, więc w kodzie też pozwól na więcej
    		                  if (target_drive > 35.0f) target_drive = 35.0f;

    		                  if (virtual_speed < 0) virtual_speed = 0;
    		              }
    		              else
    		              {
    		                  target_drag = 0.0f;
    		                  target_drive = 0.0f;
    		                  virtual_speed *= 0.98f;
    		              }

    		              // 5. FILTROWANIE WYJŚCIA
    		              current_drive_filtered = (current_drive_filtered * 0.85f) + (target_drive * 0.15f);
    		              current_drag_filtered = (current_drag_filtered * 0.85f) + (target_drag * 0.15f);

    		        // 6. WYSYŁANIE DO VESC
    		        VESC_SetCurrent(VESC_ID_L, current_drive_filtered);
    		        HAL_Delay(1);
    		        VESC_SetCurrent(VESC_ID_R, current_drive_filtered);
    		        HAL_Delay(1);
    		        VESC_SetBrakeCurrent(VESC_ID_M, current_drag_filtered);
    		        AngX = MPU6050.KalmanAngleX;
    		        AngY = MPU6050.KalmanAngleY;
    		        printf("%lu,%d,%lu,%ld,%.2f,%.2f,",
    		      	                     HAL_GetTick(),            // Czas
    		      	                     current_gear,             // Bieg
    		      	                     timer_upshift,            // Timer
    		      	                     rpm_filtered,             // RPM Filtrowane
    		      	                     current_drive_filtered,   // Prąd Zadany Napęd
    		      	                     current_drag_filtered     // Prąd Zadany Opór
    		      	              );

    		      	              // 2. VESC LEWY (L) - Struktura VescData_t
    		      	              printf("%ld,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.2f,%.2f,%ld,%.1f,",
    		      	                     vescL.rpm, vescL.current_motor, vescL.duty_cycle,
    		      	                     vescL.amp_hours, vescL.amp_hours_chg, vescL.watt_hours, vescL.watt_hours_chg,
    		      	                     vescL.temp_fet, vescL.temp_motor, vescL.current_in, vescL.pid_pos, vescL.tacho_value, vescL.v_in
    		      	              );

    		      	              // 3. VESC ŚRODKOWY (M) - Generator
    		      	              printf("%ld,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.2f,%.2f,%ld,%.1f,",
    		      	                     vescM.rpm, vescM.current_motor, vescM.duty_cycle,
    		      	                     vescM.amp_hours, vescM.amp_hours_chg, vescM.watt_hours, vescM.watt_hours_chg,
    		      	                     vescM.temp_fet, vescM.temp_motor, vescM.current_in, vescM.pid_pos, vescM.tacho_value, vescM.v_in
    		      	              );

    		      	              // 4. VESC PRAWY (R)
    		      	              printf("%ld,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.2f,%.2f,%ld,%.1f,",
    		      	                     vescR.rpm, vescR.current_motor, vescR.duty_cycle,
    		      	                     vescR.amp_hours, vescR.amp_hours_chg, vescR.watt_hours, vescR.watt_hours_chg,
    		      	                     vescR.temp_fet, vescR.temp_motor, vescR.current_in, vescR.pid_pos, vescR.tacho_value, vescR.v_in
    		      	              );

    		      	              // 5. MPU6050 (Z poprawnymi nazwami Kalman)
    		      	              // Format: Ax, Ay, Az, Gx, Gy, Gz, Temp, KalmanX, KalmanY
    		      	              // Używamy MPU6050.KalmanAngleX zgodnie z Twoim plikiem .h
    		      	              printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\r\n",
    		      	                     MPU6050.Ax, MPU6050.Ay, MPU6050.Az,
    		      	                     MPU6050.Gx, MPU6050.Gy, MPU6050.Gz,
    		      	                     MPU6050.Temperature,
    		      	                     MPU6050.KalmanAngleX,  // <--- POPRAWIONE
    		      	                     MPU6050.KalmanAngleY   // <--- POPRAWIONE
    		      	              );



    		      	        osDelay(1);
    }
}
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

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0xC0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
