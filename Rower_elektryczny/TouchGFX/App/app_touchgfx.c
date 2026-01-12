/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_touchgfx.c
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

#include "app_touchgfx.h"
#include "cmsis_os.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void touchgfx_init(void);
extern void touchgfx_taskEntry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void TouchGFX_Task(void *argument)
{
  /* USER CODE BEGIN TouchGFX_Task */

  // Krótkie opóźnienie na start, aby system się ustabilizował
  osDelay(100);

  // Uruchomienie głównej pętli TouchGFX
  // Ta funkcja normalnie NIE POWINNA nigdy wrócić.
  touchgfx_taskEntry();

  // === SAFETY LOOP ===
  // Jeśli touchgfx_taskEntry() wróci (np. z powodu błędu),
  // ta pętla zapobiegnie skokowi do prvCheckTasksWaitingTermination.
  // Jeśli debugger tu trafi, oznacza to, że TouchGFX się wyłożył.
  for(;;)
  {
      osDelay(1000);
  }
  /* USER CODE END TouchGFX_Task */
}

void MX_TouchGFX_PreOSInit(void)
{
  /* USER CODE BEGIN MX_TouchGFX_PreOSInit */
  // Tutaj można dodać inicjalizację QSPI, jeśli nie jest w main()
  /* USER CODE END MX_TouchGFX_PreOSInit */
}

void MX_TouchGFX_Init(void)
{
  /* USER CODE BEGIN MX_TouchGFX_Init */
  // Inicjalizacja frameworku TouchGFX (HAL, OSWrappers)
  touchgfx_init();
  /* USER CODE END MX_TouchGFX_Init */
}

void MX_TouchGFX_Process(void)
{
  /* USER CODE BEGIN MX_TouchGFX_Process */
  touchgfx_taskEntry();
  /* USER CODE END MX_TouchGFX_Process */
}
