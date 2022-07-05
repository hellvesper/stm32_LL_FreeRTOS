/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
        .name = "defaultTask",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ticksPerSec */
osThreadId_t ticksPerSecHandle;
const osThreadAttr_t ticksPerSec_attributes = {
        .name = "ticksPerSec",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void StartTicksPerSec(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);

unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {

}

__weak unsigned long getRunTimeCounterValue(void) {
    return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of ticksPerSec */
    ticksPerSecHandle = osThreadNew(StartTicksPerSec, NULL, &ticksPerSec_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
//        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTicksPerSec */
/**
* @brief Function implementing the ticksPerSec thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTicksPerSec */
void StartTicksPerSec(void *argument) {
    /* USER CODE BEGIN StartTicksPerSec */
    static uint8_t DutyCycle = 5;
    static uint8_t Direction = 1;
    TIM2->CCR1 = DutyCycle;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    /* Infinite loop */
    /*
     * CCR
     * ---- x 100 = x%
     * ARR
     *        x%
     * CCR = ---- x ARR
     *       100
     *
     */
    for (;;) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        if (Direction > 0) {
            if (DutyCycle < 90)
                DutyCycle += 10;
            else {
                DutyCycle = 90;
                Direction = 0;
            }
        } else {
            if (DutyCycle > 20)
                DutyCycle -= 10;
            else {
                DutyCycle = 20;
                Direction = 1;
            }

        }
//        TIM2->CCR1 = DutyCycle / 100 * (htim2.Init.Period + 1);
        TIM2->CCR1 = DutyCycle;
//        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        portENTER_CRITICAL();
        for (int i = 0; i < CHANNELS; ++i) {
            TicksSec[i] = TicksCounter[i];
            TicksCounter[i] = 0;
        }
        portEXIT_CRITICAL();
//        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

        osDelay(1000 / portTICK_PERIOD_MS);
    }
    /* USER CODE END StartTicksPerSec */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
        case GPIO_PIN_12:
            TicksCounter[0]++;
            break;
        case GPIO_PIN_13:
            TicksCounter[1]++;
            break;
        case GPIO_PIN_14:
            TicksCounter[2]++;
            break;
        case GPIO_PIN_15:
            TicksCounter[3]++;
            break;
        default:
            __NOP();
            break;
    }
}
/* USER CODE END Application */
