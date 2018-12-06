/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "semphr.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
SemaphoreHandle_t LEDMutex;
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void LED_BlueBlink (void * pvParameters);
void LED_RedBlink (void * pvParameters);
void LED_GreenBlink (void * pvParameters);
void Delay_Nonsense(uint32_t * DelayCounter, uint32_t const * TargetCount);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
void Delay_Nonsense(uint32_t * DelayCounter, uint32_t const * TargetCount)
{

	while (*DelayCounter <= *TargetCount)
	{
		*DelayCounter = *DelayCounter + 1;
	}

	*DelayCounter = 0;

}

void LED_BlueBlink (void * pvParameters)
{
	  const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
	  uint32_t BlueDelay = 0;
	  const uint32_t TargetCount = 160000;

	  for(;;)
	  {
		  for (int i = 0; i <10; i++)
		  {
			  vTaskDelay(xDelay);
		  }
		  if ( xSemaphoreTake (LEDMutex, (TickType_t ) 10 ) == pdTRUE)
		  {
		  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, 0);
		  vTaskDelay(xDelay);
		  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, 1);
		  Delay_Nonsense(&BlueDelay, &TargetCount);
		  vTaskDelay(xDelay);
		  xSemaphoreGive(LEDMutex);
		  }
	  }
}

void LED_RedBlink (void * pvParameters)
{
	  const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
	  uint32_t RedDelay = 0;
	  const uint32_t TargetCount = 160000;

	  for(;;)
	  {
		  for (int i = 0; i <10; i++)
		  {
			  vTaskDelay(xDelay);
		  }
		  if ( xSemaphoreTake (LEDMutex, (TickType_t ) 10 ) == pdTRUE)
		  {
		  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
		  vTaskDelay(xDelay);
		  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 1);
		  Delay_Nonsense(&RedDelay, &TargetCount);
		  vTaskDelay(xDelay);
		  xSemaphoreGive(LEDMutex);
		  }
	  }
}

void LED_GreenBlink (void * pvParameters)
{
	  const TickType_t xDelay = 250 / portTICK_PERIOD_MS;
	  uint32_t GreenDelay = 0;
	  const uint32_t TargetCount = 200000;
	  xSemaphoreGive(LEDMutex);
	  for(;;)
	  {
		  for (int i = 0; i <10; i++)
		  {
			  vTaskDelay(xDelay);
		  }
		  if ( xSemaphoreTake (LEDMutex, (TickType_t ) 10 ) == pdTRUE)
		  {
		  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, 0);
		  vTaskDelay(xDelay);
		  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, 1);
		  Delay_Nonsense(&GreenDelay, &TargetCount);
		  vTaskDelay(xDelay);
		  xSemaphoreGive(LEDMutex);
		  }
	  }
}

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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);



  xTaskCreate(LED_GreenBlink,
		  (const char * const)"led_green",
		  configMINIMAL_STACK_SIZE,
		  0,
		  3,
		  0);

  xTaskCreate(LED_RedBlink,
 		  (const char * const)"led_red",
 		  configMINIMAL_STACK_SIZE,
 		  0,
 		  2,
 		  0);

  xTaskCreate(LED_BlueBlink,
 		  (const char * const)"led_blue",
 		  configMINIMAL_STACK_SIZE,
 		  0,
 		  1,
 		  0);

  LEDMutex = xSemaphoreCreateMutex();


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
