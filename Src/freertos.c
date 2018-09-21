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

#include "flash.h"
#include "mb.h"
#include "hw_config.h"
#include "UserModbusSlaver.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId MonitorTaskHandle;
uint32_t MonitorTaskBuffer[ 512 ];
osStaticThreadDef_t MonitorTaskControlBlock;
osThreadId FlashWritingTasHandle;
uint32_t FlashWritingTasBuffer[ 256 ];
osStaticThreadDef_t FlashWritingTasControlBlock;
osSemaphoreId MbTimSemHandle;
osStaticSemaphoreDef_t MbTimSemControlBlock;
osSemaphoreId MbTXSemHandle;
osStaticSemaphoreDef_t MbTXSemControlBlock;
osSemaphoreId FlashWriteSemHandle;
osStaticSemaphoreDef_t FlashWriteSemControlBlock;

/* USER CODE BEGIN Variables */
FlashQueue_typedef flash_queue_data = {0,0,0,0,0};
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartMonitorTask(void const * argument);
void StartFlashWritingTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of MbTimSem */
  osSemaphoreStaticDef(MbTimSem, &MbTimSemControlBlock);
  MbTimSemHandle = osSemaphoreCreate(osSemaphore(MbTimSem), 1);

  /* definition and creation of MbTXSem */
  osSemaphoreStaticDef(MbTXSem, &MbTXSemControlBlock);
  MbTXSemHandle = osSemaphoreCreate(osSemaphore(MbTXSem), 1);

  /* definition and creation of FlashWriteSem */
  osSemaphoreStaticDef(FlashWriteSem, &FlashWriteSemControlBlock);
  FlashWriteSemHandle = osSemaphoreCreate(osSemaphore(FlashWriteSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of MonitorTask */
  osThreadStaticDef(MonitorTask, StartMonitorTask, osPriorityNormal, 0, 512, MonitorTaskBuffer, &MonitorTaskControlBlock);
  MonitorTaskHandle = osThreadCreate(osThread(MonitorTask), NULL);

  /* definition and creation of FlashWritingTas */
  osThreadStaticDef(FlashWritingTas, StartFlashWritingTask, osPriorityNormal, 0, 256, FlashWritingTasBuffer, &FlashWritingTasControlBlock);
  FlashWritingTasHandle = osThreadCreate(osThread(FlashWritingTas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartMonitorTask function */
void StartMonitorTask(void const * argument)
{

  /* USER CODE BEGIN StartMonitorTask */
  eMBInit(MB_RTU, MB_ADDRESS);///>Modbus RTU initialisation
  eMBEnable();///>Modbus RTU activating
  /* Infinite loop */
  for(;;)
  {
    if(xSemaphoreTake(MbTimSemHandle, 1 / portTICK_RATE_MS) == pdTRUE )
    {
       pxMBPortCBTimerExpired();    
    }
    eMBPoll(); ///> polling  
    
  }
  /* USER CODE END StartMonitorTask */
}

/* StartFlashWritingTask function */
void StartFlashWritingTask(void const * argument)
{
  /* USER CODE BEGIN StartFlashWritingTask */
 FlashQueue_typedef *queue_data = &flash_queue_data;
 union FloatChar FlWd;

  /* Infinite loop */
  for(;;)
  {
    if(xSemaphoreTake(FlashWriteSemHandle, portMAX_DELAY  )== pdTRUE)///> waiting for writing flash semaphore
    {
      if(queue_data->mask > 0)
      {
       if((queue_data->mask & HOLDING_REG1_H_MASK)||(queue_data->mask & HOLDING_REG1_L_MASK))
       {
         /* First, let's read and save data from address which is used as holding register1 */
         ReadFlash((void *)&FlWd.dwd, sizeof(uint32_t), mbHoldingReg1HIGH);
         if(queue_data->mask & HOLDING_REG1_H_MASK)
                      FlWd.wd[0] = queue_data->reg1_h;
         if(queue_data->mask & HOLDING_REG1_L_MASK)
                      FlWd.wd[1] = queue_data->reg1_l;
         /*we write only data which was changed*/
         WriteFlash((void*)FlWd.dwd, sizeof(uint32_t), mbHoldingReg1HIGH);
       }
       if((queue_data->mask & HOLDING_REG2_H_MASK)||(queue_data->mask & HOLDING_REG2_L_MASK))
       {
         ReadFlash((void *)&FlWd.dwd, sizeof(uint32_t), mbHoldingReg2HIGH);
         if(queue_data->mask & HOLDING_REG2_H_MASK)
                      FlWd.wd[0] = queue_data->reg2_h;
         if(queue_data->mask & HOLDING_REG2_L_MASK)
                      FlWd.wd[1] = queue_data->reg2_l;
         WriteFlash((void*)FlWd.dwd, sizeof(uint32_t), mbHoldingReg2HIGH);
       } 

      }
    }
  }
  /* USER CODE END StartFlashWritingTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
