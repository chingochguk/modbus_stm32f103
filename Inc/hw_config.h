#ifndef __hw_config_H
#define __hw_config_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"   
   
#define TRUE                          1
#define FALSE                         0
   
#define DEBUG                
   
#define usedADC                       ADC1   // used ADC
#define mbTimer                       TIM1   // MODBUS delay timer
#define adcTimer                      TIM3   // ADC sampling rate Timer
#define mbUART                        USART1 // MODBUS UART
   
#define flPageSize                    1024
#define flMemBaseAddress              0x08000000
#define flLastPageAddress             flMemBaseAddress + (64*flPageSize) 
              
#define mbHoldingReg1HIGH             flMemBaseAddress + (62*flPageSize)   //62 page 
#define mbHoldingReg1LOW              flMemBaseAddress + (62*flPageSize)+ 2   
#define mbHoldingReg2HIGH             flMemBaseAddress + (61*flPageSize)   //61 page
#define mbHoldingReg2LOW              flMemBaseAddress + (61*flPageSize)+ 2   
#define flReservPage                  flMemBaseAddress + (60*flPageSize)   //60 page 
     
#define TEST_Pin LL_GPIO_PIN_7
#define TEST_GPIO_Port GPIOB
   
#define ENTER_CRITICAL_SECTION()      taskENTER_CRITICAL() 
#define EXIT_CRITICAL_SECTION()       taskEXIT_CRITICAL()

#define MB_ADDRESS                    0x01   

#define MB_INPUT_REG                  40001 //MB Input Register, where we hold ADC val 
#define MB_INPUT_REG_FL_H             0     //MB Input Register HIGH word
#define MB_INPUT_REG_FL_L             1     //MB Input Register LOW  word
   
#define MB_HOLDING_REG1               60004 //MB output AO Register 1 
#define MB_HOLDING_REG1_FL_H          4     //MB output AO Register 1 HIGH word
#define MB_HOLDING_REG1_FL_L          5     //MB output AO Register 1 LOW word
   
#define MB_HOLDING_REG2               60014 //MB output AO Register 2
#define MB_HOLDING_REG2_FL_H          14    //MB output AO Register 2 HIGH word
#define MB_HOLDING_REG2_FL_L          15    //MB output AO Register 2 LOW word
   
#ifdef __cplusplus
}
#endif
#endif /*__hw_config_H */