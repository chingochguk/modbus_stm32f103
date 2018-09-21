#ifndef __flash_H
#define __flash_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"  
#include "stm32f1xx.h"
//#include "stm32f10x.h"
//#include "stm32f10x_flash.h"
   
typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;



uint32_t GetMemoryMask(uint32_t FlashDestination);
uint32_t FLASH_PagesMask(__IO uint32_t Size);
void     Erase_Memory(uint32_t FlashDestination, uint32_t NbrOfPage);
uint32_t WriteFlash(void * buf, uint32_t length, uint32_t PageAddr);
uint32_t ReadFlash(void * buf, uint32_t length, uint32_t PageAddr);
uint32_t FLASH_DisableWriteProtectionPages(uint32_t UserMemoryMask);
uint32_t FLASH_read(uint32_t address);

#ifdef __cplusplus
}
#endif
#endif /*__flash_H */