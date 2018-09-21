#include "flash.h"
#include "hw_config.h"
#include "stm32f1xx_hal_flash_ex.h"


/*******************************************************************************
***              ѕроверка на защиту от записи и ее отключение                ***   
***                         0    ошибка                                      ***
***                         1    нет защиты                                  ***
***                         перезагрузка прибора - была защита               ***
*******************************************************************************/
/*uint32_t FLASH_DisableWriteProtectionPages(uint32_t UserMemoryMask)
{
  uint32_t useroptionbyte = 0, WRPR = 0;
  uint8_t userconfig;
  HAL_StatusTypeDef status = HAL_BUSY;

   userconfig = OB_IWDG_SW | OB_STOP_NO_RST | OB_STDBY_NO_RST; 
   
   //читаем  Write Protection Option Byte
   WRPR = FLASH->WRPR;

  // Test if user memory is write protected 
  if ((WRPR & UserMemoryMask) != UserMemoryMask)
  {
    //читаем User Option Byte и сдвигаем дл€ удобства
    useroptionbyte = (FLASH->OBR >> 2);

    UserMemoryMask |= WRPR;

    if(HAL_FLASHEx_OBErase()!= HAL_OK)
                               return 0;

    if (UserMemoryMask != 0xFFFFFFFF)
    {
      status = FLASH_OB_EnableWRP((uint32_t)~UserMemoryMask);
    }

    // Test if user Option Bytes are programmed 
    if ((useroptionbyte & 0x07) != 0x07)
    { 
      // Restore user Option Bytes 
      if ((useroptionbyte & 0x01) == 0x0)
      {
        userconfig = OB_IWDG_HW;
      }
      if ((useroptionbyte & 0x02) == 0x0)
      {
        userconfig = userconfig | OB_STOP_RST;
      }
      if ((useroptionbyte & 0x04) == 0x0)
      {
        userconfig = userconfig | OB_STDBY_RST;
      }
      //сдвигаем согласно расположению в регистре
      userconfig = (userconfig << 2);      
      FLASH_OB_UserConfig(userconfig);
    }

    if (status == HAL_OK)
    {
      // Generate System Reset to load the new option byte values 
      NVIC_SystemReset();
    }
    else  return 0;
  }
  else  return 1;
 return 0;
}
*/
/*******************************************************************************
***                         Custom YS                                        ***
***           „тение данных из флэш пам€ти по 4 байта                        ***                 
*******************************************************************************/
uint32_t FLASH_read(uint32_t address)
{
  return (*(__IO uint32_t*) address);
}


/*******************************************************************************
                         
*******************************************************************************/
uint32_t GetMemoryMask(uint32_t FlashDestination)
{
  uint32_t BlockNbr = 0;
  /* Get the number of block (4 or 2 pages) from where the user program will be loaded */
  BlockNbr = (FlashDestination - 0x08000000) >> 12;
  /* Compute the mask to test if the Flash memory, where the user program will be
  loaded, is write protected */
  uint32_t UserMemoryMask = ((uint32_t)~((1 << BlockNbr) - 1));
  if (BlockNbr < 62)
  {
   UserMemoryMask = ((uint32_t)~((1 << BlockNbr) - 1));
  }
  else
  {
   UserMemoryMask = ((uint32_t)0x80000000);
  }
  return UserMemoryMask;
}

/*******************************************************************************
                 считаем количество страниц Flash пам€ти дл€ стирани€
*******************************************************************************/
uint32_t FLASH_PagesMask(__IO uint32_t Size)
{
  uint32_t pagenumber = 0x0;
  uint32_t size = Size;

  if ((size % flPageSize) != 0)
  {
    pagenumber = (size / flPageSize) + 1;
  }
  else
  {
    pagenumber = size / flPageSize;
  }
  return pagenumber;
}


/*******************************************************************************
                ”дал€ем нужное количество станиц Flash пам€ти
*******************************************************************************/
void Erase_Memory(uint32_t FlashDestination, uint32_t NbrOfPage)
{
  uint32_t PageSize = flPageSize;

  /* Erase the FLASH pages */
  for (uint32_t EraseCounter = 0; (EraseCounter < NbrOfPage); EraseCounter++)
   {
     FLASH_PageErase(FlashDestination + (PageSize * EraseCounter));
   }
}


/*******************************************************************************
                         пишем во Flash
*******************************************************************************/
uint32_t WriteFlash(void * buf, uint32_t length, uint32_t PageAddr)
{
 uint32_t isOK  = 0;
 uint32_t RamSource;
 FLASH_OBProgramInitTypeDef flash_ob = {OPTIONBYTE_WRP,0,PageAddr,0,0,0,0,0};
 FLASH_EraseInitTypeDef flash_er = {FLASH_TYPEERASE_PAGES,0,PageAddr,0};
 
  HAL_FLASH_Unlock();
  if(HAL_FLASHEx_OBProgram(&flash_ob) != HAL_OK) return FALSE;
              
  //определ€ем количество страниц пам€ти
   flash_er.NbPages = FLASH_PagesMask(length);  
   //очищаем нужное кол-во страниц пам€ти
   if(HAL_FLASHEx_Erase(&flash_er, &RamSource)!= HAL_OK) return FALSE;
   if(RamSource!=0xFFFFFFFFU) return FALSE;

   RamSource = (uint32_t)(buf);
                     
   for (uint32_t j = 0;(j < length) && (PageAddr <  flLastPageAddress); j += 4)
   {
    /* Program the data received into STM32F10x Flash */
    isOK = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PageAddr, RamSource);
    if(isOK != HAL_OK)
                          return 0;
/*    if (*(uint32_t*)PageAddr != *(uint32_t*)RamSource)
     {
        // End session 
        isOK = FALSE;
        break;
     }*/
    PageAddr += 4;
    RamSource += 4;
   }
   return 1;
}


/*******************************************************************************
                        читаем Flash пам€ть
*******************************************************************************/
uint32_t ReadFlash(void * buf, uint32_t length, uint32_t PageAddr)
{
 uint32_t * RamSource = (uint32_t *)buf;
  
   for(uint32_t j = 0; (j < length) && (PageAddr <  flLastPageAddress); j += 4)
   {
    *RamSource = FLASH_read(PageAddr);
    PageAddr += 4;
    RamSource ++;
   }
   return 1;
}
