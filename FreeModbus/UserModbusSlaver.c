#include "UserModbusSlaver.h"

#include "hw_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "flash.h"
/* ----------------------- Variables ---------------------------------*/
USHORT   usDiscreteInputStart                             = DISCRETE_INPUT_START;
UCHAR    usDiscreteInputBuf[DISCRETE_INPUT_NDISCRETES/8]  ;
USHORT   usCoilStart                                      = COIL_START;
UCHAR    usCoilBuf[COIL_NCOILS/8]                         ;
USHORT   usRegInputStart                                  = REG_INPUT_START;
USHORT   usRegInputBuf[REG_INPUT_NREGS]                   ;
USHORT   usRegHoldingStart                                = REG_HOLDING_START;
USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]               ;

union FloatChar FlCh;
extern osSemaphoreId FlashWriteSemHandle;///> semaphore Handle
extern FlashQueue_typedef flash_queue_data;///> buf for holding registers




//******************************                 **********************************

//**********************************************************************************
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    if( ( usAddress >= REG_INPUT_START )///> checking validation of regiser addresses
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        /* saving ADC value to Input registers 40001 and 40002*/
        usRegInputBuf[MB_INPUT_REG_FL_H] = FlCh.wd[0];
        usRegInputBuf[MB_INPUT_REG_FL_L] = FlCh.wd[1];

        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}
//*******************************                  *********************************

//**********************************************************************************
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    int isQueue = FALSE;
    uint16_t * p_temp;
    uint16_t temp;
    
    
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
              /*Checking register address and reading Flash*/
              if(iRegIndex == MB_HOLDING_REG1_FL_H)                 
                  temp = FLASH_read(mbHoldingReg1HIGH);  
               else if(iRegIndex == MB_HOLDING_REG1_FL_L)
                  temp =  FLASH_read(mbHoldingReg1LOW);
               else if(iRegIndex == MB_HOLDING_REG2_FL_H)
                  temp =  FLASH_read(mbHoldingReg2HIGH);
               else if(iRegIndex == MB_HOLDING_REG2_FL_L)
                  temp =  FLASH_read(mbHoldingReg2LOW);
               else  
                  temp = usRegHoldingBuf[iRegIndex];
              
                *pucRegBuffer++ = ( unsigned char )( temp >> 8 );
                *pucRegBuffer++ = ( unsigned char )( temp & 0xFF );
                iRegIndex++;
                usNRegs--;
              
              /*  *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;*/
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
              
            while( usNRegs > 0 )
            {
              /*Checking register 60005  and packing buf for saving to flash*/
              if((iRegIndex == MB_HOLDING_REG1_FL_H)||(iRegIndex == MB_HOLDING_REG1_FL_L)||
                 (iRegIndex == MB_HOLDING_REG2_FL_H)||(iRegIndex == MB_HOLDING_REG2_FL_L))
              {
                memset(&flash_queue_data, 0, sizeof(flash_queue_data));
                isQueue = TRUE;
              }
               if(iRegIndex == MB_HOLDING_REG1_FL_H) 
               {                 
                  p_temp = &flash_queue_data.reg1_h;
                  flash_queue_data.mask |=  HOLDING_REG1_H_MASK;  
               }
               else if(iRegIndex == MB_HOLDING_REG1_FL_L)
               {
                  p_temp = &flash_queue_data.reg1_l;
                  flash_queue_data.mask |=  HOLDING_REG1_L_MASK;
               }
               else if(iRegIndex == MB_HOLDING_REG2_FL_H)
               {
                  p_temp = &flash_queue_data.reg2_h;
                  flash_queue_data.mask |=  HOLDING_REG2_H_MASK;
               }
               else if(iRegIndex == MB_HOLDING_REG2_FL_L)
               {
                  p_temp = &flash_queue_data.reg2_l;
                  flash_queue_data.mask |=  HOLDING_REG2_L_MASK;
               }
               else
                  p_temp = &usRegHoldingBuf[iRegIndex];
               
               *p_temp = *pucRegBuffer++ << 8; 
               *p_temp |= *pucRegBuffer++;
               iRegIndex++;
               usNRegs--;
              /*   
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;*/
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    if(isQueue == TRUE)
         xSemaphoreGive( FlashWriteSemHandle );///> signalling
    return eStatus;
}
//****************************                     ********************************

//**********************************************************************************
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex , iRegBitIndex , iNReg;
    iNReg =  usNCoils / 8 + 1;        //
    if( ( usAddress >= COIL_START ) &&
        ( usAddress + usNCoils <= COIL_START + COIL_NCOILS ) )
    {
        iRegIndex    = ( int )( usAddress - usCoilStart ) / 8 ;    //
		iRegBitIndex = ( int )( usAddress - usCoilStart ) % 8 ;	   //
        switch ( eMode )
        {
            /* Pass current coil values to the protocol stack. */
        case MB_REG_READ:
            while( iNReg > 0 )
            {
				*pucRegBuffer++ = xMBUtilGetBits(&usCoilBuf[iRegIndex++] , iRegBitIndex , 8);
                iNReg --;
            }
			pucRegBuffer --;
			usNCoils = usNCoils % 8;                        //
			*pucRegBuffer = *pucRegBuffer <<(8 - usNCoils); //
			*pucRegBuffer = *pucRegBuffer >>(8 - usNCoils);
            break;

            /* Update current coil values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while(iNReg > 1)									 //
            {
				xMBUtilSetBits(&usCoilBuf[iRegIndex++] , iRegBitIndex  , 8 , *pucRegBuffer++);
                iNReg--;
            }
			usNCoils = usNCoils % 8;                            //
			xMBUtilSetBits(&usCoilBuf[iRegIndex++] , iRegBitIndex  , usNCoils , *pucRegBuffer++);
			break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}
//****************************                      ********************************

//**********************************************************************************
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex , iRegBitIndex , iNReg;
	iNReg =  usNDiscrete / 8 + 1;        //
    if( ( usAddress >= DISCRETE_INPUT_START )
        && ( usAddress + usNDiscrete <= DISCRETE_INPUT_START + DISCRETE_INPUT_NDISCRETES ) )
    {
        iRegIndex    = ( int )( usAddress - usDiscreteInputStart ) / 8 ;    //
		iRegBitIndex = ( int )( usAddress - usDiscreteInputStart ) % 8 ;	   //
	    while( iNReg > 0 )
        {
			*pucRegBuffer++ = xMBUtilGetBits(&usDiscreteInputBuf[iRegIndex++] , iRegBitIndex , 8);
            iNReg --;
        }
		pucRegBuffer --;
		usNDiscrete = usNDiscrete % 8;                     //
		*pucRegBuffer = *pucRegBuffer <<(8 - usNDiscrete); //
		*pucRegBuffer = *pucRegBuffer >>(8 - usNDiscrete);
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}
