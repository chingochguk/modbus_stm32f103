#ifndef	USER_MODBUS_SLAVER_H
#define USER_MODBUS_SLAVER_H
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
/* ----------------------- Defines ------------------------------------------*/
#define DISCRETE_INPUT_START        1
#define DISCRETE_INPUT_NDISCRETES   16
#define COIL_START                  1
#define COIL_NCOILS                 64
#define REG_INPUT_START             1
#define REG_INPUT_NREGS             100
#define REG_HOLDING_START           1
#define REG_HOLDING_NREGS           100

//===================================================
#define          HD_RESERVE                     0		  
#define          HD_CPU_USAGE_MAJOR             1         
#define          HD_CPU_USAGE_MINOR             2         
//===================================================
#define          IN_RESERVE                     0		  

//=======================================================
#define          CO_RESERVE                     2		  
//====================================================
#define          DI_RESERVE                     1		

#define          HOLDING_REG1_H_MASK            0x01
#define          HOLDING_REG1_L_MASK            0x02
#define          HOLDING_REG2_H_MASK            0x04
#define          HOLDING_REG2_L_MASK            0x08
/*
#ifndef UNALIGNED
  #define UNALIGNED   __packed
#endif

#pragma pack(push, 1)*/
union FloatChar {

  float    fl;
  uint32_t dwd;
  uint16_t wd[2];
  uint8_t  ch[4];
};

typedef struct {

  uint16_t reg1_h;///> holding register high 
  uint16_t reg1_l;
  uint16_t reg2_h;
  uint16_t reg2_l;
  uint16_t mask; ///> mask
}FlashQueue_typedef;

/*--------------------------Extern Functions------------------------------------*/
extern UCHAR xMBUtilGetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits );
extern void  xMBUtilSetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits,UCHAR ucValue );
//#pragma pack(pop)
#endif
