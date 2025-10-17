#ifndef __W25Q128_H
#define __W25Q128_H

#include "main.h"

#define  W25Qxx_Sector_Size       4096 

#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg1		0x05 
#define W25X_ReadStatusReg2		0x35 
#define W25X_ReadStatusReg3		0x15 
#define W25X_WriteStatusReg1    0x01 
#define W25X_WriteStatusReg2    0x31 
#define W25X_WriteStatusReg3    0x11 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 
#define W25X_Enable4ByteAddr    0xB7
#define W25X_Exit4ByteAddr      0xE9
#define W25X_SetReadParam		0xC0 
#define W25X_EnterQPIMode       0x38
#define W25X_ExitQPIMode        0xFF


typedef enum 
{
	W25Qxx_Not = 0        ,
	W25Qxx_16  =	0XEF14  ,
	W25Qxx_32  =	0XEF15  ,
	W25Qxx_64  =	0XEF16  ,
	W25Qxx_128 =	0XEF17  ,
} W25QxxTypeDef ;

/** 
  * @brief  W25Qxx Information Structure definition
  */ 
typedef struct
{
  W25QxxTypeDef chipType;          /*!< Specifies the card Type                          */
  uint32_t chipSectorNbr ;            /*!< Specifies the Card Capacity in sectors           */
  uint32_t chipSectorSize;            /*!< Specifies one sector size in bytes               */
  
	
  uint32_t fatfsSectorNbr  ;          /*!< Specifies the Card fatfs Capacity in sectors     */
  uint32_t fatfsSectorSize ;          /*!< Specifies fatfs sector size in bytes             */
}W25Qxx_FlashInfoTypeDef;



uint16_t W25Qxx_Read_ID(void);

void  W25Qxx_Write_Read_Check(uint32_t addr);//测试程序

#endif
