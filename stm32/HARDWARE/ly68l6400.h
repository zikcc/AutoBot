#ifndef __LY68L6400
#define __LY68L6400

#include "main.h"

HAL_StatusTypeDef QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
void QSPI_Send_CMD(uint32_t instruction,uint32_t address,uint32_t dummyCycles,uint32_t instructionMode,uint32_t addressMode,uint32_t addressSize,uint32_t dataMode);
uint8_t QSPI_Transmit(uint8_t* buf,uint32_t datalen);

HAL_StatusTypeDef QSPI_ReadID(QSPI_HandleTypeDef *hqspi, uint8_t *id);


void LY68_Write_Read_Check(void);

HAL_StatusTypeDef QSPI_Write(QSPI_HandleTypeDef *hqspi, uint32_t address, uint8_t *buffer, uint16_t length);
void LY68L6400_EnableMemoryMappedMode(void);

uint8_t QSPI_WriteManyData(uint8_t *pData, uint32_t WriteAddr, uint32_t Size);

#endif
