#include "ads8867.h"

#define SAMPLE_COUNT 400000

extern SPI_HandleTypeDef hspi1;
extern QSPI_HandleTypeDef hqspi;

uint8_t adcData[800000] __attribute__((at(0x90000000)));
uint8_t adcData1[2];
volatile uint32_t sampleIndex = 0;
volatile uint32_t sampleIndex2 = 0;
uint8_t samplegroup=0;//采样组，2500一组
uint16_t adcData2 = 0;

// 定时器中断处理函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (sampleIndex < SAMPLE_COUNT*2) {
			adcData2 = ADS8867_ReadValue();
			adcData1[0]=adcData2/256;
			QSPI_Write(&hqspi, sampleIndex, adcData1, 1);//写四次有BUG，写三次
			sampleIndex = sampleIndex+1;
			QSPI_Write(&hqspi, sampleIndex, adcData1, 1);
			sampleIndex = sampleIndex+1;
			QSPI_Write(&hqspi, sampleIndex, adcData1, 1);
			sampleIndex = sampleIndex+2;
//			QSPI_Write(&hqspi, sampleIndex, adcData1, 1);
//			sampleIndex = sampleIndex+1;
			adcData1[0]=adcData2%256;
			QSPI_Write(&hqspi, sampleIndex, adcData1, 1);//写四次
			sampleIndex = sampleIndex+1;
			QSPI_Write(&hqspi, sampleIndex, adcData1, 1);
			sampleIndex = sampleIndex+1;
			QSPI_Write(&hqspi, sampleIndex, adcData1, 1);
			sampleIndex = sampleIndex+2;
//			QSPI_Write(&hqspi, sampleIndex, adcData1, 1);
//			sampleIndex = sampleIndex+1;
        } else {
			sampleIndex=0;
            HAL_TIM_Base_Stop_IT(htim);  // 采集完成后停止定时器
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); 	//红灯灭
			tim_endflag=1;
			calistatus = 3;//采样完成
			LY68L6400_EnableMemoryMappedMode();
        }
    }
}
uint32_t k;
uint16_t zerobuf[50000];
void adcdriver(void)//测试用
{
	while(sampleIndex<800001)
	{
		if(sampleIndex<800000)
		{
			static uint16_t count=0;
			count++;
			adcData1[0] = count/256;
			QSPI_Write(&hqspi, sampleIndex+1, adcData1, 1);
			sampleIndex = sampleIndex+4;
			adcData1[0] = count%256;
			QSPI_Write(&hqspi, sampleIndex+1, adcData1, 1);
			sampleIndex = sampleIndex+4;
		}
		else
		{
			LY68L6400_EnableMemoryMappedMode();
			sampleIndex = sampleIndex+4;
			
			for (k = 50000; k < 100000; k++)
			{
				zerobuf[k-50000] = adcData[8*k+1]*256+adcData[8*k+5];
			}
			sampleIndex = sampleIndex+4;
		}
	}
   

}

static void DIN_Set(void) 
{
	HAL_GPIO_WritePin(DIN_GPIO_Port, DIN_Pin, GPIO_PIN_SET);
}

static void CONVST_Reset(void) 
{ 
	HAL_GPIO_WritePin(CONVST_GPIO_Port, CONVST_Pin, GPIO_PIN_RESET);
}

static void CONVST_Set(void) 
{ 
	HAL_GPIO_WritePin(CONVST_GPIO_Port, CONVST_Pin, GPIO_PIN_SET);
}
uint16_t adcReadBuffer = 0;
//	uint8_t readBuffer = 0;
uint8_t readBuffer[2];
uint16_t ADS8867_ReadValue(void)
{
	

	DIN_Set();
	CONVST_Set();
	delay_600ns();
	//HAL_Delay(1);
	CONVST_Reset();

	HAL_SPI_Receive(&hspi1, readBuffer, 2, 0xFFFF);
	adcReadBuffer = readBuffer[0] + readBuffer[1]*256;

	return adcReadBuffer;
}

void delay_600ns(void)
{
	for (uint8_t i = 0; i < 50; i++)
		__nop();//6.25ns

}

void ads8867Init(void)
{
	DIN_Set();
	CONVST_Reset();
}
















