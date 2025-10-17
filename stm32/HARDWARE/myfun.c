#include "myfun.h"

void IO2driver(uint8_t status)
{
	switch(status)
	{
		case 1://没有采样
		{
			HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, GPIO_PIN_RESET); //低电平
		} break;
		case 2://正在采样
		{
			static uint8_t tim3count;
			tim3count++;
			if(tim3count>98)
			{
				tim3count = 0;
				HAL_GPIO_TogglePin(IO2_GPIO_Port, IO2_Pin);
			}
		} break;
		case 3://采样完成
		{
			HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, GPIO_PIN_SET); //高电平
		} break;
	}
}
//找出sram四个数中相同的数
uint8_t Sram_Read_Compare(uint8_t packnum,uint16_t i)
{
	static uint32_t a;
	a=(packnum-1)*5000+4*i;
	 if (adcData[a] == adcData[a+1] || adcData[a] == adcData[a+2] || adcData[a] == adcData[a+3]) {
        return adcData[a];
    } else if (adcData[a+1] == adcData[a+2] || adcData[a+1] == adcData[a+3]) {
        return adcData[a+1];
    } else {
        return adcData[a+2];
    }
}









