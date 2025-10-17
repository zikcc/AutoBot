#ifndef __ADS8867_H
#define __ADS8867_H

#include "main.h"

#define Conversation_Relay 100

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Read_ADC_Sample(void);
void adcdriver(void);

uint16_t ADS8867_ReadValue(void);
void delay_600ns(void);
void ads8867Init(void);

#endif
