#ifndef __DEALY_H__
#define __DELAY_H__
#include "stm32f1xx_hal.h"

void DWT_Init(void);
void Delay_us(uint32_t xus);
void Delay_ms(uint32_t xms);

#define delay_ms Delay_ms

#endif


