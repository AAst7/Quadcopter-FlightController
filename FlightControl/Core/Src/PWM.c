#include "main.h"
#include "tim.h"



void SET_Compare1(uint16_t Compare1)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Compare1);
}


void SET_Compare2(uint16_t Compare2)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Compare2);
}

void SET_Compare3(uint16_t Compare3)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Compare3);
}

void SET_Compare4(uint16_t Compare4)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, Compare4);
}

