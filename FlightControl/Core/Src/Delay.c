#include "Delay.h"

//==============微秒延时====================

//用内核自带的调试计数器DWT延时微秒,不用SysTick，因为会破坏HAL的1ms tick
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;    //使能调试计数器
    DWT->CYCCNT = 0;    //清空计数器
    DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;    //使能计数器
}
void Delay_us(uint32_t xus)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t us_ticks = (SystemCoreClock / 1000000) * xus;      //72tick/1us
    while(DWT->CYCCNT - start < us_ticks);       //等待us_ticks个tick
}

//==============毫秒延时====================
void Delay_ms(uint32_t xms)
{
    HAL_Delay(xms);
}


