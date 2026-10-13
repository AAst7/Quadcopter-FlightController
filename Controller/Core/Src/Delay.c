#include "Delay.h"

//==============微秒延时====================

//用内核自带的调试计数器DWT延时微秒,不用SysTick，因为会破坏HAL的1ms tick
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;    //使能调试计数器
    DWT->CYCCNT = 0;    //清空计数器
    DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;    //使能计数器
}

volatile uint32_t _SysDelay_CyclesPerUs = 72;   //72MHz主频参考值

void Delay_us(uint32_t xus)
{
    //【修复】不依赖DWT的纯循环延时(F1未接调试器时CYCCNT不递增会死循环)
    uint32_t loops = xus * 15;      //72MHz下约每us 15次空循环
    while (loops--) {
        __asm("nop");
    }
}

//==============毫秒延时====================
void Delay_ms(uint32_t xms)
{
    HAL_Delay(xms);
}