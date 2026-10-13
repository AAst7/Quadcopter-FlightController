#include "LED.h"
#include "Variable.h"
#include "PWM.h"


static uint16_t LED_Time_Last = 0;      //上一次设定的半周期时长,用于识别闪烁速率切换
static uint16_t LED_Time_Count = 0;     //当前半周期已经过的ms数
static uint8_t LED_Light_Dark_Bit = 0;  //1亮，0暗

void LED_Light(uint8_t LED_bit)
{
    if(LED_bit){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);}
    else{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);}
}


//每1ms调用一次,LED_Time_number为半周期时长(ms)
void LED_Time(uint16_t LED_Time_number)
{
    if(LED_Time_number == 0) LED_Time_number = 1;   //传0会导致计数器永远追不上

    if(LED_Time_number != LED_Time_Last)            //闪烁速率变了(例如500ms切50ms),
    {                                               //旧计数值可能大于新周期,若继续用
        LED_Time_Last = LED_Time_number;            //"=="比较,要等计数器溢出才会翻转,
        LED_Time_Count = 0;                         //表现为灯卡死约65s,所以必须重新计时
    }

    if(++LED_Time_Count >= LED_Time_number)
    {
        LED_Time_Count = 0;
        LED_Light_Dark_Bit = (LED_Light_Dark_Bit == 0) ? 1 : 0;
    }
    LED_Light(LED_Light_Dark_Bit);
}





