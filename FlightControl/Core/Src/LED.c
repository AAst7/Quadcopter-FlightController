#include "LED.h"
#include "Variable.h"
#include "PWM.h"


static uint16_t LED_Time_Light = 0;
static uint16_t LED_Time_Dark = 0;
static uint8_t LED_Light_Dark_Bit = 0;  //1亮，0暗

void LED_Light(uint8_t LED_bit)
{
    if(LED_bit){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);}
    else{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);}
}


void LED_Time(uint16_t LED_Time_number)
{
    if(LED_Light_Dark_Bit==1)       //执行亮
    {
        LED_Light(1);
        LED_Time_Light++;
        if(LED_Time_Light == LED_Time_number)
        {
            LED_Time_Light=0;
            LED_Light_Dark_Bit=0;   //切换到暗
        }
    }
    else if(LED_Light_Dark_Bit==0)       //执行暗
    {
        LED_Light(0);
        LED_Time_Dark++;
        if(LED_Time_Dark == LED_Time_number)
        {
            LED_Time_Dark = 0;
            LED_Light_Dark_Bit=1;   //切换到亮
        }
    }
}





