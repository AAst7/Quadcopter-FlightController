#include "main.h"
#include "PWM.h"
#include "Variable.h"

uint8_t Motor_time = 4;


void Motor_Control(void)
{
    if(Lock == 0 && connect_protect == 0)
    {
        SET_Compare1(PWM_OUT1);
        SET_Compare2(PWM_OUT2);
        SET_Compare3(PWM_OUT3);
        SET_Compare4(PWM_OUT4);
    }
    else
    {
        SET_Compare1(0);
        SET_Compare2(0);
        SET_Compare3(0);
        SET_Compare4(0);
    }
}
