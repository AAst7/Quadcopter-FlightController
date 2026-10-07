#include "Variable.h"
#include "main.h"
#include "adc.h"

uint16_t Get_ADC(void)
{
    HAL_ADC_Start(&hadc1);      //开始转换
    while(HAL_ADC_GetState(&hadc1)!=HAL_ADC_STATE_READY);
    return HAL_ADC_GetValue(&hadc1);
}



float GetBattery(void)
{
    float temp_battery;
    temp_battery = (3.3*2*Get_ADC() / 4096);
    return temp_battery;
}


void GetCurrent_Power(void)         //获取当前电量,100次的平均值
{
    if(++Battery_num <= 100)
    {
        Battery_temp = GetBattery();
        Battery_total += Battery_temp;
    }
    else
    {
        Battery_average = (Battery_total / 100);
        Battery_total = 0;
        Battery_num = 0;
    }
}


