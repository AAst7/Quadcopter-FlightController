#include "Variable.h"
#include "stm32f1xx_hal.h"

//矩阵键盘扫描的输入引脚定义宏
#define INPUT1 (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define INPUT2 (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)
#define INPUT3 (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)
#define INPUT4 (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)
#define OUTPUT1(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, x)
#define OUTPUT2(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, x)


// void Key_Init(void)
// {
// }


uint8_t Key_GetNum(void)
{
    uint8_t Temp = 0;
    if(Key_Num)
    {
        Temp = Key_Num;
        Key_Num = 0;
    }
    return Temp;
}



uint8_t Key_GetState(void)
{
    OUTPUT1(GPIO_PIN_RESET);OUTPUT2(GPIO_PIN_SET);
    if(INPUT1 == 1 && INPUT2 == 1 && INPUT3 == 1 && INPUT4 == 0) return 1;
    if(INPUT1 == 1 && INPUT2 == 1 && INPUT3 == 0 && INPUT4 == 1) return 2;
    if(INPUT1 == 1 && INPUT2 == 0 && INPUT3 == 1 && INPUT4 == 1) return 3;
    if(INPUT1 == 0 && INPUT2 == 1 && INPUT3 == 1 && INPUT4 == 1) return 4;
    
    OUTPUT1(GPIO_PIN_SET);OUTPUT2(GPIO_PIN_RESET);
    if(INPUT1 == 1 && INPUT2 == 1 && INPUT3 == 1 && INPUT4 == 0) return 5;
    if(INPUT1 == 1 && INPUT2 == 1 && INPUT3 == 0 && INPUT4 == 1) return 6;
    if(INPUT1 == 1 && INPUT2 == 0 && INPUT3 == 1 && INPUT4 == 1) return 7;
    if(INPUT1 == 0 && INPUT2 == 1 && INPUT3 == 1 && INPUT4 == 1) return 8;
    return 0;
}

void Key_Tick(void)
{
    static uint8_t curr_state = 0;
    static uint8_t prev_state = 0;
    static uint8_t count = 0;
    count++;
    if(count >= 10)             //后续调试再修改
    {
        count = 0;
        prev_state = curr_state;
        curr_state = Key_GetState();

        if(curr_state == 0 && prev_state != 0)       //松手检测
        {
            Key_Num = prev_state;
        }
    }
    
}

void KeyNUM(void)
{
    Key_Tick();
    Key_NumberCurr = Key_GetNum();

    if(Key_NumberCurr == 1)     //上
    {
        switch(OLED_Choose_Page)
        {
            case 0://菜单界面
                OLED_Choose_Row--;
                if(OLED_Choose_Row < 1) OLED_Choose_Row = 1;
                break;

            case 2://飞行数据（机械零点调整）
                OLED_Choose_Data[OLED_Choose_Page]--;
                if(OLED_Choose_Data[OLED_Choose_Page] <1) 
                OLED_Choose_Data[OLED_Choose_Page] =1;
                break;

            case 3://PID数据（飞控PID调整）
                OLED_Choose_Data[OLED_Choose_Page]--;
                if(OLED_Choose_Data[OLED_Choose_Page] <1) 
                OLED_Choose_Data[OLED_Choose_Page] =1;
                break;
        }
        Key_LED=1;
    }
    else if(Key_NumberCurr == 2)     //下
    {
        switch(OLED_Choose_Page)
        {
            case 0://菜单界面
                OLED_Choose_Row++;
                if(OLED_Choose_Row > 4) OLED_Choose_Row = 4;
                break;

            case 2://飞行数据（机械零点调整）
                OLED_Choose_Data[OLED_Choose_Page]++;
                if(OLED_Choose_Data[OLED_Choose_Page] >4) 
                OLED_Choose_Data[OLED_Choose_Page] =4;
                break;

            case 3://PID数据（飞控PID调整）
                OLED_Choose_Data[OLED_Choose_Page]++;
                if(OLED_Choose_Data[OLED_Choose_Page] >24) 
                OLED_Choose_Data[OLED_Choose_Page] =24;
                break;
        }
        Key_LED=2;
    }
    else if(Key_NumberCurr == 3)     //数值减
    {
        switch(OLED_Choose_Data[2])     //飞行数据（机械零点调整）
        {
            case 1: //精度选择 0.001，0.01，0.1
                if(Tuning_precision_index == 0) 
                Tuning_precision_index = 3;
                Tuning_precision_index--;
                break;

            case 2: //俯仰机械零点调整
                fly_pitch_zero -= Tuning_precision[Tuning_precision_index];
                break;

            case 3: //横滚机械零点调整
                fly_roll_zero -= Tuning_precision[Tuning_precision_index];
                break;

            case 4: //偏航机械零点调整
                fly_yaw_zero = 1;
                break;
    
        }
        switch(OLED_Choose_Data[3])     //PID数据（飞控PID调整）,六组PID
        {
            //精度
            case 1:
            case 5:
            case 9:
            case 13:
            case 17:
            case 21:
                if(Tuning_precision_index == 0) 
                Tuning_precision_index = 3;
                Tuning_precision_index--;
                break;

            //外环
            //俯仰
            case 2:
                pitch_angle_balance_Kp -= Tuning_precision[Tuning_precision_index];
                if(pitch_angle_balance_Kp < 0) pitch_angle_balance_Kp = 0;
                break;
            case 3:
                pitch_angle_balance_Ki -= Tuning_precision[Tuning_precision_index];
                if(pitch_angle_balance_Ki < 0) pitch_angle_balance_Ki = 0;
                break;
            case 4:
                pitch_angle_balance_Kd -= Tuning_precision[Tuning_precision_index];
                if(pitch_angle_balance_Kd < 0) pitch_angle_balance_Kd = 0;
                break;
                //横滚
            case 6:
                roll_angle_balance_Kp -= Tuning_precision[Tuning_precision_index];
                if(roll_angle_balance_Kp < 0) roll_angle_balance_Kp = 0;
                break;
            case 7:
                roll_angle_balance_Ki -= Tuning_precision[Tuning_precision_index];
                if(roll_angle_balance_Ki < 0) roll_angle_balance_Ki = 0;
                break;
            case 8:
                roll_angle_balance_Kd -= Tuning_precision[Tuning_precision_index];
                if(roll_angle_balance_Kd < 0) roll_angle_balance_Kd = 0;
                break;
                //偏航
            case 10:
                yaw_angle_balance_Kp -= Tuning_precision[Tuning_precision_index];
                if(yaw_angle_balance_Kp < 0) yaw_angle_balance_Kp = 0;
                break;
            case 11:
                yaw_angle_balance_Ki -= Tuning_precision[Tuning_precision_index];
                if(yaw_angle_balance_Ki < 0) yaw_angle_balance_Ki = 0;
                break;
            case 12:
                yaw_angle_balance_Kd -= Tuning_precision[Tuning_precision_index];
                if(yaw_angle_balance_Kd < 0) yaw_angle_balance_Kd = 0;
                break;

                 //内环
                 //俯仰
            case 14:
                pitch_gyro_balance_Kp -= Tuning_precision[Tuning_precision_index];
                if(pitch_gyro_balance_Kp < 0) pitch_gyro_balance_Kp = 0;
                break;  
            case 15:
                pitch_gyro_balance_Ki -= Tuning_precision[Tuning_precision_index];
                if(pitch_gyro_balance_Ki < 0) pitch_gyro_balance_Ki = 0;
                break;
            case 16:
                pitch_gyro_balance_Kd -= Tuning_precision[Tuning_precision_index];
                if(pitch_gyro_balance_Kd < 0) pitch_gyro_balance_Kd = 0;
                break;
                //横滚
            case 18:
                roll_gyro_balance_Kp -= Tuning_precision[Tuning_precision_index];
                if(roll_gyro_balance_Kp < 0) roll_gyro_balance_Kp = 0;
                break;
            case 19:
                roll_gyro_balance_Ki -= Tuning_precision[Tuning_precision_index];
                if(roll_gyro_balance_Ki < 0) roll_gyro_balance_Ki = 0;
                break;
            case 20:
                roll_gyro_balance_Kd -= Tuning_precision[Tuning_precision_index];
                if(roll_gyro_balance_Kd < 0) roll_gyro_balance_Kd = 0;
                break;
                //偏航
            case 22:
                yaw_gyro_balance_Kp -= Tuning_precision[Tuning_precision_index];
                if(yaw_gyro_balance_Kp < 0) yaw_gyro_balance_Kp = 0;
                break;
            case 23:
                yaw_gyro_balance_Ki -= Tuning_precision[Tuning_precision_index];
                if(yaw_gyro_balance_Ki < 0) yaw_gyro_balance_Ki = 0;
                break;
            case 24:
                yaw_gyro_balance_Kd -= Tuning_precision[Tuning_precision_index];
                if(yaw_gyro_balance_Kd < 0) yaw_gyro_balance_Kd = 0;
                break;
        }
    }

    else if(Key_NumberCurr == 4)        //数值加
    {
        switch(OLED_Choose_Data[2])     //飞行数据（机械零点调整）
        {
            case 1: //精度选择 0.001，0.01，0.1
                if(Tuning_precision_index > 2) 
                Tuning_precision_index = 0;
                Tuning_precision_index++;
                break;

            case 2: //俯仰机械零点调整
                fly_pitch_zero += Tuning_precision[Tuning_precision_index];
                break;

            case 3: //横滚机械零点调整
                fly_roll_zero += Tuning_precision[Tuning_precision_index];
                break;

            case 4: //偏航机械零点调整
                fly_yaw_zero = 1;
                break;
    
        }
        switch(OLED_Choose_Data[3])     //PID数据（飞控PID调整）,六组PID
        {
            //精度
            case 1:
            case 5:
            case 9:
            case 13:
            case 17:
            case 21:
                if(Tuning_precision_index > 2) 
                Tuning_precision_index = 0;
                Tuning_precision_index++;
                break;

            //外环
            //俯仰
            case 2:
                pitch_angle_balance_Kp += Tuning_precision[Tuning_precision_index];
                break;
            case 3:
                pitch_angle_balance_Ki += Tuning_precision[Tuning_precision_index];
                break;
            case 4:
                pitch_angle_balance_Kd += Tuning_precision[Tuning_precision_index];
                break;
                //横滚
            case 6:
                roll_angle_balance_Kp += Tuning_precision[Tuning_precision_index];
                break;
            case 7:
                roll_angle_balance_Ki += Tuning_precision[Tuning_precision_index];
                break;
            case 8:
                roll_angle_balance_Kd += Tuning_precision[Tuning_precision_index];
                break;
                //偏航
            case 10:
                yaw_angle_balance_Kp += Tuning_precision[Tuning_precision_index];
                break;
            case 11:
                yaw_angle_balance_Ki += Tuning_precision[Tuning_precision_index];
                break;
            case 12:
                yaw_angle_balance_Kd += Tuning_precision[Tuning_precision_index];   
                break;

                 //内环
                 //俯仰
            case 14:
                pitch_gyro_balance_Kp += Tuning_precision[Tuning_precision_index];
                break;  
            case 15:
                pitch_gyro_balance_Ki += Tuning_precision[Tuning_precision_index];
                break;
            case 16:
                pitch_gyro_balance_Kd += Tuning_precision[Tuning_precision_index];
                break;
                //横滚
            case 18:
                roll_gyro_balance_Kp += Tuning_precision[Tuning_precision_index];
                break;
            case 19:
                roll_gyro_balance_Ki += Tuning_precision[Tuning_precision_index];
                break;
            case 20:
                roll_gyro_balance_Kd += Tuning_precision[Tuning_precision_index];
                break;
                //偏航
            case 22:
                yaw_gyro_balance_Kp += Tuning_precision[Tuning_precision_index];
                break;
            case 23:
                yaw_gyro_balance_Ki += Tuning_precision[Tuning_precision_index];
                break;
            case 24:
                yaw_gyro_balance_Kd += Tuning_precision[Tuning_precision_index];
                break;
        }
 
    }


    else if(Key_NumberCurr == 7)     //锁定
    {
        Lock = 1;
        Key_LED = 7;
    }
    else if(Key_NumberCurr == 8)     //解锁
    {
        Lock = 0;
        Key_LED = 8;
    }
    else if(Key_NumberCurr == 5)     //返回主菜单
    {
        OLED_Choose_Page = 0;
        for(int i=0;i<4;i++)        //将OLED_Choose_Data数组清空
        {
            OLED_Choose_Data[i] = 0;
        }
    }
    else if(Key_NumberCurr == 6)     //确认
    {
        if(OLED_Choose_Data[2] == 4)
        {
            fly_yaw_zero++;
            if(fly_yaw_zero > 100) fly_yaw_zero = 0;    
        }
        OLED_Choose_Page=OLED_Choose_Row;
		OLED_Choose_Data[OLED_Choose_Page]=1;
					
		Key_LED=11;
    }


}
