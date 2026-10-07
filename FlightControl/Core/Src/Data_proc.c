#include "NRF24L01.h"
#include "Variable.h"
#include "main.h"
#include "Delay.h"
#include "mpu6050.h"
#include "OLED.h"
#include "inv_mpu.h"
#include "Motor.h"
#include "LED.h"
#include "PowerDetection.h"






//角度转弧度，参与PID计算
#define DEG_TO_RAD 0.0174532925f


/*经过观察后得出*/
//飞机向前倾斜--pitch负数增大      角速度：  即是绕Y轴 gy负
//飞机向后倾斜--pitch正数增大                          gy正

//飞机向左倾斜--roll正数增大                 即是绕X轴 gx正
//飞机向右倾斜--roll负数增大                           gx负

//从上往下看：
//飞机逆时针--yaw正数增大                    即是绕Z轴 gz正
//飞机顺时针--yaw负数增大                              gz负



#define PID_Oil_Min 360         
#define PWM_MAX 2880            //占空比80% = 2880 / 36000，剩下20%用PID纠偏
#define PWM_MIN 360             //空心杯电机启动最小阈值10%

#define ANGLE_ERR_THRESHOLD 3.0f    // 外环积分分离阈值：误差<2°才积分（角度）
#define TARGET_GYRO_MAX 300.0f       // 最大目标角速度（°/s），空心杯推荐±50
#define TARGET_GYRO_MIN -300.0f
#define I_ANGLE_IMAX 500.0f           // 积分上限（适配角速度指令）

#define GYRO_ERR_THRESHOLD  3.0f   // 内环积分分离阈值（角速度误差<5°/s才累加）
#define TARGET_PWM_MAX 1000           // 最大目标PWM
#define TARGET_PWM_MIN -1000
#define I_GYRO_IMAX 1200.0f           // 积分上限（适配角速度指令）


int16_t PWM_limit(int16_t pwm)
{
    if(pwm > PWM_MAX)
    {
        return PWM_MAX;
    }
    else if(pwm < PWM_MIN)
    {
        return PWM_MIN;
    }
    else
    {
        return pwm;
    }
}

//====================俯仰====================
//pitch 外环
float Pitch_Angle_Balance(float angle,float gryo)
{
    float PID_Angle_Out;
    PID_Pitch_Angle_err = (fly_pitch_zero + getpitch_SET) - angle;
    if(Oil_Set > PID_Oil_Min && ABS(pitch_angle_balance_Ki) > (1e-6f))
    {
        if(ABS(PID_Pitch_Angle_err) < ANGLE_ERR_THRESHOLD)
        {
            PID_Pitch_Angle_Integral += PID_Pitch_Angle_err;
            //限幅
            PID_Pitch_Angle_Integral = MIN(PID_Pitch_Angle_Integral,(float)I_ANGLE_IMAX);
            PID_Pitch_Angle_Integral = MAX(PID_Pitch_Angle_Integral,(float)-I_ANGLE_IMAX);
        }
        //计算外环输出，需要的角速度
        PID_Angle_Out = pitch_angle_balance_Kp * PID_Pitch_Angle_err + pitch_angle_balance_Ki * PID_Pitch_Angle_Integral + pitch_angle_balance_Kd * gryo;
        //积分抗饱和，反向修复积分
        if(PID_Angle_Out > TARGET_GYRO_MAX)
        {
            PID_Pitch_Angle_Integral -= (PID_Angle_Out - TARGET_GYRO_MAX) / pitch_angle_balance_Ki;  //由外环输出反向修复积分，上面公式推出
        }
        else if(PID_Angle_Out < TARGET_GYRO_MIN)
        {
            PID_Pitch_Angle_Integral -= (TARGET_GYRO_MIN - PID_Angle_Out) / pitch_angle_balance_Ki; //同上，只是括号里为负数
        }
        //二次积分限幅
        PID_Pitch_Angle_Integral = MIN(PID_Pitch_Angle_Integral,(float)I_ANGLE_IMAX);
        PID_Pitch_Angle_Integral = MAX(PID_Pitch_Angle_Integral,(float)-I_ANGLE_IMAX);

    }
    else  PID_Pitch_Angle_Integral = 0.0f;  //油门不够，积分清零
    PID_Angle_Out = pitch_angle_balance_Kp*PID_Pitch_Angle_err+pitch_angle_balance_Ki*PID_Pitch_Angle_Integral+pitch_angle_balance_Kd*gryo;
    //限幅
    PID_Angle_Out = MIN(MAX(PID_Angle_Out, (float)TARGET_GYRO_MIN), (float)TARGET_GYRO_MAX);
    return PID_Angle_Out;
}

//Pitch 内环
int16_t Pitch_Gyro_Balance(float PID_Angle_Out,float gryo)
{
    float PID_PWM_Balance;
    //角速度误差，目标角速度-当前角速度
    PID_Pitch_Gyro_err =  PID_Angle_Out - gryo;
    if(Oil_Set > PID_Oil_Min && ABS(pitch_gyro_balance_Ki) > (1e-6f))         //油门够，且Ki不为0,进入积分逻辑
    {
        if(ABS(PID_Pitch_Gyro_err) < GYRO_ERR_THRESHOLD)
        {
            PID_Pitch_Gyro_Integral += PID_Pitch_Gyro_err;
            //限幅
            PID_Pitch_Gyro_Integral = MIN(PID_Pitch_Gyro_Integral,(float)I_GYRO_IMAX);
            PID_Pitch_Gyro_Integral = MAX(PID_Pitch_Gyro_Integral,(float)-I_GYRO_IMAX);
        }
        //计算内环输出，需要的PWM
        PID_PWM_Balance  = pitch_gyro_balance_Kp * PID_Pitch_Gyro_err + pitch_gyro_balance_Ki * PID_Pitch_Gyro_Integral + pitch_gyro_balance_Kd * (PID_Pitch_Gyro_err - PID_Pitch_Gyro_err_last);
        //积分抗饱和，反向修复积分
        if(PID_PWM_Balance > TARGET_PWM_MAX)
        {
            PID_Pitch_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MAX) / pitch_gyro_balance_Ki;  //由内环输出反向修复积分，上面公式推出
        }
        else if(PID_PWM_Balance < TARGET_PWM_MIN)
        {
            PID_Pitch_Gyro_Integral -= (TARGET_PWM_MIN - PID_PWM_Balance) / pitch_gyro_balance_Ki; //同上，只是括号里为负数
        }
        //二次积分限幅
        PID_Pitch_Gyro_Integral = MIN(PID_Pitch_Gyro_Integral,(float)I_GYRO_IMAX);
        PID_Pitch_Gyro_Integral = MAX(PID_Pitch_Gyro_Integral,(float)-I_GYRO_IMAX);

    }
    else  PID_Pitch_Gyro_Integral = 0.0f;  //油门不够，积分清零
    //基于修正的积分重新计算PWM输出
    PID_PWM_Balance = pitch_gyro_balance_Kp*PID_Pitch_Gyro_err+pitch_gyro_balance_Ki*PID_Pitch_Gyro_Integral+pitch_gyro_balance_Kd*gryo;
    //限幅
    PID_PWM_Balance = MIN(MAX(PID_PWM_Balance, (float)TARGET_PWM_MIN), (float)TARGET_PWM_MAX);
    //记录当前角速度，用于下一帧D项计算
	PID_Pitch_Gyro_err_last = PID_Pitch_Gyro_err;
    return (int16_t)PID_PWM_Balance;     //返回PWM输出，转整型给电机驱动
}

//====================翻滚====================
//Roll 外环
float Roll_Angle_Balance(float angle,float gryo)
{
    float PID_Angle_Out;
    PID_Roll_Angle_err = (fly_roll_zero + getroll_SET) - angle;
    if(Oil_Set > PID_Oil_Min && ABS(roll_angle_balance_Ki) > (1e-6f))
    {
        if(ABS(PID_Roll_Angle_err) < ANGLE_ERR_THRESHOLD)
        {
            PID_Roll_Angle_Integral += PID_Roll_Angle_err;
            //限幅
            PID_Roll_Angle_Integral = MIN(PID_Roll_Angle_Integral,(float)I_ANGLE_IMAX);
            PID_Roll_Angle_Integral = MAX(PID_Roll_Angle_Integral,(float)-I_ANGLE_IMAX);
        }
        //预计算外环输出，需要的角速度
        PID_Angle_Out = roll_angle_balance_Kp * PID_Roll_Angle_err + roll_angle_balance_Ki * PID_Roll_Angle_Integral + roll_angle_balance_Kd * gryo;
        //积分抗饱和，反向修复积分
        if(PID_Angle_Out > TARGET_GYRO_MAX)
        {
            PID_Roll_Angle_Integral -= (PID_Angle_Out - TARGET_GYRO_MAX) / roll_angle_balance_Ki;  //由外环输出反向修复积分，上面公式推出
        }
        else if(PID_Angle_Out < TARGET_GYRO_MIN)
        {
            PID_Roll_Angle_Integral -= (TARGET_GYRO_MIN - PID_Angle_Out) / roll_angle_balance_Ki; //同上，只是括号里为负数
        }
        //二次积分限幅
        PID_Roll_Angle_Integral = MIN(PID_Roll_Angle_Integral,(float)I_ANGLE_IMAX);
        PID_Roll_Angle_Integral = MAX(PID_Roll_Angle_Integral,(float)-I_ANGLE_IMAX);
    }
    else  PID_Roll_Angle_Integral = 0.0f;  //油门不够，积分清零
    //计算最终输出的角速度
    PID_Angle_Out = roll_angle_balance_Kp*PID_Roll_Angle_err+roll_angle_balance_Ki*PID_Roll_Angle_Integral+roll_angle_balance_Kd*gryo;
    //限幅
    PID_Angle_Out = MIN(MAX(PID_Angle_Out, (float)TARGET_GYRO_MIN), (float)TARGET_GYRO_MAX);
    return PID_Angle_Out;

}


//Roll 内环

int16_t Roll_Gyro_Balance(float PID_Angle_Out,float gryo)
{
    float PID_PWM_Balance;
    //角速度误差，目标角速度-当前角速度
    PID_Roll_Gyro_err =  PID_Angle_Out - gryo;
    if(Oil_Set > PID_Oil_Min && ABS(roll_gyro_balance_Ki) > (1e-6f))         //油门够，且Ki不为0,进入积分逻辑
    {
        if(ABS(PID_Roll_Gyro_err) < GYRO_ERR_THRESHOLD)      
        {
            PID_Roll_Gyro_Integral += PID_Roll_Gyro_err;        //积分累加
            //积分限幅
            PID_Roll_Gyro_Integral = MIN(PID_Roll_Gyro_Integral,(float)I_GYRO_IMAX);
            PID_Roll_Gyro_Integral = MAX(PID_Roll_Gyro_Integral,(float)-I_GYRO_IMAX);
        }
        //预计算内环输出，需要的PWM
        PID_PWM_Balance  = roll_gyro_balance_Kp * PID_Roll_Gyro_err + roll_gyro_balance_Ki * PID_Roll_Gyro_Integral + roll_gyro_balance_Kd * (PID_Roll_Gyro_err - PID_Roll_Gyro_err_last);
        //积分抗饱和，反向修复积分
        if(PID_PWM_Balance > TARGET_PWM_MAX)
        {
            PID_Roll_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MAX) / roll_gyro_balance_Ki;
        }
        else if(PID_PWM_Balance < TARGET_PWM_MIN)
        {
            PID_Roll_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MIN) / roll_gyro_balance_Ki;
        }
        //二次积分限幅
        PID_Roll_Gyro_Integral = MIN(PID_Roll_Gyro_Integral,(float)I_GYRO_IMAX);
        PID_Roll_Gyro_Integral = MAX(PID_Roll_Gyro_Integral,(float)-I_GYRO_IMAX);
    }
    else PID_Roll_Gyro_Integral = 0.0f;     //油门不够，清零积分
    //基于修正的积分重新计算内环输出，需要的PWM
    PID_PWM_Balance  = roll_gyro_balance_Kp * PID_Roll_Gyro_err + roll_gyro_balance_Ki * PID_Roll_Gyro_Integral + roll_gyro_balance_Kd * (PID_Roll_Gyro_err - PID_Roll_Gyro_err_last);
    //限幅
    PID_PWM_Balance = MIN(MAX(PID_PWM_Balance, (float)TARGET_PWM_MIN), (float)TARGET_PWM_MAX);
    //记录上次误差
    PID_Roll_Gyro_err_last = PID_Roll_Gyro_err;
    //返回整型
    return (int16_t)PID_PWM_Balance;
}            


//Yaw 外环
//==========================航向==================================
float Yaw_Angle_Balance(float angle,float gryo)
{
    float PID_Angle_Out;
    //计算航向误差
    PID_Yaw_Angle_err = (fly_yaw_zero + getyaw_SET) - angle;
    if(Oil_Set > PID_Oil_Min && ABS(yaw_angle_balance_Ki) > (1e-6f))      //油门够，且Ki不为0,进入积分逻辑
    {
        if(ABS(PID_Yaw_Angle_err) < ANGLE_ERR_THRESHOLD)
        {
            PID_Yaw_Angle_Integral += PID_Yaw_Angle_err;    //积分累加
            //限幅
            PID_Yaw_Angle_Integral = MIN(PID_Yaw_Angle_Integral,(float)I_ANGLE_IMAX);
            PID_Yaw_Angle_Integral = MAX(PID_Yaw_Angle_Integral,(float)-I_ANGLE_IMAX);
        }
        //预计算外环输出，需要的角速度
        PID_Angle_Out = yaw_angle_balance_Kp * PID_Yaw_Angle_err + yaw_angle_balance_Ki * PID_Yaw_Angle_Integral + yaw_angle_balance_Kd * gryo;
        //积分抗饱和，反向修复积分
        if(PID_Angle_Out > TARGET_GYRO_MAX)
        {
            PID_Yaw_Angle_Integral -= (PID_Angle_Out - TARGET_GYRO_MAX);
        }
        else if(PID_Angle_Out < TARGET_GYRO_MIN)
        {
            PID_Yaw_Angle_Integral -= (TARGET_GYRO_MIN - PID_Angle_Out) / yaw_angle_balance_Ki; //同上，只是括号里为负数
        }
        //二次限幅
        PID_Angle_Out = MIN(MAX(PID_Angle_Out, (float)TARGET_GYRO_MIN), (float)TARGET_GYRO_MAX);
    }
    else PID_Yaw_Angle_Integral = 0.0f;     //油门不够，清零积分
    //计算最终输出的角速度
    PID_Angle_Out = yaw_angle_balance_Kp*PID_Yaw_Angle_err+yaw_angle_balance_Ki*PID_Yaw_Angle_Integral+yaw_angle_balance_Kd*gryo;
    //限幅
    PID_Angle_Out = MIN(MAX(PID_Angle_Out, (float)TARGET_GYRO_MIN), (float)TARGET_GYRO_MAX);
    return PID_Angle_Out;
}

//内环 Yaw
int16_t Yaw_Gyro_Balance(float PID_Angle_Out,float gryo)
{
    float PID_PWM_Balance;
    //角速度误差，目标角速度-当前角速度
    PID_Yaw_Gyro_err =  PID_Angle_Out - gryo;
    if(Oil_Set > PID_Oil_Min && ABS(yaw_gyro_balance_Ki) > (1e-6f))      //油门够，且Ki不为0,进入积分逻辑
    {
        if(ABS(PID_Yaw_Gyro_err) < GYRO_ERR_THRESHOLD)
        {
            PID_Yaw_Gyro_Integral += PID_Yaw_Gyro_err;        //积分累加
            //积分限幅
            PID_Yaw_Gyro_Integral = MIN(PID_Yaw_Gyro_Integral,(float)I_GYRO_IMAX);
            PID_Yaw_Gyro_Integral = MAX(PID_Yaw_Gyro_Integral,(float)-I_GYRO_IMAX);
        }
        //计算内环输出，需要的PWM
        PID_PWM_Balance = yaw_angle_balance_Kp * PID_Yaw_Gyro_err + yaw_angle_balance_Ki * PID_Yaw_Gyro_Integral + yaw_angle_balance_Kd * (PID_Yaw_Gyro_err - PID_Yaw_Gyro_err_last);
        //积分抗饱和，反向修复积分
        if(PID_PWM_Balance > TARGET_PWM_MAX)
        {
            PID_Yaw_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MAX) / yaw_gyro_balance_Ki;
        }
        else if(PID_PWM_Balance < TARGET_PWM_MIN)
        {
            PID_Yaw_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MIN) / yaw_gyro_balance_Ki;
        }
        //二次积分限幅
        PID_Yaw_Gyro_Integral = MIN(PID_Yaw_Gyro_Integral,(float)I_GYRO_IMAX);
        PID_Yaw_Gyro_Integral = MAX(PID_Yaw_Gyro_Integral,(float)-I_GYRO_IMAX);
    }
    else PID_Yaw_Gyro_Integral = 0.0f;     //油门不够，清零积分
    //基于修正的积分重新计算内环输出，需要的PWM
    PID_PWM_Balance = yaw_angle_balance_Kp * PID_Yaw_Gyro_err + yaw_angle_balance_Ki * PID_Yaw_Gyro_Integral + yaw_angle_balance_Kd * (PID_Yaw_Gyro_err - PID_Yaw_Gyro_err_last);
    //限幅
    PID_PWM_Balance = MIN(MAX(PID_PWM_Balance, (float)TARGET_PWM_MIN), (float)TARGET_PWM_MAX);
    //记录上次误差
    PID_Yaw_Gyro_err_last = PID_Yaw_Gyro_err;
    //返回整型
    return (int16_t)PID_PWM_Balance;
}

void fly_task(void)
{
    //外环
    Pitch_Angle_out=Pitch_Angle_Balance(Pitch,gy);
    Roll_Angle_out=Roll_Angle_Balance(Roll,gx);
    Yaw_Angle_out=Yaw_Angle_Balance(Yaw,gz);
    //内环
    Pitch_Balance_out=Pitch_Gyro_Balance(Pitch_Angle_out,gy);
    Roll_Balance_out=Roll_Gyro_Balance(Roll_Angle_out,gx);
    Yaw_Balance_out=Yaw_Gyro_Balance(Yaw_Angle_out,gz);

    if(Oil_Set > PID_Oil_Min)       //油门够就进入PID调试
    { 
        PWM_OUT1 = Oil_Set + Pitch_Balance_out - Roll_Balance_out + Yaw_Balance_out;    //右前电机,抬头，右倾减力
        PWM_OUT2 = Oil_Set + Pitch_Balance_out + Roll_Balance_out - Yaw_Balance_out;    //左前电机,抬头，左倾减力
        PWM_OUT3 = Oil_Set - Pitch_Balance_out - Roll_Balance_out - Yaw_Balance_out;    //右后电机,抬尾，右倾减力
        PWM_OUT4 = Oil_Set - Pitch_Balance_out + Roll_Balance_out + Yaw_Balance_out;    //左后电机,抬尾，左倾减力

        //PWM限幅
        PWM_OUT1=PWM_limit(PWM_OUT1);
        PWM_OUT2=PWM_limit(PWM_OUT2);
        PWM_OUT3=PWM_limit(PWM_OUT3);
        PWM_OUT4=PWM_limit(PWM_OUT4);
    }
    else
    {
        PWM_OUT1 = Oil_Set;
        PWM_OUT2 = Oil_Set;
        PWM_OUT3 = Oil_Set;
        PWM_OUT4 = Oil_Set;
    }
 

    Motor_Control();
}

void LED_Detect(void)
{
    if(connect_protect == 1 && Battery_Protect == 0)  //通信连接失败，LED闪烁500ms一次
    LED_Time(500);
    else if(connect_protect == 0 && Battery_Protect == 0)  //通信连接成功，LED亮
    LED_Light(1);
    else if(Battery_Protect == 1)  //电池低，LED闪烁50ms一次
    LED_Time(50);
}



//数据处理
void Data_Proc(void)
{
    if(getpitch > 40 && getpitch < 60) getpitch_SET = 0;  //死区
    if(getpitch >0 && getpitch < 40) getpitch_SET = 0.25 * getpitch - 10;  //0到-10
    if(getpitch > 60 && getpitch < 100) getpitch_SET = 0.25 * getpitch - 15;  //0到10

    if(getroll > 40 && getroll < 60) getroll_SET = 0;  //死区
    if(getroll >0 && getroll < 40) getroll_SET = 0.25 * getroll - 10;  //0到-10
    if(getroll > 60 && getroll < 100) getroll_SET = 0.25 * getroll - 15;  //0到10
  
    //航向
	getyaw_SET=getyaw-100;

	//油门限制
	Oil_Set=Oil_Get*36;
    
    if(index_type != 0)
    {
        switch(index_type)
        {
            case 1: //重置俯仰零点
                fly_pitch_zero = (PID_Data_Send[0] + PID_Data_Send[1] * 0.01) - 100.0;
                break;
            case 2:     //重置横滚零点
                fly_roll_zero = (PID_Data_Send[0] + PID_Data_Send[1] * 0.01) - 100.0;
                break;
            case 3:     //重置航向零点
                fly_yaw_zero_bit = (PID_Data_Send[0] + PID_Data_Send[1] * 0.01) - 100.0;
                if(fly_yaw_zero_bit != fly_yaw_zero_bit_old)
                {
                    MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw);
                    fly_yaw_zero = Yaw;
                    fly_yaw_zero_bit_old = fly_yaw_zero_bit;
                }
                break;
            
            case 4:     //外环俯仰P
                PID_Data[0][0] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 5:     //外环俯仰I
                PID_Data[0][1] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 6:     //外环俯仰D
                PID_Data[0][2] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 7:     //外环横滚P
                PID_Data[1][0] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 8:     //外环横滚I
                PID_Data[1][1] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 9:     //外环横滚D
                PID_Data[1][2] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 10:     //外环航向P
                PID_Data[2][0] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 11:     //外环航向I
                PID_Data[2][1] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 12:     //外环航向D
                PID_Data[2][2] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
    //=================内环==================
            case 13:     //内环俯仰P
                PID_Data[3][0] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 14:     //内环俯仰I
                PID_Data[3][1] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 15:     //内环俯仰D
                PID_Data[3][2] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 16:     //内环横滚P
                PID_Data[4][0] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 17:     //内环横滚I
                PID_Data[4][1] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 18:     //内环横滚D
                PID_Data[4][2] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 19:     //内环航向P
                PID_Data[5][0] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 20:     //内环航向I
                PID_Data[5][1] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
            case 21:     //内环航向D
                PID_Data[5][2] = PID_Data_Send[0] * 0.1 + (PID_Data_Send[1] * 0.001);
                break;
        }
        //设置当前PID参数
        pitch_angle_balance_Kp = PID_Data[0][0];
        pitch_angle_balance_Ki = PID_Data[0][1];
        pitch_angle_balance_Kd = PID_Data[0][2];
        roll_angle_balance_Kp = PID_Data[1][0];
        roll_angle_balance_Ki = PID_Data[1][1];
        roll_angle_balance_Kd = PID_Data[1][2];
        yaw_angle_balance_Kp = PID_Data[2][0];
        yaw_angle_balance_Ki = PID_Data[2][1];
        yaw_angle_balance_Kd = PID_Data[2][2];
        pitch_gyro_balance_Kp = PID_Data[3][0];
        pitch_gyro_balance_Ki = PID_Data[3][1];
        pitch_gyro_balance_Kd = PID_Data[3][2];
        roll_gyro_balance_Kp = PID_Data[4][0];
        roll_gyro_balance_Ki = PID_Data[4][1];
        roll_gyro_balance_Kd = PID_Data[4][2];
        yaw_gyro_balance_Kp = PID_Data[5][0];
        yaw_gyro_balance_Ki = PID_Data[5][1];
        yaw_gyro_balance_Kd = PID_Data[5][2];
    }
    //电池检测
    if(Battery_average <= 3.5) Battery_Protect = 1;
    else Battery_Protect = 0;

}


void Attitude_Data_Proc(void)   //获取姿态角度与角速度
{
    MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw);
    MPU_Get_Gyroscope(&gx,&gy,&gz);
}

void Data_Display(void)
{
    /*油门显示*/
	OLED_Printf(0,0,OLED_6X8,"o:%04d",Oil_Set);
	/*各电机的PWM显示*/
	OLED_Printf(0,8,OLED_6X8,"1:%04d",PWM_OUT1);
	OLED_Printf(0,16,OLED_6X8,"2:%04d",PWM_OUT2);
	OLED_Printf(0,24,OLED_6X8,"3:%04d",PWM_OUT3);
	OLED_Printf(0,32,OLED_6X8,"4:%04d",PWM_OUT4);
	/*当前电量*/
	OLED_Printf(0,55,OLED_6X8,"B:%03.2f",Battery_average);
  /*初始角度*/
	OLED_Printf(50,0,OLED_6X8,"pitch:%04.2f",fly_pitch_zero);
	OLED_Printf(50,8,OLED_6X8,"roll:%04.2f",fly_roll_zero);
	OLED_Printf(50,16,OLED_6X8,"yaw:%04.2f",fly_yaw_zero);
  /*PD调试*/
	OLED_Printf(46,28,OLED_6X8,"Y:%04.3f",gy);
	OLED_Printf(88,28,OLED_6X8,"P:%04.2f",Pitch);
	OLED_Printf(46,38,OLED_6X8,"X:%04.2f",gx);
	OLED_Printf(88,38,OLED_6X8,"R:%04.2f",Roll);
	OLED_Printf(46,48,OLED_6X8,"Z:%04.2f",gz);
	OLED_Printf(88,48,OLED_6X8,"Y:%04.2f",Yaw);
	OLED_Update();
}



void Date_Receive(void)
{
    if(NRF24L01_R_IRQ() == 0)
    {
        Receive_NRF24L01_Data(Receive_Data);    //接收遥控发来的数据
        Lock = Receive_Data[0];
        Oil_Get= Receive_Data[1];       //油门
        getpitch=Receive_Data[2];
        getroll= Receive_Data[3];
        getyaw = Receive_Data[4];
        connect_value   =Receive_Data[5];       //持续连接成功的标志
        index_type      =Receive_Data[6];       //数据类型
        PID_Data_Send[0]=Receive_Data[7];       //浮点数的整数
        PID_Data_Send[1]=Receive_Data[8];       //浮点数的小数
    }
}



void Data_Send(void)    //发送数据
{
    Send(Send_Data);
}


void Connect_Portect_Test(void)    //连接保护测试
{
    if(connect_bit==0)
    {
        connect_oldvalue=connect_value; //把当前的值储存起来用于下次的比较。			
        connect_bit=1;
    }
    else
    {
        if(connect_oldvalue == connect_value)	connect_protect=1; //相同掉线，不同连接成功
        else	connect_protect=0;			
        connect_bit=0;
    }
}







