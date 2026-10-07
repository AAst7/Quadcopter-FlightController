#include "mpu6050.h"
#include "MyI2C.h"


#define GYRO_FILTER_GAIN 0.98

uint8_t mpu6050_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)         //写入数据,返回0成功,1失败
{
    addr = addr << 1;       //I2C规定前七位为地址，第八位是读写位
    MyI2C_Start();
    MyI2C_SendByte(addr);
    if(MyI2C_Receive_ACK()){MyI2C_Stop();return 1;}

    MyI2C_SendByte(reg);
    if(MyI2C_Receive_ACK()){MyI2C_Stop();return 1;}

    for(uint8_t i=0;i<len;i++)
    {
        MyI2C_SendByte(*buf++);
        MyI2C_Receive_ACK();
        if(MyI2C_Receive_ACK()){MyI2C_Stop();return 1;}
    }
    MyI2C_Stop();
    return 0;
}

uint8_t mpu6050_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)         //读取数据,返回0成功
{
    addr = addr << 1;
    MyI2C_Start();
    MyI2C_SendByte(addr);
    if(MyI2C_Receive_ACK()){MyI2C_Stop();return 1;}

    MyI2C_SendByte(reg);
    if(MyI2C_Receive_ACK()){MyI2C_Stop();return 1;}

    for(uint8_t i=0;i<len - 1;i++)
    {
        *buf++ = MyI2C_Receive();
        MyI2C_Send_ACK(0);
    }
    *buf = MyI2C_Receive();
    MyI2C_Send_ACK(1);
    MyI2C_Stop();
    return 0;
}


//=======语法糖，等价于上面两函数，更简洁========
void mpu6050_W_reg(uint8_t reg, uint8_t dat)
{
    mpu6050_Write(MPU_ADDR,reg,1,&dat);
}


uint8_t mpu6050_R_reg(uint8_t reg)
{
    uint8_t dat;
    mpu6050_Read(MPU_ADDR,reg,1,&dat);
    return dat;

}



uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)       //设置MPU6050陀螺仪传感器满量程范围fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps,返回0成功
{
    mpu6050_W_reg(GYRO_CONFIG,fsr<<3);
    return 0;
}

uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)       //设置MPU6050加速度传感器满量程范围fsr:0,±2g;1,±4g;2,±8g;3,±16g,返回0成功
{
    mpu6050_W_reg(ACCEL_CONFIG,fsr<<3);
    return 0;
}


uint8_t MPU_Set_LPF(uint16_t lpf)       //设置MPU6050的数字低通滤波器lpf:数字低通滤波频率(Hz),返回0成功
{
    uint8_t data=0;
    if(lpf >= 188)data=1;
    else if(lpf >= 98)data=2;
    else if(lpf >= 42)data=3;
    else if(lpf >= 21)data=4;
    else if(lpf >= 10)data=5;

    mpu6050_W_reg(CONFIG,data);
    return 0;
}


uint8_t MPU_Set_Rate(uint16_t rate)       //设置MPU6050的采样率rate:采样率(Hz),返回0成功
{
    //rate = Gyroscope Output Rate / (1 + data),开启LPF,Gyroscope Output Rate = 1KHz,未开启为8KHz
    uint8_t data ;
    if(rate > 1000)rate = 1000;
    if(rate < 4)rate = 4;       //开启LPF后最小采样率为4Hz，data最大为255
    data = (uint8_t)(1000  / rate - 1);
    mpu6050_W_reg(SMPLRT_DIV,data);
    return MPU_Set_LPF(rate / 2);
}



void MPU_Init(void)
{
    uint8_t res;
    mpu6050_W_reg(PWR_MGMT_1,0x80);     //重置所有
    Delay_ms(100);
    mpu6050_W_reg(PWR_MGMT_1,0x00);     //唤醒6050
    MPU_Set_Gyro_Fsr(3);
    MPU_Set_Accel_Fsr(0);
    MPU_Set_Rate(200);
    mpu6050_W_reg(MPU_INT_EN_REG,0x00);     //关闭中断
    mpu6050_W_reg(MPU_USER_CTRL_REG,0x00);     //关闭I2C主模式
    mpu6050_W_reg(MPU_FIFO_EN_REG,0x00);     //关闭FIFO
    mpu6050_W_reg(MPU_INTBP_CFG_REG,0x00);     //关闭中断/旁路设置
    //检测id
    res = mpu6050_R_reg(MPU_DEVICE_ID_REG);
    if(res == MPU_ADDR)
    {
        mpu6050_W_reg(PWR_MGMT_1,0x01);     //设置CLKSEL,PLL X轴为参考
        mpu6050_W_reg(PWR_MGMT_2,0x00);     //让加速度计和陀螺仪都工作

    }
}


short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    int16_t raw;
    float temp;
    mpu6050_Read(MPU_ADDR,TEMP_OUT_H,2,buf);
    raw = (buf[0] << 8) | buf[1];   //合并高8位和低8位
    temp = 36.53 + (double)raw / 340;       //转化为摄氏度
    return temp*100;
}


uint8_t MPU_Get_Gyroscope(float *gx,float *gy,float *gz)
{
    uint8_t buf[6],res;
    short gx_raw, gy_raw, gz_raw; // 临时存储原始整型值
    res = mpu6050_Read(MPU_ADDR,GYRO_XOUT_H,6,buf);
    if(res == 0)
    {
        gx_raw = ((uint16_t)buf[0] << 8) | buf[1];  
        gy_raw = ((uint16_t)buf[2] << 8) | buf[3];  
        gz_raw = ((uint16_t)buf[4] << 8) | buf[5];

        *gx = (float)gx_raw / 16.4f;
        *gy = (float)gy_raw / 16.4f;
        *gz = (float)gz_raw / 16.4f;
    }
    return res;
}


uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;
    res = mpu6050_Read(MPU_ADDR,ACCEL_XOUT_H,6,buf);
    if(res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];  
        *ay = ((uint16_t)buf[2] << 8) | buf[3];  
        *az = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

float gyro_filter(float new_val,float *last_val)    //陀螺仪一阶互补滤波，过滤角速度噪声
{
    float out = GYRO_FILTER_GAIN * new_val + (1 - GYRO_FILTER_GAIN) * (*last_val);
    *last_val = out;
    return out;
}








