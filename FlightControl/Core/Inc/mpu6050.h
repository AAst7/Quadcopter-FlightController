#ifndef __MPU6050_H__
#define __MPU6050_H__


#include "main.h"
#include "Delay.h"

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

//寄存器地址
#define SMPLRT_DIV 0x19            //采样率寄存器 
#define CONFIG 0x1A              //配置寄存器
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器


#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B 
#define	PWR_MGMT_2		0x6C
#define	MPU_DEVICE_ID_REG	  	0x75   //0x75	
#define	MPU_ADDR	0x68 //IIC地址寄存器0x68,AD0悬空是0x69    CHUANSHONGMEN

#define MPU6050_USE_Filter 1 // 是否使用滤波器



uint8_t mpu6050_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t mpu6050_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
void mpu6050_W_reg(uint8_t reg, uint8_t dat);
uint8_t mpu6050_R_reg(uint8_t reg);
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);
void MPU_Init(void);
short MPU_Get_Temperature(void);
uint8_t MPU_Get_Gyroscope(float *gx,float *gy,float *gz);
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);
float gyro_filter(float new_val,float *last_val);

#define mpu6050_write mpu6050_Write
#define mpu6050_read  mpu6050_Read

#endif
