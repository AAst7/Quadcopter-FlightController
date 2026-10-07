#ifndef __MPU6050_I2C_H__
#define __MPU6050_I2C_H__

#include "main.h"
#include "Delay.h"

#define MPU6050_IIC_GPIO                   GPIOB
#define MPU6050_IIC_SCL_Pin                GPIO_PIN_10	         //PB10
#define MPU6050_IIC_SDA_Pin                GPIO_PIN_11	         //PB11

#define	MPU6050_IIC_SCL                    PBout(10)
#define	MPU6050_IIC_SDA                    PBout(11)  
#define	MPU6050_IIC_SDA_IN                 PBin(11)              //读数据端口
#define MPU6050_IIC_delay_4us()            delay_us(4)



#endif