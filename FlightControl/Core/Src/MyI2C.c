#include "Delay.h"

void MyI2C_W_SCL(uint8_t bit)       //写入SCL位
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,(GPIO_PinState)bit);
    Delay_us(10);
}

void MyI2C_W_SDA(uint8_t bit)       //写入SDA位
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,(GPIO_PinState)bit);
    Delay_us(10);
}

uint8_t MyI2C_R_SDA(void)       //读取SDA位
{
    uint8_t bitval = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
    Delay_us(10);
    return bitval;
}


void MyI2C_Start(void)       //起始信号
{
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);

    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)       //停止信号
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}


void MyI2C_SendByte(uint8_t byte)
{
    for(uint8_t i=0;i<8;i++)
    {
        MyI2C_W_SDA(byte&(0x80 >> i));
        MyI2C_W_SCL(1);     //拉高从机上升沿读取
        MyI2C_W_SCL(0);     //拉低准备发送下一位
    }
}

uint8_t MyI2C_Receive(void)       //接收字节
{
    MyI2C_W_SDA(1); //释放给从机
    uint8_t R_byte=0x00;
    for(uint8_t i = 0;i<8;i++)
    {
        MyI2C_W_SCL(1);     //拉高从机上升沿读取
        if(MyI2C_R_SDA() == 1){R_byte |= (0x80 >> i);}
        MyI2C_W_SCL(0);     //拉低准备发送下一位
    }
    return R_byte;

}


void MyI2C_Send_ACK(uint8_t ACK)       //发送ACK信号
{
    MyI2C_W_SDA(ACK);
    MyI2C_W_SCL(1);     //拉高从机上升沿读取
    MyI2C_W_SCL(0);     //拉低准备发送下一位
}


uint8_t MyI2C_Receive_ACK(void)       //接收ACK信号
{
    uint8_t timeout = 0;
    MyI2C_W_SDA(1); //释放给从机
    MyI2C_W_SCL(1);     //拉高从机上升沿读取
    while(MyI2C_R_SDA() == 1)   //等待从机拉低SDA应答
    {
        timeout++;
        if(timeout > 250)       //超时
        {
            MyI2C_Stop();        //发送停止信号，结束传输
            return 1;            //返回1表示无应答
        }
    }
    MyI2C_W_SCL(0);     //拉低准备发送下一位
    return 0;           //返回0表示收到ACK
}

