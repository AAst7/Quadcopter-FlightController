#include "stm32f1xx_hal.h"
#include "NRF24L01_Reg.h"
#include "MySPI.h"
#include "Delay.h"

uint8_t T_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};
uint8_t R_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};

// void NRF24L01_GPIO_Init(void)
// {
//     GPIO_InitTypeDef GPIO_InitStruct;
//     __HAL_RCC_GPIOA_CLK_ENABLE();
//     __HAL_RCC_GPIOB_CLK_ENABLE();
//     //输出的引脚配置
//     GPIO_InitStruct.Pin = CE_PIN | SCK_PIN | CSN_PIN;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = MOSI_PIN;
//     HAL_GPIO_Init(MOSI_PORT, &GPIO_InitStruct);

//     //输入的引脚配置
//     GPIO_InitStruct.Pin = IRQ_PIN | MISO_PIN;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
// }

uint8_t NRF24L01_ReadIRQ(void)      //中断读取位
{
    return HAL_GPIO_ReadPin(IRQ_PORT, IRQ_PIN);
}


void NRF24L01_WriteVal(uint8_t reg, uint8_t value)      //写入一个数据
{
    MySPI_W_CSN(0);
    MySPI_SwapData(reg);
    MySPI_SwapData(value);
    MySPI_W_CSN(1);
}

uint8_t NRF24L01_ReadVal(uint8_t reg)      //读取一个数据
{
    uint8_t vlaue;
    MySPI_W_CSN(0);
    MySPI_SwapData(reg);
    vlaue = MySPI_SwapData(NOP);
    MySPI_W_CSN(1);
    return vlaue;
}


void NRF24L01_WriteBuf(uint8_t reg, uint8_t *buf, uint8_t len)      //写入一个数组
{
    MySPI_W_CSN(0);
    MySPI_SwapData(reg);
    for(uint8_t i = 0; i < len; i++)
    {
        MySPI_SwapData(buf[i]);
    }
    MySPI_W_CSN(1);
}


void Receive_NRF24L01_Buf(uint8_t reg, uint8_t *buf, uint8_t len)      //读取一个数组
{
    MySPI_W_CSN(0);
    MySPI_SwapData(reg);
    for(uint8_t i = 0; i < len; i++)
    {
        buf[i] = MySPI_SwapData(NOP);
    }
    MySPI_W_CSN(1);
}


void NRF24L01_Init(void)            //初始化NRF24L01
{
    // NRF24L01_GPIO_Init();    gpio.c里已配置
    MySPI_W_CSN(0);

    NRF24L01_WriteBuf(W_REGISTER+TX_ADDR,T_ADDR,5);     //写入发送地址
    NRF24L01_WriteBuf(W_REGISTER+RX_ADDR_P0,R_ADDR,5);     //写入接收地址
    NRF24L01_WriteVal(W_REGISTER+CONFIG,0x0F);          //配置接收模式
    NRF24L01_WriteVal(W_REGISTER+EN_AA,0x01);          //开启通道0自动应答
    NRF24L01_WriteVal(W_REGISTER+RF_CH,0X00);          //2.4Ghz频率
    NRF24L01_WriteVal(W_REGISTER+RX_PW_P0,9);          //配置通道0的32字节接收数据宽度
    NRF24L01_WriteVal(W_REGISTER+EN_RXADDR,0x01);          //使能通道0的接收
    NRF24L01_WriteVal(W_REGISTER+SETUP_RETR,0x1A);//配置500us自动重发，最多10次重发
    NRF24L01_WriteVal(FLUSH_RX,NOP);                //重置接收寄存器标志位

    MySPI_W_CSN(1);
}


void Receive_NRF24L01_Data(uint8_t *buf)      //读取数据
{
    uint8_t status = NRF24L01_ReadVal(R_REGISTER+STATUS);
    if(status & RX_OK)
    {
        Receive_NRF24L01_Buf(R_REGISTER+R_RX_PAYLOAD,buf,9);
        NRF24L01_WriteVal(FLUSH_RX,NOP);                //清除接收寄存器标志位
        NRF24L01_WriteVal(W_REGISTER+STATUS,status);    //清除中断
        Delay_us(150);
    }

}


uint8_t Send(uint8_t *buf)          //发送数据
{
    uint8_t status;
    NRF24L01_WriteBuf(W_REGISTER+W_TX_PAYLOAD,buf,9);
    MySPI_W_CE(0);
    NRF24L01_WriteVal(W_REGISTER+CONFIG,0x0E);      //配置发送模式
    MySPI_W_CE(1);

    while(NRF24L01_ReadIRQ() == 1);                 //等待中断标志位拉低，发送完成
    status = NRF24L01_ReadVal(R_REGISTER+STATUS);
    if(status & MAX_TX)                             //如果到达最大重发次数，发送失败
    {
        NRF24L01_WriteVal(FLUSH_TX,NOP);              //清除发送数据缓存器
        NRF24L01_WriteVal(W_REGISTER+STATUS,status);    //清除中断
        return MAX_TX;
    }
    if(status & TX_OK)
    {
        NRF24L01_WriteVal(W_REGISTER+STATUS,status);    //清除中断
        return TX_OK;
    }
    return 0;

}




