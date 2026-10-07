#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "stm32f1xx_hal.h"

// void NRF24L01_GPIO_Init(void);      //初始化GPIO引脚
uint8_t NRF24L01_ReadIRQ(void);     //读取中断标志位
void NRF24L01_WriteVal(uint8_t reg, uint8_t value);     //写入一个数据
uint8_t NRF24L01_ReadVal(uint8_t reg);                   //读取一个数据
void NRF24L01_WriteBuf(uint8_t reg, uint8_t *buf, uint8_t len);     //写入一个数据缓冲区
void Receive_NRF24L01_Buf(uint8_t reg, uint8_t *buf, uint8_t len);      //接收取一个数据缓冲区
void NRF24L01_Init(void);               //初始化NRF24L01
uint8_t Receive_NRF24L01_Data(uint8_t *buf);        //接收取一个数据
uint8_t Send(uint8_t *buf);                          //发送一个数据



#endif
