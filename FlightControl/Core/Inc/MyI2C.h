#ifndef __MYI2C_H__
#define __MYI2C_H__

#include "main.h"

void MyI2C_W_SCL(uint8_t bit);
void MyI2C_W_SDA(uint8_t bit);
uint8_t MyI2C_R_SDA(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t byte);
uint8_t MyI2C_Receive(void);
void MyI2C_Send_ACK(uint8_t ACK);
uint8_t MyI2C_Receive_ACK(void);





#endif
