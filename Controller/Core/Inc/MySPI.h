#ifndef __MYSPI_H__
#define __MYSPI_H__


#include "stm32f1xx_hal.h"

#define NRF_SCK_GPIO_Port   GPIOA
#define NRF_SCK_Pin         GPIO_PIN_8    /* SCK时钟（推挽输出） */
#define NRF_CSN_GPIO_Port   GPIOA
#define NRF_CSN_Pin         GPIO_PIN_11   /* CSN片选，低有效（推挽输出） */
#define NRF_CE_GPIO_Port    GPIOA
#define NRF_CE_Pin          GPIO_PIN_12   /* CE收发使能（推挽输出） */
#define NRF_MOSI_GPIO_Port  GPIOB
#define NRF_MOSI_Pin        GPIO_PIN_15   /* MOSI主出从入（推挽输出） */
#define NRF_MISO_GPIO_Port  GPIOB
#define NRF_MISO_Pin        GPIO_PIN_14   /* MISO主入从出（上拉输入） */
#define NRF_IRQ_GPIO_Port   GPIOB
#define NRF_IRQ_Pin         GPIO_PIN_13   /* 中断标志，低有效（上拉输入） */



void MySPI_W_MOSI(uint8_t vlaue);
void MySPI_W_SCK(uint8_t vlaue);
void MySPI_W_CSN(uint8_t vlaue);
void MySPI_W_CE(uint8_t vlaue);
uint8_t MySPI_R_MISO(void);
uint8_t MySPI_SwapData(uint8_t Byte);



#endif
