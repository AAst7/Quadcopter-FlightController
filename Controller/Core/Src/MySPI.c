#include "MySPI.h"

//==================设置引脚状态====================
void MySPI_W_MOSI(uint8_t vlaue)        //MOSI
{
    HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port, NRF_MOSI_Pin, (GPIO_PinState)vlaue);
}

void MySPI_W_SCK(uint8_t vlaue)        //SCK    
{
    HAL_GPIO_WritePin(NRF_SCK_GPIO_Port, NRF_SCK_Pin, (GPIO_PinState)vlaue);
}
void MySPI_W_CSN(uint8_t vlaue)        //CSN
{
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, (GPIO_PinState)vlaue);
}

void MySPI_W_CE(uint8_t vlaue)        //CE
{
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, (GPIO_PinState)vlaue);
}

uint8_t MySPI_R_MISO(void)        //MISO
{
    return (uint8_t)HAL_GPIO_ReadPin(NRF_MISO_GPIO_Port, NRF_MISO_Pin);
}

//==================数据交换====================
uint8_t MySPI_SwapData(uint8_t Byte)            //SPI模式0，SCK空闲低，上升沿采样
{
    uint8_t ReceiveByte = 0x00;
    uint8_t i = 0;
    for(i = 0; i < 8; i++)
    {
        MySPI_W_MOSI((Byte & (0x80 >> i)) ? 1 : 0);     //SCK拉高前放好数据
        MySPI_W_SCK(1);                              //SCK拉高
        if(MySPI_R_MISO() == 1)
        {
            ReceiveByte |= (0x80 >> i);             //接收数据，如果是高电平就设置为1，低电平不管就为0
                                                    //等效于ReceiveByte |= (MySPI_R_MISO() ? (0x80 >> i) : 0);
        }

        MySPI_W_SCK(0);                              //SCK拉低
    }
    return ReceiveByte;
}

