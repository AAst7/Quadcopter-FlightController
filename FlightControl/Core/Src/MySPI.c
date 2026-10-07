#include "main.h"
#include "NRF24L01_Reg.h"



void MySPI_W_MOSI(uint8_t Value)
{
	HAL_GPIO_WritePin(MOSI_Port,MOSI_Pin,(GPIO_PinState)Value);
}

void MySPI_W_SCLK(uint8_t Value)
{
	HAL_GPIO_WritePin(SCK_Port,SCK_Pin,(GPIO_PinState)Value);
}

void MySPI_W_CSN(uint8_t Value)
{
	HAL_GPIO_WritePin(CSN_Port,CSN_Pin,(GPIO_PinState)Value);
}

void MySPI_W_CE(uint8_t Value)
{
	HAL_GPIO_WritePin(CE_Port,CE_Pin,(GPIO_PinState)Value);
}

uint8_t MySPI_R_MISO(void)
{
	return (uint8_t)HAL_GPIO_ReadPin(MISO_Port,MISO_Pin);
}

//交换一个字节
uint8_t MySPI_SwapData(uint8_t Byte)//1111 1010
{
	uint8_t i,ReceiveByte=0x00;
	for(i=0;i<8;i++)
	{
		//SCLK高电平之前放置好数据
		MySPI_W_MOSI(Byte&(0x80>>i));
		//拉高SCLK，开始交换数据
		MySPI_W_SCLK(1);
		if(MySPI_R_MISO()==1)
		{
			ReceiveByte=ReceiveByte|(0x80>>i);
		}
		//拉低SCLK，放下一个数据
		MySPI_W_SCLK(0);
	}
	return ReceiveByte;
}





