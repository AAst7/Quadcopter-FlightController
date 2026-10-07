#include "NRF24L01_Reg.H"
#include "MySPI.h"
#include "Delay.h"
#include "main.h"

uint8_t T_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};
uint8_t R_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};



//NRF24L01中断读取位
uint8_t NRF24L01_R_IRQ(void)
{
	return HAL_GPIO_ReadPin(IRQ_Port,IRQ_Pin);
}

//写入NRF24L01寄存器一个数据的操作
void Write_NRF24L01_Val(uint8_t Reg,uint8_t Value)
{
	MySPI_W_CSN(0);												//选中从机
	MySPI_SwapData(Reg);									//指令
	MySPI_SwapData(Value);								//数据
	MySPI_W_CSN(1);												//停止选中从机
}

//读NRF24L01寄存器的操作
uint8_t Read_NRF24L01_Val(uint8_t Reg)
{
	uint8_t value;
	MySPI_W_CSN(0);												//选中从机
	MySPI_SwapData(Reg);									//指令
	value=MySPI_SwapData(NOP);						//数据
	MySPI_W_CSN(1);												//停止选中从机
	return value;
}

//写入NRF24L01寄存器一个数组的操作
void Write_NRF24L01_Buf(uint8_t Reg,uint8_t *Buf,uint8_t Len)
{
	uint8_t i;
	MySPI_W_CSN(0);												//选中从机
	MySPI_SwapData(Reg);									//指令
	for(i=0;i<Len;i++)
	{
		MySPI_SwapData(Buf[i]);
	}
	MySPI_W_CSN(1);												//停止选中从机
}

//接收NRF24L01寄存器一个数组的操作
void Receive_NRF24L01_Buf(uint8_t Reg,uint8_t *Buf,uint8_t Len)
{
	MySPI_W_CSN(0);												
	MySPI_SwapData(Reg);									//指令
	for(uint8_t i=0;i<Len;i++)
	{
		Buf[i]=MySPI_SwapData(NOP);
	}
	MySPI_W_CSN(1);												//停止选中从机
}

//NRF24L01初始化
void NRF24L01_Init(void)
{
	MySPI_W_CE(0);
	
	Write_NRF24L01_Buf(W_REGISTER+TX_ADDR,T_ADDR,5);//配置发送地址
	Write_NRF24L01_Buf(W_REGISTER+RX_ADDR_P0,R_ADDR,5);//配置接收地址
	Write_NRF24L01_Val(W_REGISTER+CONFIG,0x0F);//配置成接收模式
	Write_NRF24L01_Val(W_REGISTER+EN_AA,0x01);//配置自动应答模式
	Write_NRF24L01_Val(W_REGISTER+RF_CH,0x00);//配置通信频率2.4G
	Write_NRF24L01_Val(W_REGISTER+RX_PW_P0,9);//配置接收数据32字节
	Write_NRF24L01_Val(W_REGISTER+EN_RXADDR,0x01);//使能接通通道0
	Write_NRF24L01_Val(W_REGISTER+SETUP_RETR,0x1A);//配置500us自动重发,最多10次重发
	Write_NRF24L01_Val(FLUSH_RX,NOP);
	
	MySPI_W_CE(1);
}

//接收数据
void Receive_NRF24L01_Data(uint8_t*Buf)
{
	uint8_t Status; 																	//定义状态变量
	Status=Read_NRF24L01_Val(R_REGISTER+STATUS);			//获取当前接收状态变量
	if(Status & RX_OK)																//获取当前接收状态变量，若为真则说明接收消息成功
	{																				
		Receive_NRF24L01_Buf(R_RX_PAYLOAD,Buf,9);  		//将收到的消息存入Buff数组
		Write_NRF24L01_Val(FLUSH_RX,NOP);						    //清除接收寄存器
		Write_NRF24L01_Val(W_REGISTER+STATUS,Status);   //清除中断
		Delay_us(150);
	}	
	
}

//发送数据
uint8_t Send(uint8_t *Buf)
{
	uint8_t Status;																		//定义状态变量
	Write_NRF24L01_Buf(W_TX_PAYLOAD,Buf,9);   			 	//写入数据至发送缓冲区
	
	MySPI_W_CE(0);                               
	Write_NRF24L01_Val(W_REGISTER+CONFIG,0x0E);     	//配置发送模式
	MySPI_W_CE(1);
	
	while(NRF24L01_R_IRQ()==1);               		  	//等待中断标志位被拉低
	Status=Read_NRF24L01_Val(R_REGISTER+STATUS);	
	if(Status & MAX_TX)                         			//若发送次数达到最大
	{
		Write_NRF24L01_Val(FLUSH_TX,NOP);             	//清除发送数据缓冲区
		Write_NRF24L01_Val(W_REGISTER+STATUS,Status);	  //清除中断
		return MAX_TX;
	}
	if(Status & TX_OK)                        			  //若发送成功
	{
		Write_NRF24L01_Val(W_REGISTER+STATUS,Status);   //清除中断
		return TX_OK;
	}
	return 0;
}


