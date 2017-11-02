

/* Includes ------------------------------------------------------------------*/
#include "I2C_init.h"

/*******************************************************************************
* I2C_delay
*******************************************************************************/
void I2C_delay(void)
{       
   u8 i=150;
   while(i) { i--; }
}

/*******************************************************************************
* GPIO_Configuration_I2C
*******************************************************************************/
void GPIO_Configuration_I2C(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);
	
	/* Configure I2C1 pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure MLX90621 GND */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure MLX90621 VCC */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure MLX90621 VCC */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_6);
}

/*******************************************************************************
* : MLX90621_PowerOn
*******************************************************************************/
void MLX90621_PowerOn(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_7); //Reset GND
	GPIO_SetBits(GPIOA,GPIO_Pin_5);    //Set VCC
}

/*******************************************************************************
*I2C_Start
*******************************************************************************/
bool I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return FALSE;        //
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;        //
	SDA_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* I2C_Stop
*******************************************************************************/
void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}

/*******************************************************************************
*  I2C_Ack
*******************************************************************************/
void I2C_Ack(void)
{       
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

/*******************************************************************************
* I2C_NoAck
*******************************************************************************/
void I2C_NoAck(void)
{       
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}
/*******************************************************************************
* I2C_WaitAck
*******************************************************************************/
bool I2C_WaitAck(void)          
{
	SCL_L;
	I2C_delay();
	SDA_H;                       
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
		SCL_L;
		return FALSE;
	}
	SCL_L;
	return TRUE;
}

/*******************************************************************************
* I2C_SendByte
*******************************************************************************/
void I2C_SendByte(u8 SendByte) 
{
    u8 i=8;
    while(i--)
    {
      SCL_L;
      I2C_delay();
      if(SendByte&0x80) {SDA_H;}  
      else {SDA_L;}  
      SendByte<<=1;
      I2C_delay();
      SCL_H;
      I2C_delay();
    }
    SCL_L;
}

/*******************************************************************************
* I2C_ReceiveByte
*******************************************************************************/
u8 I2C_ReceiveByte(void)  
{
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;                               
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
      SCL_H;
      I2C_delay();       
      if(SDA_read) { ReceiveByte|=0x01; }
    }
    SCL_L;
    return ReceiveByte;
}

/*******************************************************************************
* I2C_write
*******************************************************************************/
bool I2C_write(u8* lpu8Buffer, 
               u8 lu8Addr, 
               u8 lu8Size)
{
	u8 lu8WriteAddr, lu8WriteSize,i;
	
	lu8WriteSize = lu8Size;
	lu8WriteAddr = lu8Addr;
	
	if(!I2C_Start()) return FALSE;
	
	I2C_SendByte(lu8WriteAddr);  //send address
	if (!I2C_WaitAck())
	{
		I2C_Stop();
		return FALSE;
	}
	
	for(i=0;i<lu8WriteSize;i++) // start write;
	{
		I2C_SendByte(lpu8Buffer[i]); //write data;
		if (!I2C_WaitAck())
		{
			I2C_Stop();
			return FALSE;
		}	
	}	
	I2C_Stop();		
	return TRUE;
}


