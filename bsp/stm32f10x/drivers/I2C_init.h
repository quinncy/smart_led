#ifndef __I2C_init_H
#define __I2C_init_H

#include "stm32f10x.h"

#include "stdbool.h" 

#define TRUE  1
#define FALSE 0

#define SCL_H         GPIOB->BSRR = GPIO_Pin_8
#define SCL_L         GPIOB->BRR  = GPIO_Pin_8
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_9
#define SDA_L         GPIOB->BRR  = GPIO_Pin_9

#define SCL_read      GPIOB->IDR  & GPIO_Pin_8
#define SDA_read      GPIOB->IDR  & GPIO_Pin_9

void I2C_delay(void);
void GPIO_Configuration_I2C(void);
void MLX90621_PowerOn(void);
bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
bool I2C_WaitAck(void);
void I2C_SendByte(u8 SendByte);
u8 I2C_ReceiveByte(void);

bool I2C_write(u8* lpu8Buffer, 
               u8 lu8Addr, 
               u8 lu8Size);


#endif
