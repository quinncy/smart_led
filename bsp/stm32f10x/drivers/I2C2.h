/*******************************************************************************
* File Name          : i2c_fram.h
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : Header for i2c_ee.c module
*******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */

#ifndef __I2C2_H
#define __I2C2_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#include "stdbool.h" 

#define NROWS           4
#define NCOLS           16


extern unsigned char BEIGHBOR;
extern unsigned char THREDHOLD;


/* Exported types ------------------------------------------------------------*/

typedef __packed struct
{
	u8 bOperationFlag;
	u16 u16OperationDelay[9];
} stI2CWR;

/* Exported constants --------------------------------------------------------*/
/* Exported variable ------------------------------------------------------------*/
extern u16 u16PORDelay5ms;
extern u16 u16PORDelay2ms;
extern stI2CWR  stI2CWriteRead;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

bool I2C_readEeprom(void);
bool I2C_writeDevConfig(void); 
bool I2C_writeOscTrim(void);	
bool I2C_readDevConfig(void);
bool I2C_readOscTrim(void);
bool I2C_readPTAT(void);
bool I2C_readIRData(void);
bool I2C_readPixelCompensation(void);
void MLX90621IR_ReadSead(void);
void BackgroundUpdate(short image[4][16]);
void MLX90621_WorkInit(void);
void Get_sensor_data(void);
void BackgroundInit(void);

#endif
