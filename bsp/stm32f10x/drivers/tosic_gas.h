
#ifndef __TOSIC_GAS_H__
#define __TOSIC_GAS_H__

#include "stm32f10x.h"
#include "global.h"
#include "math.h"

/**************** ”≤º˛∆ΩÃ®“¿¿µ ************************************************/
extern  GPIO_InitTypeDef    GPIO_InitStructure;


#define RCC_GPIO_TOSIC_GAS        RCC_APB2Periph_GPIOA
#define TOSIC_GAS_PORT            GPIOA
#define GPIO_Pin_TOSIC_GAS_DAT    GPIO_Pin_5

#define TOSIC_GAS_DATA_INPUT()  st(   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_TOSIC_GAS_DAT;\
                                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;\
                                GPIO_Init(TOSIC_GAS_PORT, &GPIO_InitStructure);\
                            )
														
#define TOSIC_GAS_INIT()        st(   RCC_APB2PeriphClockCmd(RCC_GPIO_TOSIC_GAS,ENABLE);\
                                TOSIC_GAS_DATA_INPUT();\
                            )
														

void Tosic_Gas_Init(void);
u8 Tosic_Gas_MeasureOnce(void);

#endif	/*__VOICE_H__*/


