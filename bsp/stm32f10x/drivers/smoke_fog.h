
#ifndef __SMOKE_FOG_H__
#define __SMOKE_FOG_H__

#include "stm32f10x.h"
#include "global.h"
#include "math.h"

/**************** ”≤º˛∆ΩÃ®“¿¿µ ************************************************/
extern  GPIO_InitTypeDef    GPIO_InitStructure;


#define RCC_GPIO_SMOKE_FOG        RCC_APB2Periph_GPIOA
#define SMOKE_FOG_PORT            GPIOA
#define GPIO_Pin_SMOKE_FOG_DAT    GPIO_Pin_5

#define SMOKE_FOG_DATA_INPUT()  st(   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SMOKE_FOG_DAT;\
                                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;\
                                GPIO_Init(SMOKE_FOG_PORT, &GPIO_InitStructure);\
                            )
														
#define SMOKE_FOG_INIT()        st(   RCC_APB2PeriphClockCmd(RCC_GPIO_SMOKE_FOG,ENABLE);\
                                SMOKE_FOG_DATA_INPUT();\
                            )
														

void Smoke_Fog_Init(void);
u8 Smoke_Fog_MeasureOnce(void);

#endif	/*__VOICE_H__*/


