
#ifndef __VOICE_H__
#define __VOICE_H__

#include "stm32f10x.h"
#include "global.h"
#include "math.h"

/**************** ”≤º˛∆ΩÃ®“¿¿µ ************************************************/
extern  GPIO_InitTypeDef    GPIO_InitStructure;


#define RCC_GPIO_VOICE        RCC_APB2Periph_GPIOA
#define VOICE_PORT            GPIOA
#define GPIO_Pin_VOICE_DAT    GPIO_Pin_5

#define VOICE_DATA_INPUT()  st(   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_VOICE_DAT;\
                                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;\
                                GPIO_Init(VOICE_PORT, &GPIO_InitStructure);\
                            )
														
#define VOICE_INIT()        st(   RCC_APB2PeriphClockCmd(RCC_GPIO_VOICE,ENABLE);\
                                VOICE_DATA_INPUT();\
                            )
														

void Voice_Init(void);
u8 Voice_MeasureOnce(void);

#endif	/*__VOICE_H__*/


