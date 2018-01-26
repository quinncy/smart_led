
#ifndef __ZET07_H__
#define __ZET07_H__

#include "stm32f10x.h"
#include "rtthread.h"


#define CO_BUFFER_SIZE    9

extern float CO_value;//COÅ¨¶È

extern uint8_t CO_ASKMODE[10];
extern uint8_t CO_ACTIVEMODE[10];
extern uint8_t CO_RxBuffer[CO_BUFFER_SIZE];

#endif /*__ZET07_H__*/

