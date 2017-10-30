#ifndef __ESP8266_H__
#define __ESP8266_H__

#include "rtthread.h"
#include "stm32f10x.h"

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

void esp8826_hw_init( void );

	

#endif

