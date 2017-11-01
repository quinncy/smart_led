#ifndef __ESP8266_H__
#define __ESP8266_H__

#include "rtthread.h"
#include "stm32f10x.h"

#define ESP_RST_rcc                    RCC_APB2Periph_GPIOC
#define ESP_RST_gpio                   GPIOC
#define ESP_RST_pin                    (GPIO_Pin_13)

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

void esp8826_hw_init( void );

	

#endif

