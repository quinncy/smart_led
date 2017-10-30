#ifndef __TEST_H__
#define __TEST_H__

#include <rtthread.h>
#include <stm32f10x.h>
#include "test.h"
#include "esp8266.h"
#include "led.h"

rt_err_t demo_thread_creat(void);

void led_thread_entry(void* paramete);

void esp8266_thread_entry(void* parameter);


#endif

