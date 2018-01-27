#ifndef __ECHO_H__
#define __ECHO_H__


#include "stm32f10x.h"
#include "rtthread.h"

////是否使用红外传感器部分
//#define USING_MLX90621

////是否使用CO传感器部分
//#define USING_CO_ZE07

////是否使用温湿度传感器
//#define USING_TS_SHT20

//是否使用语音传感器部分
#define USING_VOICE

//是否使用烟雾传感器部分
#define USING_SMOKE_FOG

//是否使用毒害气体传感器部分
#define USING_TOSIC_GAS

rt_err_t uart_input(rt_device_t dev, rt_size_t size);
void usr_echo_thread_entry(void* parameter);
void usr_echo_init( void );




#endif

