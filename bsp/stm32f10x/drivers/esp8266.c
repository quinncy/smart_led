#include "esp8266.h"
#include "board.h"
#include "usart.h"
#include "echo.h"


void esp8826_hw_init( void )
{
		rt_kprintf("AT\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
	
		rt_kprintf("AT+CWMODE_DEF=1\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
	
		rt_kprintf("AT+CWJAP_DEF=\"TCPtest\",\"1234567890\"\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND * 10 );
	
		rt_kprintf("AT+CIPSTART=\"TCP\",\"192.168.1.10\",1002\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/2 );
	
		rt_kprintf("AT+CIPMODE=1\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
	
		rt_kprintf("AT+CIPSEND\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
}
