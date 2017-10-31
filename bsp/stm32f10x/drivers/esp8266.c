#include "esp8266.h"
#include "board.h"
#include "usart.h"
#include "echo.h"


void esp8826_hw_init( void )
{
		//--------------------------------
		//¶¨Òå¶Ë¿ÚºÅ
		const int PORT = 1007;
	
	
		rt_kprintf("AT\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
	
		rt_kprintf("AT+CWMODE_DEF=1\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
	
		rt_kprintf("AT+CWJAP_DEF=\"TCPtest\",\"1234567890\"\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND * 10 );
	
		rt_kprintf("AT+CIPSTART=\"TCP\",\"192.168.1.10\",%d\r\n", PORT);
		rt_thread_delay( RT_TICK_PER_SECOND * 3 );
	
		rt_kprintf("AT+CIPMODE=1\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
	
		rt_kprintf("AT+CIPSEND\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
}
