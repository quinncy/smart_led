#include "esp8266.h"
#include "board.h"
#include "usart.h"
#include "echo.h"

//--------------------------------
//定义端口号
int PORT = 1007;


void esp8826_hw_init( void )
{
		//初始化复位引脚
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(ESP_RST_rcc,ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin   = ESP_RST_pin;
    GPIO_Init(ESP_RST_gpio, &GPIO_InitStructure);
		
	
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
