#include "echo.h"
#include "led.h"
#include "esp8266.h"

 
struct rx_msg
{
    rt_device_t dev;
    rt_size_t   size;
};
 
static struct rt_messagequeue  rx_mq;
static char uart_rx_buffer[150];
static char msg_pool[1024];
 
// ????????
rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    msg.dev = dev;
    msg.size = size;
   
        // ???????????
    rt_mq_send(&rx_mq, &msg, sizeof(struct rx_msg));
   
    return RT_EOK;
}
 
// ??????
void usr_echo_thread_entry(void* parameter)
{
    struct rx_msg msg;
		static vu8 led_state = 0;
   
    rt_device_t device;
    rt_err_t result = RT_EOK;
	
		//初始化LED
	  rt_hw_led_init();
		//初始化8266
		rt_thread_delay( RT_TICK_PER_SECOND/2 );
		esp8826_hw_init();
   
        // ?RT???????1??
    device = rt_device_find("uart1");
    if (device != RT_NULL)
    {
                           // ?????????????
        rt_device_set_rx_indicate(device, uart_input);
                           // ?????????
        rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
   
    while(1)
    {
				rt_thread_delay( RT_TICK_PER_SECOND * 3 );
                           // ???????????????????????
        result = rt_mq_recv(&rx_mq, &msg, sizeof(struct rx_msg), 80);
        if (result == -RT_ETIMEOUT)
        {
            // timeout, do nothing
        }
       
        if (result == RT_EOK)
        {
            rt_uint32_t rx_length = 5;
						
           
//            rx_length = (sizeof(uart_rx_buffer) - 1) > msg.size ?
//                msg.size : sizeof(uart_rx_buffer) - 1;
           
            rx_length = rt_device_read(msg.dev, 0, &uart_rx_buffer[0], rx_length);
            uart_rx_buffer[rx_length] = '\0';
						if(rx_length == 0)
								continue;
            // ????????1
						rt_kprintf("rx_length = %d, uart_rx_buffer[1] = %c\r\n", rx_length, uart_rx_buffer[1]);
//            rt_device_write(device, 0, &uart_rx_buffer[0], rx_length);
						if(uart_rx_buffer[1] == 'n' && rx_length == 5)
						{
								led_state ^=1;
								if (led_state!=0)
								{
											rt_hw_led_on(0);
//										rt_kprintf(" get OTA 1 \r\n");
								}
								else
								{
											rt_hw_led_off(0);
//										rt_kprintf(" get OTA 0 \r\n");
								}
						}
        }
    }
}
// ?????????
void usr_echo_init( void )
{
    rt_thread_t thread ;
   
    rt_err_t result; 
      // ??????,????????
    result = rt_mq_init(&rx_mq, "mqt", &msg_pool[0], 128 - sizeof(void*), sizeof(msg_pool), RT_IPC_FLAG_FIFO);
   
    if (result != RT_EOK) 
    { 
        rt_kprintf("init message queue failed.\n"); 
        return; 
    } 
    // ??????
    thread = rt_thread_create("devt",
        usr_echo_thread_entry, RT_NULL,
        1024, 14, 50);
    // ??????
    if (thread != RT_NULL)
        rt_thread_startup(thread);

}

