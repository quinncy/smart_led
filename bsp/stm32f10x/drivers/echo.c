#include "echo.h"
#include "led.h"
#include "esp8266.h"
#include "rthw.h"
#include "I2C2.h"
#include "flash.h"

extern struct rt_semaphore send_lock_sem;//用于阻塞mlx线程
extern int PORT;

//全局变量
vu8 esp_rst_flag = 0;
vu8 esp_rst_cnt = 0;

//定义OTA包长
#define REV_DATA_LENGTH 5
#define TICK_DATA_LENGTH 2



 
struct rx_msg
{
    rt_device_t dev;
    rt_size_t   size;
};
 
static struct rt_messagequeue  rx_mq;
static unsigned char uart_rx_buffer[150];
static char msg_pool[1024];

rt_uint32_t debug_length;
unsigned char OTA_buffer[150];
 
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
//	  rt_hw_led_init();
		//初始化8266
		esp8826_hw_init();
   
        // ?RT???????1??
    device = rt_device_find("uart3");
    if (device != RT_NULL)
    {
                           // ?????????????
        rt_device_set_rx_indicate(device, uart_input);
                           // ?????????
        rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
   
		rt_sem_release(&send_lock_sem);//释放一次信号量
    while(1)
    {
                           // ???????????????????????
        result = rt_mq_recv(&rx_mq, &msg, sizeof(struct rx_msg), 80);
        if (result == -RT_ETIMEOUT)
        {
            // timeout, do nothing
        }
       
        if (result == RT_EOK)
        {
            rt_uint32_t rx_length = 64;
						
            rx_length = rt_device_read(msg.dev, 0, &uart_rx_buffer[0], rx_length);
            uart_rx_buffer[rx_length] = '\0';

//						//****************OTA代码***********************************
////						if(rx_length != 0)
////								rt_kprintf("rx_length = %d, uart_rx_buffer[1] = %c\r\n", rx_length, uart_rx_buffer[1]);
//            rt_device_write(device, 0, &uart_rx_buffer[0], rx_length);
									
						if((uart_rx_buffer[0] == 0x55) && (uart_rx_buffer[1] == 0xaa))//接收到OTA数据包
						{
								esp_rst_cnt = 0; //重启计数清零
							
								//修改相应参数
								BEIGHBOR = uart_rx_buffer[3];
								THREDHOLD = uart_rx_buffer[4];
								//将数据写入flash
								WriteFlashOneWord(0, uart_rx_buffer[0] + (uart_rx_buffer[1] << 8) + (uart_rx_buffer[3] << 16) + (uart_rx_buffer[4] << 24));
								//回传数据
//								rt_device_write(device, 0, &uart_rx_buffer[0], REV_DATA_LENGTH);
								//OTA数据回复确认
								rt_kprintf("ota ok,%x,%x\r\n",BEIGHBOR,THREDHOLD);
						}
						else if((rx_length != 0))//接收到心跳包
						{
								esp_rst_cnt = 0; //重启计数清零
						}
						
						if(esp_rst_cnt > 10)//如果超过10次没有收到，证明连接已经中断                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
						{
								esp_rst_flag = 1;//关闭wifi发送操作
								//进行重连操作
								//1.复位wifi引脚
								GPIO_ResetBits(ESP_RST_gpio, ESP_RST_pin);
								rt_thread_delay( RT_TICK_PER_SECOND / 5 );
								GPIO_SetBits(ESP_RST_gpio, ESP_RST_pin);
								
								rt_thread_delay( RT_TICK_PER_SECOND * 5 );//复位后延时一段时间
								//2.进行AT指令连接
		//						rt_kprintf("AT+CIPSTART=\"TCP\",\"192.168.2.83\",%d\r\n", PORT);
								rt_kprintf("AT+CIPSTART=\"TCP\",\"192.168.1.101\",%d\r\n", PORT);
								rt_thread_delay( RT_TICK_PER_SECOND * 3 );
							
								rt_kprintf("AT+CIPMODE=1\r\n");
								rt_thread_delay( RT_TICK_PER_SECOND/10 );
							
								rt_kprintf("AT+CIPSEND\r\n");
								rt_thread_delay( RT_TICK_PER_SECOND/10 );
						
								esp_rst_flag = 0;//打开wifi发送操作
								esp_rst_cnt = 0;//计数值清零
						}

							
        }
				
				
				rt_thread_delay( RT_TICK_PER_SECOND * 3 );
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

