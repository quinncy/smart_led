#include "test.h"
#include <rthw.h>
#include "I2C2.h"
#include "echo.h"
#include "led.h"
#include "esp8266.h"
#include "zet07.h"
#include "bsp_sht2x.h"

/*  变量分配4字节对齐 */
ALIGN(RT_ALIGN_SIZE)

//定义与发送数据包相关的量
#define SENSOR_ID  30
#define SORSOR_DATA_LENGTH 141

//传感器数组
static char sensor_data_buff[150];

/*  静态线程的 线程堆栈*/
static rt_uint8_t module_stack[512];
static rt_uint8_t esp8266_stack[512];
/* 静态线程的 线程控制块 */
static struct rt_thread module_thread;
static struct rt_thread esp8266_thread;

/* 信号量控制块 */
struct rt_semaphore lock_sem;
struct rt_semaphore send_lock_sem;//用于阻塞mlx线程

//全局变量
extern vu8 esp_rst_flag;
extern vu8 esp_rst_cnt;
extern char Binaryzation_8byte[8];
extern short   IMAGE[NROWS][NCOLS];

#if defined(USING_TS_SHT20)//如果使用温湿度传感器
//温湿度传感器结构体定义
SHT_DATA_TYPE SHTxxVal;
#endif

struct CO_rx_msg
{
    rt_device_t dev;
    rt_size_t   size;
};
 
static struct rt_messagequeue  CO_rx_mq;
static unsigned char CO_uart_rx_buffer[150];
static char CO_msg_pool[1024];

rt_err_t CO_uart_input(rt_device_t dev, rt_size_t size)
{
    struct CO_rx_msg CO_msg;
    CO_msg.dev = dev;
    CO_msg.size = size;
   
        // ???????????
    rt_mq_send(&CO_rx_mq, &CO_msg, sizeof(struct CO_rx_msg));
   
    return RT_EOK;
}


rt_err_t demo_thread_creat(void)
{
    rt_err_t result;
		//初始化用于CO的消息队列
		result = rt_mq_init(&CO_rx_mq, "CO_mqt", &CO_msg_pool[0], 128 - sizeof(void*), sizeof(CO_msg_pool), RT_IPC_FLAG_FIFO);
	
		if (result != RT_EOK) 
    { 
        rt_kprintf("init message queue failed.\n"); 
        return -1; 
    }
    /* 初始化静态信号量，初始值是0 */
    result = rt_sem_init(&lock_sem, "locksem", 0, RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        rt_kprintf("init locksem failed.\n");
        return -1;
    }
		
    /* 初始化静态信号量，初始值是0 */
    result = rt_sem_init(&send_lock_sem, "sendlocksem", 0, RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        rt_kprintf("init sendlocksem failed.\n");
        return -1;
    }		

    /* 创建mlx线程 ： 优先级 16 ，时间片 5个系统滴答 */
    result = rt_thread_init(&module_thread,
                            "module",
                            module_thread_entry, RT_NULL,
                            (rt_uint8_t*)&module_stack[0], sizeof(module_stack), 16, 20);

    if (result == RT_EOK)
    {
        rt_thread_startup(&module_thread);
    }
		
    /* 创建esp8266线程 ： 优先级 15 ，时间片 5个系统滴答 */
    result = rt_thread_init(&esp8266_thread,
                            "esp8266",
                            esp8266_thread_entry, RT_NULL,
                            (rt_uint8_t*)&esp8266_stack[0], sizeof(esp8266_stack), 15, 20);

    if (result == RT_EOK)
    {
        rt_thread_startup(&esp8266_thread);
    }
    return 0;
}


void module_thread_entry(void* paramete)
{	
	#if defined(USING_CO_ZE07)//如果使用CO传感器--串口2
		uint8_t CO_ASK[9]={0xff,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};//问答式询问报文
		rt_device_t device;
		rt_err_t result = RT_EOK;
		struct CO_rx_msg CO_msg;
		
		device = rt_device_find("uart2");
    if (device != RT_NULL)
    {
                           // ?????????????
        rt_device_set_rx_indicate(device, CO_uart_input);
                           // ?????????
        rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }		
	rt_thread_delay( RT_TICK_PER_SECOND * 10 );
	#endif
		/* 以永久等待方式获取信号量*/
		rt_sem_take(&send_lock_sem, RT_WAITING_FOREVER);
	#if defined(USING_CO_ZE07)//CO读取数据
//		rt_device_write(device, 0, &CO_ACTIVEMODE[0], CO_BUFFER_SIZE+1);
//		rt_thread_delay( RT_TICK_PER_SECOND/10 );
//		rt_device_write(device, 0, &CO_ACTIVEMODE[1], CO_BUFFER_SIZE);
	#endif
	  while(1)
		{
				rt_thread_delay( RT_TICK_PER_SECOND/1 );
			
				rt_enter_critical();
			
		#if defined(USING_MLX90621)//红外读取数据
				MLX90621IR_ReadSead();
		#endif
			
		#if defined(USING_CO_ZE07)//CO读取数据
				//采用问答式方式获取数据
				//1.首先通过串口发送问格式报文
//				rt_device_write(device, 0, &CO_ASK[0], CO_BUFFER_SIZE);
			
				//2.延时一小段时间
				rt_thread_delay( RT_TICK_PER_SECOND/10 );//延时20ms   1000/50==20ms
			
				//3.读取缓存区里面的CO数据
				result = rt_mq_recv(&CO_rx_mq, &CO_msg, sizeof(struct CO_rx_msg), 80);
				if (result == -RT_ETIMEOUT)
        {
            // timeout, do nothing
        }
				
				if (result == RT_EOK)
				{
						rt_uint32_t rx_length = 64;
						rt_uint8_t cnt;
						rx_length = rt_device_read(CO_msg.dev, 0, &CO_uart_rx_buffer[0], rx_length);
						CO_uart_rx_buffer[rx_length] = '\0';
						
						for(cnt = 0; cnt < rx_length; cnt++)
						{
								if(CO_uart_rx_buffer[cnt] != 0xFF)	continue;//如果不是0xff，跳过
								else if(CO_uart_rx_buffer[cnt+1] == 0x04 && CO_uart_rx_buffer[cnt+7] == 0x88)//如果为格式报文
								{
										CO_value = (CO_uart_rx_buffer[cnt+4]*256 + CO_uart_rx_buffer[cnt+5])*0.1;//计算得到CO的值
										break;
								}
						}
//						if(CO_uart_rx_buffer[0] == 0xFF && CO_uart_rx_buffer[1] == 0x04 && CO_uart_rx_buffer[7] == 0x88)//判断是否为接收CO数据包
//						{
//								CO_value = (CO_uart_rx_buffer[4]*256 + CO_uart_rx_buffer[5])*0.1;//计算得到CO的值
//						}
				}
		#endif
				
		#if defined(USING_TS_SHT20)//如果使用温湿度传感器
				Bsp_ShtMeasureOnce(&SHTxxVal);
		#endif
			
				rt_exit_critical();
				if(!esp_rst_flag)
						rt_sem_release(&lock_sem);//释放一次信号量
				
		}
}

void esp8266_thread_entry(void* parameter)
{
		vu8 cnt = 0;

    rt_device_t device;

		device = rt_device_find("uart3");
    if (device != RT_NULL)
    {
                           // ?????????????
        rt_device_set_rx_indicate(device, uart_input);
                           // ?????????
        rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
    /* 无限循环*/
    while (1)
    {
        /* 以永久等待方式获取信号量*/
        rt_sem_take(&lock_sem, RT_WAITING_FOREVER);
			
				rt_enter_critical();//调度器上锁
			
//				esp_rst_cnt++; //复位计数变量自增
			
        /* 当得到信号量以后才有可能执行下面程序*/
			
		#if defined(USING_MLX90621)//红外读取数据
				//填充数据包数组
				for(cnt = 0; cnt < SORSOR_DATA_LENGTH; cnt++)
				{
						if(cnt == 0)
								sensor_data_buff[cnt] = SENSOR_ID / 10 + '0';
						else if(cnt == 1)
								sensor_data_buff[cnt] = SENSOR_ID % 10 + '0';
						else if(cnt == 2)
								sensor_data_buff[cnt] = SORSOR_DATA_LENGTH;
						else if(cnt < 11)
								sensor_data_buff[cnt] = Binaryzation_8byte[cnt - 3];
						else if(cnt < 139)
						{
								if(cnt % 2 == 1)
										sensor_data_buff[cnt] = *(&IMAGE[0][0] + (cnt - 1) / 2 - 5) / 256;
								else
										sensor_data_buff[cnt] = *(&IMAGE[0][0] + (cnt - 1) / 2 - 5) % 256;
						}		
						else if(cnt == 139)
								sensor_data_buff[cnt] = 0x5a;
						else if(cnt == 140)
								sensor_data_buff[cnt] = 0xa5;
								
				}
				sensor_data_buff[SORSOR_DATA_LENGTH] = '\0';
				//发送红外传感器数据包
				rt_device_write(device, 0, &sensor_data_buff[0], SORSOR_DATA_LENGTH);
		#endif
			
		#if defined(USING_CO_ZE07)
				//填充数据包数组
				
		#endif
				
				rt_exit_critical();

    }	

}





