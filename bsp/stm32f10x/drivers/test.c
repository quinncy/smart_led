#include "test.h"
#include <rthw.h>

/*  变量分配4字节对齐 */
ALIGN(RT_ALIGN_SIZE)

/*  静态线程的 线程堆栈*/
static rt_uint8_t mlx_stack[512];
static rt_uint8_t esp8266_stack[512];
/* 静态线程的 线程控制块 */
static struct rt_thread mlx_thread;
static struct rt_thread esp8266_thread;

/* 信号量控制块 */
struct rt_semaphore lock_sem;

rt_err_t demo_thread_creat(void)
{
    rt_err_t result;

    /* 初始化静态信号量，初始值是0 */
    result = rt_sem_init(&lock_sem, "locksem", 0, RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        rt_kprintf("init locksem failed.\n");
        return -1;
    }


    /* 创建mlx线程 ： 优先级 16 ，时间片 5个系统滴答 */
    result = rt_thread_init(&mlx_thread,
                            "mlx",
                            mlx_thread_entry, RT_NULL,
                            (rt_uint8_t*)&mlx_stack[0], sizeof(mlx_stack), 16, 20);

    if (result == RT_EOK)
    {
        rt_thread_startup(&mlx_thread);
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


void mlx_thread_entry(void* paramete)
{	
		rt_thread_delay( RT_TICK_PER_SECOND * 5 );
	  while(1)
		{
				rt_thread_delay( RT_TICK_PER_SECOND/1 );
			
				rt_sem_release(&lock_sem);//释放一次信号量
				
		}
}

void esp8266_thread_entry(void* parameter)
{
		vu8 led_state = 0;	
//		//初始化8266
//		rt_thread_delay( RT_TICK_PER_SECOND/2 );
//		esp8826_hw_init();
	
		
    /* 无限循环*/
    while (1)
    {
        /* 以永久等待方式获取信号量*/
        rt_sem_take(&lock_sem, RT_WAITING_FOREVER);
        /* 当得到信号量以后才有可能执行下面程序*/
        led_state ^=1;
        if (led_state!=0)
        {
//            rt_hw_led_on(0);
            rt_kprintf(" get semaphore ok, take 1 \r\n");
        }
        else
        {
//            rt_hw_led_off(0);
            rt_kprintf(" get semaphore ok, take 0 \r\n");
        }
    }	

}





