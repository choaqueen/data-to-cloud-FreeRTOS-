#include "stm32f4xx.h"
#include "bsp_led.h"
#include "bsp_UART.h"
#include <stdbool.h> // 添加这一行，使用标准的 bool, true, false

#include "bsp_DHT11.h"
#include "delay.h"
#include "onenet.h"
#include "esp8266.h"
#include <string.h>

#include "ov7670.h"
#include "dcmi.h"
#include "ov7670test.h"
#include "rgb2bmp.h"
#include "sys_cfg.h"
#include "tiny_jpeg.h"
#include <time.h> // 时间戳所用头文件
#include "bsp_LCD_ILI9341.h"
#include "base64.h"
#include "rgb2bmp.h"
#include "gps.h"
#include "huawei.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#define USART_DEBUG		USART1		//调试打印所使用的串口组
#define TEMP_X   20
#define TEMP_Y   80
#define HUMI_X   20
#define HUMI_Y   130
#define TEXT_COLOR   WHITE
#define BG_COLOR     BLACK

// MQTT 主题（自定义，用于上传分片）
#define DEVICE_ID       "69d31f3de094d615922eeaf2_dev1_0_0_2026040605"
#define IMAGE_CHUNK_TOPIC "/user/image_upload"

#define BMP_FILE_SIZE   (54 + PIC_WIDTH * PIC_HEIGHT * 2)   // 38,454 字节
// 分片参数（每片原始数据大小，Base64 后约 4KB，MQTT 可承受）
#define CHUNK_RAW_SIZE  1000
#define MAX_CHUNKS      ((BMP_FILE_SIZE + CHUNK_RAW_SIZE - 1) / CHUNK_RAW_SIZE)

// 在 camera_buffer 定义附近添加
#define BMP_BUFFER_SIZE (54 + PIC_WIDTH * PIC_HEIGHT * 2)

#define DHT11_GPIO_PORT     GPIOC       // DHT11 数据引脚所在的 GPIO 端口
#define DHT11_GPIO_PIN      GPIO_Pin_3  // DHT11 数据引脚号
//static uint8_t bmp_buffer[BMP_BUFFER_SIZE];

// 临时缓冲区（用于分片 Base64 编码和 MQTT payload）
//static uint8_t chunk_raw[CHUNK_RAW_SIZE];      
//static char    base64_buf[CHUNK_RAW_SIZE * 4 / 3 + 8];
//static char    mqtt_payload[2048];   // JSON 格式，大小足够
extern uint16_t camera_buffer[]; 
extern unsigned char esp8266_buf[512];
uint8_t g_image_pool[BMP_BUFFER_SIZE] __attribute__((aligned(4)));
extern const unsigned char bmp_128_128[ ];               // 图片数组声明; 存放于font.h文件中。这里仅作示范，不建议使用, 太吃资源了。
//static uint16_t image_seq = 0;   // 图片序号，用于云端区分不同图片
/********************************************************************************************************************
*                                                 全局变量
*********************************************************************************************************************
*/

#define NET_READY_BIT   (0x01 << 0)
typedef struct {
    float temperature;
    float humidity;
} SensorData_t;

float temperature=0, humidity=0;

/********************************************************************************************************************
*                                                 函数声明
*********************************************************************************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */
static void LCD_show(void);

static void SensorTask(void* pvParameters);/* TEMP_Task任务实现 */
static void DisplayTask(void* pvParameters);/* IMAGEshot_Task任务实现 */
static void NetInitTask(void* pvParameters);/* GPS_Task任务实现 */
static void DhtUploadTask(void* pvParameters);/* huawei_Task任务实现 */
//static void ImageUploadTask(void* pvParameters);/* ILI9341_Task任务实现 */

static void BSP_Init(void);/* 用于初始化板载相关资源 */

/**************************** 任务句柄 ********************************/
/* 
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;/* 创建任务句柄 */
static TaskHandle_t SensorTask_Handle = NULL;/* SensorTask任务句柄 */
static TaskHandle_t DisplayTask_Handle = NULL;/* DisplayTask任务句柄 */
static TaskHandle_t NetInitTask_Handle = NULL;/* NetInitTask任务句柄 */
static TaskHandle_t DhtUploadTask_Handle = NULL;/* DhtUploadTask任务句柄 */
//static TaskHandle_t ImageUploadTask_Handle = NULL;/* ImageUploadTask任务句柄 */

/********************************** 内核对象句柄 *********************************/
/*
 * 信号量，消息队列，事件标志组，软件定时器这些都属于内核的对象，要想使用这些内核
 * 对象，必须先创建，创建成功之后会返回一个相应的句柄。实际上就是一个指针，后续我
 * 们就可以通过这个句柄操作这些内核对象。
 *
 * 内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，
 * 任务间的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数
 * 来完成的
 * 
 */

static QueueHandle_t      xSensorQueue = NULL;
static SemaphoreHandle_t  xEspMutex = NULL;      /* 保护 UART2 / ESP8266 */
static SemaphoreHandle_t  xFlashMutex = NULL;    /* 保护 W25Q128：字库 + 图片共用 */
static EventGroupHandle_t xNetEventGroup = NULL; /* 网络就绪事件组 */

void LCD_show(void)
{
    LCD_Fill (0, 0, 240, 320, WHITE);                    // 整个背景填充白色
    static char strTemp[30];                            
    LCD_String(5, 50, " 温度:  ",24, WHITE, GREEN);      // 本函数，可显示中英文字符串; 函数内自动选择font.h中的ASCII字模、Flash中的汉字字模; Flash中已存在放较为完成的汉字字模数据，四字字号，12号、16号、24号、32号，约6M大小
    //float temperature = 25.3f;                           // 假设已采集的温度。用于展示如何显示小数等数据;
    float temperature = DHT11_GetTemperature(DHT11_GPIO_PORT,DHT11_GPIO_PIN);
    //sprintf(strTemp, "%4.1f℃    ", temperature);        // 重要：很多人咨询, 如何把各种数据类型，转换为字符串，再显示到显示屏: sprintf函数有神效，请自行百度用法;
    snprintf(strTemp, sizeof(strTemp), "%4.1f℃    ", temperature);
    LCD_String(85, 50,strTemp, 24, BLACK, GREEN);        // 显示数值到显示屏上                                             
                                                        
    LCD_String(5, 85, " 湿度:  ",24, BLACK, GBLUE);                      
    //float humidity = 48.0f;                              // 假设已采集的温度。用于展示如何显示小数等数据;
    float humidity = DHT11_GetHumidity(DHT11_GPIO_PORT,DHT11_GPIO_PIN);
    //sprintf(strTemp, "%2.1f %%    ", humidity);          // 重要：很多人咨询, 如何把各种数据类型，转换为字符串，再显示到显示屏: sprintf函数有神效，请自行百度用法;
    snprintf(strTemp, sizeof(strTemp), "%2.1f %%    ", humidity);
    LCD_String(85, 85, strTemp, 24, BLACK, GBLUE);       // 显示数值到显示屏上 
    LCD_String(5, 120, " 联网中   ", 24, WHITE, RED);
    delay_ms(2000);
}

static volatile uint32_t ulTimesMS;    // 使用volatile声明，防止变量被编译器优化
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS)
        ulTimesMS--;                   // 操作外部变量，防止空循环被编译器优化掉
}

void LCD_show1(void)
{
    LCD_Fill (0, 0, 240, 320, WHITE);                    // 整个背景填充白色
                               
    LCD_String(5, 50, " 温度：  ",24, WHITE, GREEN);      // 本函数，可显示中英文字符串; 函数内自动选择font.h中的ASCII字模、Flash中的汉字字模; Flash中已存在放较为完成的汉字字模数据，四字字号，12号、16号、24号、32号，约6M大小
                                                   
    LCD_String(5, 85, " 湿度：  ",24, BLACK, GBLUE); 
}
void LCD_show2(void)
{
    LCD_Fill (0, 0, 240, 320, WHITE);                    // 整个背景填充白色
                               
    LCD_String(5, 50, " 温度数据上传完成 ",24, WHITE, GREEN);      // 本函数，可显示中英文字符串; 函数内自动选择font.h中的ASCII字模、Flash中的汉字字模; Flash中已存在放较为完成的汉字字模数据，四字字号，12号、16号、24号、32号，约6M大小
                                                   
    LCD_String(5, 85, " 湿度数据上传完成 ",24, BLACK, GBLUE); 
}

int main(void)
{                             
    BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
  /* 开发板硬件初始化 */
  BSP_Init();
  
   /* 创建AppTaskCreate任务 */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                        (const char*    )"AppTaskCreate",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* 任务控制块指针 */ 
  /* 启动任务调度 */           
  if(pdPASS == xReturn)
    vTaskStartScheduler();   /* 启动任务，开启调度 */
  else
    return -1;  
  
  while(1)
{
    UsartPrintf(USART_DEBUG, "Test\r\n");
    for(int i=0; i<1000000; i++);
}   /* 正常不会执行到这里 */   
}


/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
  taskENTER_CRITICAL();           //进入临界区
  
  /* 创建xSensorQueue */
  xSensorQueue  = xQueueCreate(1, sizeof(SensorData_t));
  if(NULL != xSensorQueue)
  UsartPrintf(USART_DEBUG, "创建xSensorQueue消息队列成功!\r\n");


  /* 创建xEspMutex */
  xEspMutex = xSemaphoreCreateMutex();
  if(NULL != xEspMutex)
  UsartPrintf(USART_DEBUG, "创建xEspMutex互斥量成功!\r\n");
  xReturn = xSemaphoreGive(xEspMutex);           //给出互斥量
  if(xReturn == pdTRUE)
  UsartPrintf(USART_DEBUG, "释放xEspMutex互斥量!\r\n");

  /* 创建xFlashMutex */
  xFlashMutex  = xSemaphoreCreateMutex();
  if(NULL != xFlashMutex)
  UsartPrintf(USART_DEBUG, "创建xFlashMutex互斥量成功!\r\n");
  xReturn = xSemaphoreGive(xFlashMutex);         //给出互斥量

  if(xReturn == pdTRUE)
  UsartPrintf(USART_DEBUG, "释放xFlashMutex互斥量!\r\n");

  /* 创建xNetEventGroup */
  xNetEventGroup = xEventGroupCreate();
  if(NULL != xNetEventGroup)
  UsartPrintf(USART_DEBUG, "xNetEventGroup事件创建成功!\r\n");

  /* 创建SensorTask任务 */
  xReturn = xTaskCreate((TaskFunction_t )SensorTask, /* 任务入口函数 */
                        (const char*    )"SensorTask",/* 任务名字 */
                        (uint16_t       )256,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )3,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&SensorTask_Handle);/* 任务控制块指针 */
  if(pdPASS == xReturn)
    UsartPrintf(USART_DEBUG, "创建SensorTask任务!\r\n");
  /* 创建DisplayTask任务 */
  xReturn = xTaskCreate((TaskFunction_t )DisplayTask,  /* 任务入口函数 */
                        (const char*    )"DisplayTask",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )3, /* 任务的优先级 */
                        (TaskHandle_t*  )&DisplayTask_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    UsartPrintf(USART_DEBUG, "创建DisplayTask任务成功!\r\n");
  /* 创建NetInitTask任务 */
  xReturn = xTaskCreate((TaskFunction_t )NetInitTask, /* 任务入口函数 */
                        (const char*    )"NetInitTask",/* 任务名字 */
                        (uint16_t       )512,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )4,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&NetInitTask_Handle);/* 任务控制块指针 */
  if(pdPASS == xReturn)
    UsartPrintf(USART_DEBUG, "创建NetInitTask任务成功!\r\n");
  /* 创建DhtUploadTask任务 */
  xReturn = xTaskCreate((TaskFunction_t )DhtUploadTask,  /* 任务入口函数 */
                        (const char*    )"DhtUploadTask",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )3, /* 任务的优先级 */
                        (TaskHandle_t*  )&DhtUploadTask_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    UsartPrintf(USART_DEBUG, "创建DhtUploadTask任务成功!\r\n");
  /* 创建ImageUploadTask任务 */
//  xReturn = xTaskCreate((TaskFunction_t )ImageUploadTask,  /* 任务入口函数 */
//                        (const char*    )"ImageUploadTask",/* 任务名字 */
//                        (uint16_t       )10000,  /* 任务栈大小 */
//                        (void*          )NULL,/* 任务入口函数参数 */
//                        (UBaseType_t    )2, /* 任务的优先级 */
//                        (TaskHandle_t*  )&ImageUploadTask_Handle);/* 任务控制块指针 */ 
//  if(pdPASS == xReturn)
//    UsartPrintf(USART_DEBUG, "创建ImageUploadTask任务成功!\r\n");
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}
///**********************************************************************
//  * @ 函数名  ： SensorTask
//  * @ 功能说明： SensorTask任务主体
//  * @ 参数    ：   
//  * @ 返回值  ： 无
//  ********************************************************************/
static void SensorTask(void* parameter)
{	
    SensorData_t data;
    TickType_t last = xTaskGetTickCount();

    for (;;)
    {
        data.temperature = DHT11_GetTemperature(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
        data.humidity    = DHT11_GetHumidity   (DHT11_GPIO_PORT, DHT11_GPIO_PIN);

        /* 队列长度 1，覆盖写，保证其他任务总是拿到最新值 */
        xQueueOverwrite(xSensorQueue, &data);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(2000));
    }
}
/**********************************************************************
  * @ 函数名  ： DisplayTask
  * @ 功能说明： DisplayTask任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void DisplayTask(void* parameter)
{	
     SensorData_t data;
     EventBits_t r_event;
    static char strTemp[32];
    for (;;)
    {
        if (xQueuePeek(xSensorQueue, &data, pdMS_TO_TICKS(100)) == pdPASS)
        {
            
            if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(500)) == pdPASS)
            {
               /* LCD_String 内部会读 W25Q128 汉字字模，
               必须与图片上传的 Flash 读操作串行化。
               超时设 500ms，防止图片上传长时间占用时本任务一直阻塞。 */
                W25Q128_Init ();                                     // 外部Flash 初始化，其内部存储有汉字字模数据
                LCD_Init1();
                LCD_show1();
                snprintf(strTemp, sizeof(strTemp), "%4.1f℃    ", data.temperature);
                LCD_String(85, 50, strTemp, 24, BLACK, GREEN);
                
                snprintf(strTemp, sizeof(strTemp), "%4.1f %%    ", data.humidity);
                LCD_String(85, 85, strTemp, 24, BLACK, GBLUE);
                r_event = xEventGroupWaitBits(xNetEventGroup,
                        NET_READY_BIT,
                        pdFALSE,        /* 不清位，其他任务也要用 */
                        pdTRUE,         /* 等所有位（这里只有 1 位） */
                        //portMAX_DELAY
                        0                );
                if ((r_event & (NET_READY_BIT)) == (NET_READY_BIT))
                {
                    LCD_String(5, 120, " 已联网   ", 24, WHITE, YELLOW);
                    //LCD_String(5, 155, " 数据上传中   ", 24, WHITE, BRRED);
                }
                else
                    LCD_String(5, 120, " 未联网   ", 24, WHITE, YELLOW);
                xSemaphoreGive(xFlashMutex);
            }
            /* 拿不到锁就放弃本次刷新，下个 1s 周期再试，避免死等 */
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
/**********************************************************************
  * @ 函数名  ： NetInitTask
  * @ 功能说明： NetInitTask任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void NetInitTask(void* parameter)
{	
       /* 1) 屏上提示"联网中..." */
    if (xSemaphoreTake(xFlashMutex, portMAX_DELAY) == pdPASS)
    {
        LCD_String(5, 120, " 联网中  ", 24, WHITE, RED);
        xSemaphoreGive(xFlashMutex);
    }

    /* 2) 初始化 ESP8266（void 返回，内部处理重试/错误） */
    ESP8266_Init();
    UsartPrintf(USART_DEBUG, "ESP8266 Init done\r\n");

    /* 3) 连接华为云，失败重试；连续失败时重新 Init */
    uint32_t fail_cnt = 0;
    while (HuaWei_DevLink() != 0)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        fail_cnt++;
        if (fail_cnt >= 20)  // 约 10 秒还没连上，重新初始化 ESP8266
        {
            UsartPrintf(USART_DEBUG, "HuaWei link retry %u, re-init ESP8266\r\n", fail_cnt);
            ESP8266_Init();
            fail_cnt = 0;
        }
    }
    UsartPrintf(USART_DEBUG, "HuaWei Cloud Connected\r\n");

    /* 4) 屏上提示"已联网" */
    if (xSemaphoreTake(xFlashMutex, portMAX_DELAY) == pdPASS)
    {
        LCD_String(5, 120, " 已联网   ", 24, WHITE, YELLOW);
        xSemaphoreGive(xFlashMutex);
    }

    /* 5) 置位网络就绪位 */
    xEventGroupSetBits(xNetEventGroup, NET_READY_BIT);

    /* 6) 删除自身 */
    vTaskDelete(NULL);
}

/**********************************************************************
  * @ 函数名  ： DhtUploadTask
  * @ 功能说明： DhtUploadTask任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void DhtUploadTask(void* parameter)
{	
      /* 阻塞等待 NetInitTask 完成 ESP8266 初始化并连云成功 */
    xEventGroupWaitBits(xNetEventGroup,
                        NET_READY_BIT,
                        pdFALSE,        /* 不清位，其他任务也要用 */
                        pdTRUE,         /* 等所有位（这里只有 1 位） */
                        portMAX_DELAY);

    /* 注意：last 必须在等待之后取，否则 vTaskDelayUntil 会因时间差过大而立即触发 */
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        //if (xSemaphoreTake(xEspMutex, portMAX_DELAY) == pdPASS)
        if (xSemaphoreTake(xEspMutex, pdMS_TO_TICKS(500)) == pdPASS)
        {
            //LCD_String(5, 190, " 已联网   ", 24, WHITE, YELLOW);
            
            Upload_DHT11_To_HuaweiCloud();
            xSemaphoreGive(xEspMutex);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(5000));
    }
}

/**********************************************************************
  * @ 函数名  ： ImageUploadTask
  * @ 功能说明： ImageUploadTask任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
//static void ImageUploadTask(void* parameter)
//{	
//   /* 阻塞等待 NetInitTask 完成 ESP8266 初始化并连云成功 */
//    xEventGroupWaitBits(xNetEventGroup,
//                        NET_READY_BIT,
//                        pdFALSE,        /* 不清位，其他任务也要用 */
//                        pdTRUE,         /* 等所有位（这里只有 1 位） */
//                        portMAX_DELAY);

//    /* 注意：last 必须在等待之后取，否则 vTaskDelayUntil 会因时间差过大而立即触发 */
//    TickType_t last = xTaskGetTickCount();
//    for (;;)
//    {
//     if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(500)) == pdPASS)
//      {
//        if (xSemaphoreTake(xEspMutex, pdMS_TO_TICKS(500)) == pdPASS)
//        {
//            //LCD_String(5, 120, " 已联网   ", 24, WHITE, YELLOW);
//            LCD_String(5, 155, " 数据上传中   ", 24, WHITE, BRRED);
//            if (OV7670_Init() != 0)
//	{
//        UsartPrintf(USART_DEBUG, "Ov7670 Init Failed.\r\n");
//	}
//	else
//	{
//         UsartPrintf(USART_DEBUG, "Ov7670 Init Succeed.\r\n");	
//#ifdef OV7670_DBG
//		OV7670_USART_Init();
//#endif
//            Upload_Image_To_HuaWei_Chunked(); 
//            UsartPrintf(USART_DEBUG, "test1!\r\n");
//            LCD_Init1();
//            W25Q128_Init();
//            LCD_show2();
//            LCD_String(5, 120, " 已联网   ", 24, WHITE, YELLOW);
//            LCD_String(5, 155, " 图片数据上传完成 ", 24, WHITE, BRRED);
//    }
//            xSemaphoreGive(xEspMutex);
//            UsartPrintf(USART_DEBUG, "test2!\r\n");
//        }
//            UsartPrintf(USART_DEBUG, "before give flash!\r\n");
//        xSemaphoreGive(xFlashMutex);
//            UsartPrintf(USART_DEBUG, "after give flash!\r\n");
//            UsartPrintf(USART_DEBUG, "test3!\r\n");
//      }
//        vTaskDelayUntil(&last, pdMS_TO_TICKS(30000));
//    }
//}

/***********************************************************************
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 板级外设初始化，所有板子上的初始化均可放在这个函数里面
  * @ 参数    ：   
  * @ 返回值  ： 无
  *********************************************************************/
static void BSP_Init(void)
{
	/*
	 * STM32中断优先级分组为4，即4bit都用来表示抢占优先级，范围为：0~15
	 * 优先级分组只需要分组一次即可，以后如果有其他的任务需要用到中断，
	 * 都统一用这个优先级分组，千万不要再分组，切忌。
	 */
	  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); // 设置系统中断优先级分组4
      UART1_Init(115200);                                                   // UART1 初始化，用于与串口软件通信; TX-PA9、RX-PA10, 115200-8-N-1，中断发送、接收
      UART2_Init(115200);
      W25Q128_Init ();                                     // 外部Flash 初始化，其内部存储有汉字字模数据
      LCD_Init();
      DHT11_Init(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
      LCD_show();
      UsartPrintf(USART_DEBUG, " Hardware init OK\r\n");    
}
// 每个代码文件的末尾，注意要加两个空行

