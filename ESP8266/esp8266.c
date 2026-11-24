//单片机头文件
#include "stm32f4xx.h"

//网络设备驱动
#include "esp8266.h"

//硬件驱动
#include "delay.h"
#include "bsp_UART.h"
//#include "oled.h"
//C库
#include <string.h>
#include <stdio.h>
#include "cJSON.h"
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#define ESP8266_WIFI_INFO		"AT+CWJAP=\"www\",\"12345678\"\r\n"           //WIFI名称与密码
#define USART_DEBUG		USART1		//调试打印所使用的串口组
#define ALI_APPCODE		"e616ce2fcae54f718e2ae3121af871ad"
unsigned char esp8266_buf[512];
unsigned short esp8266_cnt = 0, esp8266_cntPre = 0;


//==========================================================
//	函数名称：	ESP8266_Clear
//
//	函数功能：	清空缓存
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void ESP8266_Clear(void)
{
 
	memset(esp8266_buf, 0, sizeof(esp8266_buf));
	esp8266_cnt = 0;

}

//==========================================================
//	函数名称：	ESP8266_WaitRecive
//
//	函数功能：	等待接收完成
//
//	入口参数：	无
//
//	返回参数：	REV_OK-接收完成		REV_WAIT-接收超时未完成
//
//	说明：		循环调用检测是否接收完成
//==========================================================
int ESP8266_WaitRecive(void)
{

	if(esp8266_cnt == 0) 							//如果接收计数为0 则说明没有处于接收数据中，所以直接跳出，结束函数
		return REV_WAIT;
		
	if(esp8266_cnt == esp8266_cntPre)				//如果上一次的值和这次相同，则说明接收完毕
	{
		esp8266_cnt = 0;							//清0接收计数
			
		return REV_OK;								//返回接收完成标志
	}
		
	esp8266_cntPre = esp8266_cnt;					//置为相同
	
	return REV_WAIT;								//返回接收未完成标志

}

//==========================================================
//	函数名称：	ESP8266_SendCmd
//
//	函数功能：	发送命令
//
//	入口参数：	cmd：命令
//				res：需要检查的返回指令
//
//	返回参数：	0-成功	1-失败
//
//	说明：		
//==========================================================
_Bool ESP8266_SendCmd(char *cmd, char *res)
{
	
	unsigned char timeOut = 200;

	Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
	
	while(timeOut--)
	{
		if(ESP8266_WaitRecive() == REV_OK)							//如果收到数据
		{
			if(strstr((const char *)esp8266_buf, res) != NULL)		//如果检索到关键词
			{
				ESP8266_Clear();									//清空缓存
				
				return 0;
			}
		}
		
		//delay_ms(10);
        vTaskDelay(10);
	}
	
	return 1;

}

//==========================================================
//	函数名称：	ESP8266_SendData
//
//	函数功能：	发送数据
//
//	入口参数：	data：数据
//				len：长度
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void ESP8266_SendData(unsigned char *data, unsigned short len)
{

	char cmdBuf[512];
	
	ESP8266_Clear();								//清空接收缓存
	sprintf(cmdBuf, "AT+CIPSEND=%d\r\n", len);		//发送命令
	if(!ESP8266_SendCmd(cmdBuf, ">"))				//收到‘>’时可以发送数据
	{
		Usart_SendString(USART2, data, len);		//发送设备连接请求数据
	}

}

//==========================================================
//	函数名称：	ESP8266_GetIPD
//
//	函数功能：	获取平台返回的数据
//
//	入口参数：	等待的时间(乘以10ms)
//
//	返回参数：	平台返回的原始数据
//
//	说明：		不同网络设备返回的格式不同，需要去调试
//				如ESP8266的返回格式为	"+IPD,x:yyy"	x代表数据长度，yyy是数据内容
//==========================================================
unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{

	char *ptrIPD = NULL;
	
	do
	{
		if(ESP8266_WaitRecive() == REV_OK)								//如果接收完成
		{
			ptrIPD = strstr((char *)esp8266_buf, "IPD,");				//搜索“IPD”头
			if(ptrIPD == NULL)											//如果没找到，可能是IPD头的延迟，还是需要等待一会，但不会超过设定的时间
			{
				UsartPrintf(USART_DEBUG, "\"IPD\" not found\r\n");
			}
			else
			{
				ptrIPD = strchr(ptrIPD, ':');							//找到':'
				if(ptrIPD != NULL)
				{
					ptrIPD++;
					return (unsigned char *)(ptrIPD);
				}
				else
					return NULL;
				
			}
		}
		
		//delay_ms(5);													//延时等待
        vTaskDelay(5);
	} while(timeOut--);
	
	return NULL;														//超时还未找到，返回空指针

}

//==========================================================
//	函数名称：	ESP8266_Init
//
//	函数功能：	初始化ESP8266
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void ESP8266_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//ESP8266复位引脚
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;              // PA0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;          // 输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;       // 无上拉下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // 速度50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
	//delay_ms(250);
    vTaskDelay(250);
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
	//delay_ms(500);
    vTaskDelay(500);
	
	ESP8266_Clear();
	
	UsartPrintf(USART_DEBUG, "1. AT\r\n");//基本测试指令，检查ESP8266是否正常响应
	while(ESP8266_SendCmd("AT\r\n", "OK"))
		//delay_ms(500);
        vTaskDelay(500);
	
	UsartPrintf(USART_DEBUG, "2. CWMODE\r\n");
	while(ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK"))//配置ESP8266为客户端模式，可以连接Wi-Fi热点
		//delay_ms(500);
        vTaskDelay(500);
	
	UsartPrintf(USART_DEBUG, "3. AT+CWDHCP\r\n");
	while(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK"))//允许ESP8266自动从路由器获取IP地址
		//delay_ms(500);
        vTaskDelay(500);
	
	UsartPrintf(USART_DEBUG, "4. CWJAP\r\n");
	while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP"))
		//delay_ms(500);
        vTaskDelay(500);
	
	UsartPrintf(USART_DEBUG, "5. ESP8266 Init OK\r\n");
	
}

//==========================================================
//	函数名称：	USART3_IRQHandler
//
//	函数功能：	串口2收发中断
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void USART2_IRQHandler(void)
{
    uint8_t ch;

    // 处理接收中断
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(USART2);  // 读取数据，清除RXNE标志
        if(esp8266_cnt < sizeof(esp8266_buf))
        {
            esp8266_buf[esp8266_cnt++] = ch;
        }
        else
        {
            // 缓冲区满，可以选择丢弃或者从头开始覆盖
            esp8266_cnt = 0;
            esp8266_buf[esp8266_cnt++] = ch;
        }
        // 注意：这里不需要再清除RXNE中断标志，因为读取数据寄存器已经清除。
        // 但是为了确保，可以调用：USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }

    // 处理错误中断，防止错误标志导致中断一直触发
    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) == SET ||
       USART_GetFlagStatus(USART2, USART_FLAG_NE) == SET ||
       USART_GetFlagStatus(USART2, USART_FLAG_FE) == SET ||
       USART_GetFlagStatus(USART2, USART_FLAG_PE) == SET)
    {
        // 清除错误标志
        // 对于ORE，需要先读SR，再读DR
        volatile uint32_t tmp = USART2->SR;
        tmp = USART2->DR;
        (void)tmp;
    }
}

//==========================================================
// 函数名称： ESP8266_MQTTPublish
// 功能：     发布MQTT消息（原始数据）
// 参数：     topic   - 发布的主题
//            payload - 消息内容（字符串）
//            qos     - QoS等级（0或1）
// 返回：     0-成功   1-失败
// 说明：     使用RAW方式发送，适合长数据
//==========================================================
_Bool ESP8266_MQTTPublish(char *topic, char *payload, unsigned char qos)
{
    ESP8266_Clear();
    char cmd[512];
    unsigned short len = strlen(payload);
    
    // 发送AT+MQTTPUBRAW指令
    sprintf(cmd, "AT+MQTTPUBRAW=0,\"%s\",%d,0,%d\r\n", topic, len, qos);
    Usart_SendString(USART2, (unsigned char*)cmd, strlen(cmd));
    
    // 等待 ">" 提示符（使用自定义等待函数）
    //unsigned short timeout = 200; // 2秒
    unsigned short timeout = 200;
    while(timeout--)
    {
        if(ESP8266_WaitRecive() == REV_OK)
        {
            if(strstr((const char*)esp8266_buf, ">") != NULL)
            {
                ESP8266_Clear();
                break;
            }
        }
       // delay_ms(10);
    vTaskDelay(10);
    }
    if(timeout == 0) return 1; // 超时失败
    
    // 发送实际数据
    Usart_SendString(USART2, (unsigned char*)payload, len);
    
    // 等待 "OK" 应答
    //timeout = 500;
    timeout = 500;
    while(timeout--)
    {
        if(ESP8266_WaitRecive() == REV_OK)
        {
            if(strstr((const char*)esp8266_buf, "OK") != NULL)
            {
                ESP8266_Clear();
                return 0;
            }
        }
        //delay_ms(10);
       vTaskDelay(10); 
    }
    return 1;
}

