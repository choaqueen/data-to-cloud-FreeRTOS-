#ifndef _ESP8266_H_
#define _ESP8266_H_

#define MQTT_PUBLISH_TOPIC     "$oc/devices/69d31f3de094d615922eeaf2_dev1_0_0_2026040605/sys/properties/report"

#define REV_OK		0	//接收完成标志
#define REV_WAIT	1	//接收未完成标志


void ESP8266_Init(void);

void ESP8266_Clear(void);

_Bool ESP8266_SendCmd(char *cmd, char *res);

void ESP8266_SendData(unsigned char *data, unsigned short len);

unsigned char *ESP8266_GetIPD(unsigned short timeOut);

_Bool ESP8266_MQTTPublish(char *topic, char *payload, unsigned char qos);
//注：将bool类型改为int型
int ESP8266_WaitRecive(void);


#endif
