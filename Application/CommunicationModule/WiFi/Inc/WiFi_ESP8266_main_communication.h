#ifndef __WIFI_ESP8266_MAIN_COMMUNICATION_H__
#define __WIFI_ESP8266_MAIN_COMMUNICATION_H__

#include "main.h"
#include "stm32l4xx_hal.h"
#include "ESP8266_driver.h"

/********************************** 用户需要设置的参数**********************************/
//#define      macUser_ESP8266_ApSsid                       "ESP8266TEST"         //要连接的热点的名称
//#define      macUser_ESP8266_ApPwd                        "12345678"           //要连接的热点的密钥

//#define      macUser_ESP8266_UdpServer_IP                 "192.168.4.1"      //要连接的服务器的 IP
//#define      macUser_ESP8266_UdpServer_Port               "9090"               //要连接的服务器的端口
//#define      macUser_ESP8266_UdpLocal_Port               "8080"               //本机端口

#define      macUser_ESP8266_ApSsid                       "Sky"                //要连接的热点的名称
#define      macUser_ESP8266_ApPwd                        "01452108"           //要连接的热点的密钥

#define      macUser_ESP8266_UdpServer_IP                 "192.168.1.106"      //要连接的服务器的 IP
#define      macUser_ESP8266_UdpServer_Port               "9090"               //要连接的服务器的端口

#define      macUser_ESP8266_UdpLocal_Port               "8080"               //本机端口


uint8_t WiFi_ESP8266_Driver_Init(void);

#endif

