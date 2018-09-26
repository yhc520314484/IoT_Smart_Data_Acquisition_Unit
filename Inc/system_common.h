#ifndef __SYSTEM_COMMON_H__
#define __SYSTEM_COMMON_H__

#include "main.h"
#include "stm32l4xx_hal.h"

extern com_mod_phy_connected communication_module_physical_connected;
extern sen_mod_phy_connected sensor_module_physical_connected;
extern sys_task_flag system_task_flag;
extern uint8_t unique_ID_STM32L4[12]; 
/**
  * @brief  ERROR_CODE 错误代码表
  */
typedef enum {
	ERROR_CODE_GPIO_LED_Init_FAIL,                         //LED相关GPIO管脚初始化失败
	ERROR_CODE_DEBUG_USART1_Init_FAIL,                     //调试串口1初始化失败
	ERROR_CODE_IWDG_TIM3_Init_FAIL,                        //用于独立看门狗喂狗的TIM3初始化失败
	ERROR_CODE_IWDG_Init_FAIL,                             //独立看门狗初始化失败
	ERROR_CODE_EEPROM_24C256_Init_FAIL,                    //EEPROM 24C256初始化失败
	ERROR_CODE_GPIO_DIP_Switch_Init_FAIL,                  //拨码开关相关GPIO管脚初始化失败
	ERROR_CODE_GPIO_DIP_Switch_Com_Mod_Read_FAIL,          //读取通信模块选择相关的GPIO口失败
	ERROR_CODE_GPIO_LoRa_SX1278_Extra_Init_FAIL,           //LoRa SX1278模块除SPI通信必要管脚外所需的其他GPIO管脚初始化失败
	ERROR_CODE_LoRa_SX1278_SPI_Init_FAIL,                  //LoRa SX1278模块SPI2初始化失败
	ERROR_CODE_LoRa_SX1278_Radio_Driver_Init_FAIL,         //LoRa SX1278模块频率及驱动初始化失败
	ERROR_CODE_GPIO_WiFi_ESP8266_Extra_Init_FAIL,          //WiFi ESP8266模块除USART3通信必要管脚外所需的其他GPIO管脚初始化失败
	ERROR_CODE_WiFi_ESP8266_USART3_Init_FAIL,              //WiFi ESP8266模块USART3外设初始化失败
	ERROR_CODE_WiFi_ESP8266_Driver_Init_FAIL,              //WiFi ESP8266模块驱动初始化失败
	ERROR_CODE_BlueTooth_nRF52832_USART3_Init_FAIL,        //BlueTooth nRF52832模块USART3外设初始化失败
	ERROR_CODE_BlueTooth_nRF52832_Driver_Init_FAIL,        //BlueTooth nRF52832模块驱动初始化失败
	ERROR_CODE_NB_IoT_BC26_USART3_Init_FAIL,               //NB-IoT BC26模块USART3外设初始化失败
	ERROR_CODE_NB_IoT_BC26_Driver_Init_FAIL,               //NB-IoT BC26模块驱动初始化失败
	
	ERROR_CODE_GPIO_DIP_Switch_Sen_Mod_Read_FAIL,          //读取传感器模块选择相关的GPIO口失败
	ERROR_CODE_Electric_Meter_USART2_Init_FAIL,            //SP3485模块所使用的USART2是初始化失败
	ERROR_CODE_Electric_Meter_Baudrate_Auto_Get_FAIL,      //电表传感器自动波特率获取失败
	ERROR_CODE_Other_System_Initialization_Process_FAIL,   //其他系统初始化过程失败
	
	ERROR_CODE_Get_STM32L4_Chip_ID_FAIL,                   //获取STM32内部的96位产品唯一身份标识失败
	
	ERROR_CODE_Read_Connection_History_Byte_From_EEPROM_FAIL,  //读取AP端是否曾成功与路由器建立过连接的EEPROM标志位失败
	
	ERROR_CODE_EEPROM_24C256_REGISTRATION_RECOVRY_READ_PROCEDURE_FAIL,  //非首次注册时从EEPROM读历史注册信息失败
	
	ERROR_CODE_UART1_RX_BUFFER_OVERFLOW,                   //串口1接收缓存溢出
	ERROR_CODE_UART2_RX_BUFFER_OVERFLOW,                   //串口2接收缓存溢出
	ERROR_CODE_UART3_RX_BUFFER_OVERFLOW,                   //串口3接收缓存溢出
	
	ERROR_CODE_EEPROM_Read_Bytes_FAIL,                      //EEPROM读取字节数据失败
	ERROR_CODE_EEPROM_Write_Bytes_FAIL,                     //EEPROM写入字节数据失败
}ErrorCodeDefinition;

uint8_t error_code_handle(uint8_t error_code);                     //错误码处理函数

void init_instruction_resend_procedure(uint8_t resend_count);   //初始化指令重发复位流程
uint8_t clear_alarm_and_count(uint8_t count_reg_exist, 
	  uint8_t * alarm_flag, uint8_t alarm_flag_addr, uint8_t alarm_flag_size, 
		uint8_t *  alarm_count, uint8_t alarm_count_addr, uint8_t alarm_count_size);    //清空告警寄存器中的告警位及其相关的计数器

#endif
