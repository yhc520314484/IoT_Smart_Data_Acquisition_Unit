#ifndef __SYSTEM_COMMON_H__
#define __SYSTEM_COMMON_H__

#include "main.h"
#include "stm32l4xx_hal.h"
#include "linked_list.h"

extern com_mod_phy_connected communication_module_physical_connected;
extern sen_mod_phy_connected sensor_module_physical_connected;
extern sys_task_flag system_task_flag;
extern uint8_t unique_ID_STM32L4[12]; 
extern Linked_List *memory_data_sensor_point_start_address;
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
	ERROR_CODE_GPIO_Electric_Meter_Extra_Init_FAIL,        //电表相关的GPIO口初始化失败
	ERROR_CODE_Electric_Meter_USART2_Init_FAIL,            //SP3485模块所使用的USART2是初始化失败
	ERROR_CODE_Electric_Meter_Baudrate_Auto_Get_FAIL,      //电表传感器自动波特率获取失败
	ERROR_CODE_Other_System_Initialization_Process_FAIL,   //其他系统初始化过程失败
	
	ERROR_CODE_Get_STM32L4_Chip_ID_FAIL,                   //获取STM32内部的96位产品唯一身份标识失败
	
	ERROR_CODE_Read_Connection_History_Byte_From_EEPROM_FAIL,  //读取AP端是否曾成功与路由器建立过连接的EEPROM标志位失败
	
	ERROR_CODE_EEPROM_24C256_REGISTRATION_RECOVRY_READ_PROCEDURE_FAIL,  //非首次注册时从EEPROM读历史注册信息失败
	
	ERROR_CODE_RTC_Init_Set_FAIL,                              //RTC时钟初始化失败
	ERROR_CODE_Data_UpdateTime_Settings_Set_FAIL,              //间隔时间和窗口时间解析失败
	ERROR_CODE_Period_Change_FAIL,                             //系统时间周期更改失败
	ERROR_CODE_Sensor_Parameter_Change_FAIL,                   //传感器参数变更请求解析失败
	ERROR_CODE_Wipe_Key_Check_FAIL,                            //恢复初始化设置的密钥校验失败
	
	
	
	ERROR_CODE_Protocol_Main_Package_Decode_Header_Length_Check_FAIL,           //收到的数据包长度小于最小协议包长度
	ERROR_CODE_Protocol_Main_Package_Decode_Header_Top_Check_FAIL,              //收到的数据包的帧头不正确
	ERROR_CODE_Protocol_Main_Package_Decode_Header_Version_Check_FAIL,          //收到的数据包的版本号不正确
	ERROR_CODE_Protocol_Main_Package_Decode_Header_Serial_ID_Check_FAIL,        //收到的数据包的序号不正确
	ERROR_CODE_Protocol_Main_Package_Decode_Header_Master_ID_Check_FAIL,         //已注册状态下收到的数据包源地址错误
	ERROR_CODE_Protocol_Main_Package_Decode_Header_Slave_ID_Check_FAIL,           //已注册状态下收到的数据包目的地址错误	
	
	ERROR_CODE_Protocol_Device_Registere_Responce_Part_Decode_FAIL,              //路由器注册响应解码失败
	ERROR_CODE_Protocol_Sensor_Parameter_Responce_Part_Decode_FAIL,              //路由器传感器参数确认/返回失败
	ERROR_CODE_Protocol_Time_Sync_Responce_Part_Decode_FAIL,                     //路由器时间同步确认失败
	ERROR_CODE_Protocol_Data_Entity_Ack_Decode_FAIL,                             //路由器数据上传确认失败
	ERROR_CODE_Protocol_System_Warning_Ack_Part_Decode_FAIL,                     //路由器告警信息确认失败
	ERROR_CODE_Protocol_Period_Change_Request_Part_Decode_FAIL,                  //路由器周期更改请求失败
	ERROR_CODE_Protocol_Sensor_Parameter_Change_Request_Part_Decode_FAIL,        //路由器传感器参数变更请求失败
	ERROR_CODE_Protocol_Communication_Key_Update_Request_Part_Decode_FAIL,       //路由器更新通信密钥请求失败
	ERROR_CODE_Protocol_Network_Switching_Request_Part_Decode_FAIL,              //路由器退网换区命令失败
	ERROR_CODE_Protocol_Reboot_Request_Part_Decode_FAIL,                         //路由器重启命令失败
	ERROR_CODE_Protocol_Device_State_Request_Part_Decode_FAIL,                   //路由器获取AP设备状态失败
	ERROR_CODE_Protocol_Energy_Saving_Mode_Request_Part_Decode_FAIL,             //路由器节能模式更改命令失败
	ERROR_CODE_Protocol_Factory_Setting_Reset_Request_Part_Decode_FAIL,          //路由器恢复出厂设置命令失败
	
	

	ERROR_CODE_UART1_RX_BUFFER_OVERFLOW,                   //串口1接收缓存溢出
	ERROR_CODE_UART2_RX_BUFFER_OVERFLOW,                   //串口2接收缓存溢出
	ERROR_CODE_UART3_RX_BUFFER_OVERFLOW,                   //串口3接收缓存溢出
	
	ERROR_CODE_EEPROM_24C256_SENSOR_PARAMETER_READ_FAIL,    //EEPROM读取传感器相关的寄存器失败
	ERROR_CODE_EEPROM_Read_Bytes_FAIL,                      //EEPROM读取字节数据失败
	ERROR_CODE_EEPROM_Write_Bytes_FAIL,                     //EEPROM写入字节数据失败
}ErrorCodeDefinition;

uint8_t error_code_handle(uint8_t error_code);                     //错误码处理函数

void init_instruction_resend_procedure(uint8_t resend_count);   //初始化指令重发复位流程
uint8_t clear_alarm_and_count(uint8_t count_reg_exist, 
	  uint8_t * alarm_flag, uint8_t alarm_flag_addr, uint8_t alarm_flag_size, 
		uint8_t *  alarm_count, uint8_t alarm_count_addr, uint8_t alarm_count_size);    //清空告警寄存器中的告警位及其相关的计数器
uint8_t message_send_to_router(uint8_t * message_entity, uint8_t message_entity_length);

#endif
