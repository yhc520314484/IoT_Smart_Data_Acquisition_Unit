#include "system_common.h"
#include "EEPROM_24C256.h"
#include "gpio.h"

#define RESEND_COUNT_LIMITS    10
#define RESET_COUNT_LIMITS     10

/**
  * @brief  STM32L4芯片产品唯一身份标识 96位
  */
uint8_t unique_ID_STM32L4[12]; 

/**
  * @brief  传感器参数存储寄存器动态分配内存地址相关指针
  */
Linked_List *memory_data_sensor_point_start_address;
	
	
///**
//  * @brief  内存中预置的通信密钥
//  */
//uint8_t init_communication_key[16] = {0xE6, 0XD8, 0x25, 0x6A, 0x02, 0xE8, 0x46, 0x36,
//																			0x9F, 0x21, 0x93, 0xA1, 0x30, 0x2D, 0xBE, 0x73}; 	

///**	
//  * @brief  内存中预置的恢复出厂设置密钥
//  */
//uint8_t init_wipe_key[16] = {0x52, 0XDE, 0x86, 0x6C, 0x7D, 0x15, 0xE8, 0x33,
//														 0xF7, 0xF0, 0xA8, 0xA3, 0x19, 0xC5, 0xE1, 0x55}; 	


/**
  * @brief  系统所连接的通信模块定义，该结构体中的值由通信模块选择相关的拨码开关决定
  */
com_mod_phy_connected communication_module_physical_connected = {
	.lora_sx1278_connected                  = 0,                 //lora sx1278模块已连接至AP端
	.wifi_esp8266_connected                 = 0,                 //wifi esp8266模块已连接至AP端
	.bluetooth_nrf52832_connected           = 0,                 //bluetooth nrf52832模块已连接至AP端
	.nb_iot_bc26_connected                  = 0                  //NB-IoT BC26模块已连接至AP端
};

/**
  * @brief  系统所连接的通信模块定义，该结构体中的值由通信模块选择相关的拨码开关决定
  */
sen_mod_phy_connected sensor_module_physical_connected = {
	.electric_meter_connected                  = 0,              //电表模块已连接至AP端
	.water_meter_connected                     = 0,              //水表模块已连接至AP端
	.gas_meter_connected                       = 0               //气表模块已连接至AP端
};

/**
  * @brief  系统任务的工作状态标志位
  */
sys_task_flag system_task_flag = {
	.task_sensor_data_collect    =   0,                 //数据采集任务
	.task_sensor_data_update     =   0,                 //数据上传任务
	.task_control_command_get    =   0,                 //控制命令接收任务
	.task_sleep                  =   0                  //休眠任务（停止模式2）	
};


/**
  * @name         init_instruction_resend_procedure
  * @brief        初始化指令重发复位流程
  * @param        resend_count:已重发次数
  * @retval       void
	* @lastModify   2018/9/25  19:57
	* @author       JackWilliam
  */
void init_instruction_resend_procedure(uint8_t resend_count){
	if(resend_count <= RESEND_COUNT_LIMITS) return;
	else{
		if(ee_ReadBytes(&register_data_warning_store.InitResetCount, register_address_warning_store.InitResetCount, 1))
			error_code_handle(ERROR_CODE_EEPROM_Read_Bytes_FAIL);
		
		if(register_data_warning_store.InitResetCount <= RESET_COUNT_LIMITS){
			register_data_warning_store.InitResetCount++;
			register_data_warning_store.InitAlarm = 1;
			if(ee_WriteBytes(&register_data_warning_store.InitResetCount, register_address_warning_store.InitResetCount, 1))
				error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
			if(ee_WriteBytes(&register_data_warning_store.InitAlarm, register_address_warning_store.InitAlarm, 1))
				error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
			NVIC_SystemReset();
		}
		
		else{
			HAL_GPIO_WritePin(GPIOA, state_initialization_failed_LED_Pin, GPIO_PIN_SET);     //点亮初始化失败指示灯，该指示灯为共阴指示灯
			while(1);        //挂死程序，点亮初始化失败指示灯等待技术人员进行维护
		}
	}
}




/**
  * @name         clear_alarm_and_count
  * @brief        清空告警寄存器中的告警位及其相关的计数器
  * @param        count_reg_exist: 该位是否带有配套相关的计数器，若有则此参数为1，反之为0
  * @param        *alarm_flag:     指向寄存器告警位变量在内存中保存地址的指针
  * @param        alarm_flag_addr: 寄存器告警位在EEPROM中的保存地址
  * @param        alarm_flag_size: 寄存器告警位的长度
  * @param        *alarm_count:     指向寄存器告警位相关的计数器变量在内存中保存地址的指针
  * @param        alarm_count_addr: 寄存器告警位相关的计数器在EEPROM中的保存地址
  * @param        alarm_count_size: 寄存器告警位相关的计数器的长度
  * @retval       清空成功则返回0   失败则返回1
	* @lastModify   2018/9/25  19:57
	* @author       JackWilliam
  */
uint8_t clear_alarm_and_count(uint8_t count_reg_exist, 
	  uint8_t * alarm_flag, uint8_t alarm_flag_addr, uint8_t alarm_flag_size, 
		uint8_t * alarm_count, uint8_t alarm_count_addr, uint8_t alarm_count_size)
{
	if(ee_ReadBytes(alarm_flag, alarm_flag_addr, alarm_flag_size))
			error_code_handle(ERROR_CODE_EEPROM_Read_Bytes_FAIL);
	
	if(*alarm_flag == 1){
		*alarm_flag = 0;
		if(ee_WriteBytes(alarm_flag, alarm_flag_addr, alarm_flag_size))
			error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
		
		if(count_reg_exist == 1){
			*alarm_count = 0;
			if(ee_WriteBytes(alarm_count, alarm_count_addr, alarm_flag_size))
					error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
		}
	}
	return 0;
}

///**
//  * @name         clear_init_alarm_and_count
//  * @brief        清空告警寄存器中的告警位及其相关的计数器
//  * @param        resend_count:已重发次数
//  * @retval       void
//	* @lastModify   2018/9/25  19:57
//	* @author       JackWilliam
//  */
//void clear_init_alarm_and_count(void){
//	ee_ReadBytes(&register_data_warning_store.InitAlarm, register_address_warning_store.InitAlarm, 1);
//	if(register_data_warning_store.InitAlarm == 1){
//		register_data_warning_store.InitAlarm = 0;
//		register_data_warning_store.InitResetCount = 0;
//		ee_WriteBytes(&register_data_warning_store.InitResetCount, register_address_warning_store.InitResetCount, 1);
//		ee_WriteBytes(&register_data_warning_store.InitAlarm, register_address_warning_store.InitAlarm, 1);
//	}
//}


/**
  * @name         error_code_handle
  * @brief        错误编码处理程序
  * @param        void
  * @retval       错误编码Error Code
	* @lastModify   2018/9/25  14:14
	* @author       JackWilliam
  */
uint8_t error_code_handle(uint8_t error_code){
	switch(error_code){
		//与路由器连接状态指示灯LED初始化失败
		case ERROR_CODE_GPIO_LED_Init_FAIL:
			printf(" Error Code: %d \r\n GPIO LED init fail\r\n",  ERROR_CODE_GPIO_LED_Init_FAIL);      
			break;
		
		//调试串口1初始化失败
		case ERROR_CODE_DEBUG_USART1_Init_FAIL:
			printf(" Error Code: %d \r\n DEBUG USART1 init fail\r\n",  ERROR_CODE_DEBUG_USART1_Init_FAIL);    
			break; 
		
		//用于独立看门狗喂狗的TIM3初始化失败
		case ERROR_CODE_IWDG_TIM3_Init_FAIL:
			printf(" Error Code: %d \r\n IWDG TIM3 init fail\r\n", ERROR_CODE_IWDG_TIM3_Init_FAIL);      
			break;
		
		//独立看门狗初始化失败
		case ERROR_CODE_IWDG_Init_FAIL:
			printf(" Error Code: %d \r\n IWDG init\r\n", ERROR_CODE_IWDG_Init_FAIL);
			break;
		
		//EEPROM 24C256初始化失败
		case ERROR_CODE_EEPROM_24C256_Init_FAIL:
			printf(" Error Code: %d \r\n EEPROM 24C256 init\r\n", ERROR_CODE_EEPROM_24C256_Init_FAIL);
			break;
		
		//用于拨码开关相关GPIO管脚初始化
		case ERROR_CODE_GPIO_DIP_Switch_Init_FAIL:
			printf(" Error Code: %d \r\n GPIO DIP Switch init\r\n", ERROR_CODE_GPIO_DIP_Switch_Init_FAIL);
			break;
		
		//用于读取通信模块选择相关的GPIO口失败
		case ERROR_CODE_GPIO_DIP_Switch_Com_Mod_Read_FAIL:
			printf(" Error Code: %d \r\n GPIO DIP Switch com mod read fail\r\n", ERROR_CODE_GPIO_DIP_Switch_Init_FAIL);
			break;
		
		//LoRa SX1278模块除SPI通信必要管脚外所需的其他GPIO管脚初始化失败
		case ERROR_CODE_GPIO_LoRa_SX1278_Extra_Init_FAIL:
			printf(" Error Code: %d \r\n GPIO LoRa SX1278 Extra init fail\r\n", ERROR_CODE_GPIO_LoRa_SX1278_Extra_Init_FAIL);
			break;
		
		//LoRa SX1278模块SPI初始化
		case ERROR_CODE_LoRa_SX1278_SPI_Init_FAIL:
			printf(" Error Code: %d \r\n LoRa SX1278 SPI init fail\r\n", ERROR_CODE_LoRa_SX1278_SPI_Init_FAIL);
			break;
		
		//LoRa SX1278模块频率及驱动初始化  
		case ERROR_CODE_LoRa_SX1278_Radio_Driver_Init_FAIL:
			printf(" Error Code: %d \r\n LoRa SX1278 Radio Driver init fail\r\n", ERROR_CODE_LoRa_SX1278_Radio_Driver_Init_FAIL);
			break;
		
		//LoRa SX1278模块频率及驱动初始化  
		case ERROR_CODE_GPIO_WiFi_ESP8266_Extra_Init_FAIL:
			printf(" Error Code: %d \r\n GPIO WiFi ESP8266 Extra init fail\r\n", ERROR_CODE_GPIO_WiFi_ESP8266_Extra_Init_FAIL);
			break;

		//WiFi ESP8266模块USART3外设初始化失败
		case ERROR_CODE_WiFi_ESP8266_USART3_Init_FAIL:
			printf(" Error Code: %d \r\n WiFi ESP8266 Usart3 init fail\r\n", ERROR_CODE_WiFi_ESP8266_USART3_Init_FAIL);
			break;
		
		//WiFi ESP8266模块驱动初始化失败
		case ERROR_CODE_WiFi_ESP8266_Driver_Init_FAIL:
			printf(" Error Code: %d \r\n GPIO WiFi ESP8266 Driver init fail\r\n", ERROR_CODE_WiFi_ESP8266_Driver_Init_FAIL);
			break;
		
		//BlueTooth nRF52832模块USART3外设初始化失败
		case ERROR_CODE_BlueTooth_nRF52832_USART3_Init_FAIL:
			printf(" Error Code: %d \r\n BlueTooth nRF52832 USART3 init fail\r\n", ERROR_CODE_BlueTooth_nRF52832_USART3_Init_FAIL);
			break;
		
	  //BlueTooth nRF52832模块驱动初始化失败
		case ERROR_CODE_BlueTooth_nRF52832_Driver_Init_FAIL:
			printf(" Error Code: %d \r\n BlueTooth nRF52832 Driver init fail\r\n", ERROR_CODE_BlueTooth_nRF52832_Driver_Init_FAIL);
			break;
		
		//NB-IoT BC26模块USART3外设初始化失败
		case ERROR_CODE_NB_IoT_BC26_USART3_Init_FAIL:
			printf(" Error Code: %d \r\n NB-IoT BC26 USART3 init fail\r\n", ERROR_CODE_NB_IoT_BC26_USART3_Init_FAIL);
			break;
		
		//NB-IoT BC26模块USART3外设初始化失败
		case ERROR_CODE_NB_IoT_BC26_Driver_Init_FAIL:
			printf(" Error Code: %d \r\n NB-IoT BC26 Driver init fail\r\n", ERROR_CODE_NB_IoT_BC26_Driver_Init_FAIL);
			break;
		
		
		
		
		//读取传感器模块选择相关的GPIO口失败
		case ERROR_CODE_GPIO_DIP_Switch_Sen_Mod_Read_FAIL:
			printf(" Error Code: %d \r\n GPIO DIP Switch Sen Mod Read init fail\r\n", ERROR_CODE_GPIO_DIP_Switch_Sen_Mod_Read_FAIL);
			break;
		
		
		//电表相关的GPIO口初始化失败
		case ERROR_CODE_GPIO_Electric_Meter_Extra_Init_FAIL:
			printf(" Error Code: %d \r\n GPIO electric meter extra init fail\r\n", ERROR_CODE_GPIO_Electric_Meter_Extra_Init_FAIL);
			break;
		
		//SP3485模块所使用的USART2是初始化失败
		case ERROR_CODE_Electric_Meter_USART2_Init_FAIL:
			printf(" Error Code: %d \r\n electric meter USART2 init fail\r\n", ERROR_CODE_Electric_Meter_USART2_Init_FAIL);
			break;
		
		//电表传感器自动波特率获取失败
		case ERROR_CODE_Electric_Meter_Baudrate_Auto_Get_FAIL:
			printf(" Error Code: %d \r\n electric meter baudrate auto get fail\r\n", ERROR_CODE_Electric_Meter_Baudrate_Auto_Get_FAIL);
			break;
		
		
		
		
		//其他系统初始化过程失败
		case ERROR_CODE_Other_System_Initialization_Process_FAIL:
			printf(" Error Code: %d \r\n other system initialization process fail\r\n", ERROR_CODE_Other_System_Initialization_Process_FAIL);
			break;
		
		
		
		
		//获取STM32内部的96位产品唯一身份标识失败
		case ERROR_CODE_Get_STM32L4_Chip_ID_FAIL:
			printf(" Error Code: %d \r\n get STM32L4 chip ID fail\r\n", ERROR_CODE_Get_STM32L4_Chip_ID_FAIL);
			break;
		
		//读取AP端是否曾成功与路由器建立过连接的EEPROM标志位失败
		case ERROR_CODE_Read_Connection_History_Byte_From_EEPROM_FAIL:
			printf(" Error Code: %d \r\n read connection history byte from EEPROM fail\r\n", ERROR_CODE_Read_Connection_History_Byte_From_EEPROM_FAIL);
			break;
		
		//非首次注册时从EEPROM读历史注册信息失败
		case ERROR_CODE_EEPROM_24C256_REGISTRATION_RECOVRY_READ_PROCEDURE_FAIL:
			printf(" Error Code: %d \r\n EEPROM 24c256 registration recovry read procedure fail\r\n", ERROR_CODE_EEPROM_24C256_REGISTRATION_RECOVRY_READ_PROCEDURE_FAIL);
			break;	 
		
		//RTC时钟初始化失败
		case ERROR_CODE_RTC_Init_Set_FAIL:
			printf(" Error Code: %d \r\n RTC init set fail\r\n", ERROR_CODE_RTC_Init_Set_FAIL);
			break;

		//间隔时间和窗口时间解析失败
		case ERROR_CODE_Data_UpdateTime_Settings_Set_FAIL:
			printf(" Error Code: %d \r\n data updateTime settings set fail\r\n", ERROR_CODE_Data_UpdateTime_Settings_Set_FAIL);
			break;			
		
		
		
		
		
		//收到的数据包长度小于最小协议包长度
		case ERROR_CODE_Protocol_Main_Package_Decode_Header_Length_Check_FAIL:
			printf(" Error Code: %d \r\n protocol main package decode header length check fail\r\n", ERROR_CODE_Protocol_Main_Package_Decode_Header_Length_Check_FAIL);
			break;	 
		
		//收到的数据包的帧头不正确
		case ERROR_CODE_Protocol_Main_Package_Decode_Header_Top_Check_FAIL:
			printf(" Error Code: %d \r\n protocol main package decode header top check fail\r\n", ERROR_CODE_Protocol_Main_Package_Decode_Header_Top_Check_FAIL);
			break;	

		//收到的数据包的版本号不正确
		case ERROR_CODE_Protocol_Main_Package_Decode_Header_Version_Check_FAIL:
			printf(" Error Code: %d \r\n protocol main package decode header version check fail\r\n", ERROR_CODE_Protocol_Main_Package_Decode_Header_Version_Check_FAIL);
			break;	 		
		
		//收到的数据包的序号不正确
		case ERROR_CODE_Protocol_Main_Package_Decode_Header_Serial_ID_Check_FAIL:
			printf(" Error Code: %d \r\n protocol main package decode header serial ID check fail\r\n", ERROR_CODE_Protocol_Main_Package_Decode_Header_Serial_ID_Check_FAIL);
			break;
		
		//已注册状态下收到的数据包源地址错误
		case ERROR_CODE_Protocol_Main_Package_Decode_Header_Master_ID_Check_FAIL:
			printf(" Error Code: %d \r\n protocol main package decode header master ID check fail\r\n", ERROR_CODE_Protocol_Main_Package_Decode_Header_Master_ID_Check_FAIL);
			break;
		
		//已注册状态下收到的数据包目的地址错误	
		case ERROR_CODE_Protocol_Main_Package_Decode_Header_Slave_ID_Check_FAIL:
			printf(" Error Code: %d \r\n protocol main package decode header slave ID check fail\r\n", ERROR_CODE_Protocol_Main_Package_Decode_Header_Slave_ID_Check_FAIL);
			break;
		
		//路由器注册响应解码失败	
		case ERROR_CODE_Protocol_Device_Registere_Responce_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol device registere responce part decode fail\r\n", ERROR_CODE_Protocol_Device_Registere_Responce_Part_Decode_FAIL);
			break;
		
		//路由器传感器参数确认/返回失败	
		case ERROR_CODE_Protocol_Sensor_Parameter_Responce_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol sensor parameter responce part decode fail\r\n", ERROR_CODE_Protocol_Sensor_Parameter_Responce_Part_Decode_FAIL);
			break;
		
		//路由器时间同步确认失败
		case ERROR_CODE_Protocol_Time_Sync_Responce_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol time sync responce part decode fail\r\n", ERROR_CODE_Protocol_Time_Sync_Responce_Part_Decode_FAIL);
			break;
		
		//路由器数据上传确认失败
		case ERROR_CODE_Protocol_Data_Entity_Ack_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol data entity ack decode fail\r\n", ERROR_CODE_Protocol_Data_Entity_Ack_Decode_FAIL);
			break;

		//路由器告警信息确认失败
		case ERROR_CODE_Protocol_System_Warning_Ack_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol system warning ack decode fail\r\n", ERROR_CODE_Protocol_System_Warning_Ack_Part_Decode_FAIL);
			break;
		
		//路由器周期更改请求失败
		case ERROR_CODE_Protocol_Period_Change_Request_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol period change request part decode fail\r\n", ERROR_CODE_Protocol_Period_Change_Request_Part_Decode_FAIL);
			break;
		
		//路由器传感器参数变更请求失败
		case ERROR_CODE_Protocol_Sensor_Parameter_Change_Request_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol sensor parameter change request part decode fail\r\n", ERROR_CODE_Protocol_Sensor_Parameter_Change_Request_Part_Decode_FAIL);
			break;
		
		//路由器更新通信密钥请求失败
		case ERROR_CODE_Protocol_Communication_Key_Update_Request_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol communication key update request part decode fail\r\n", ERROR_CODE_Protocol_Communication_Key_Update_Request_Part_Decode_FAIL);
			break;
	
		//路由器退网换区命令失败
		case ERROR_CODE_Protocol_Network_Switching_Request_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol network switching request part decode fail\r\n", ERROR_CODE_Protocol_Network_Switching_Request_Part_Decode_FAIL);
			break;

		//路由器重启命令失败
		case ERROR_CODE_Protocol_Reboot_Request_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol reboot request part decode fail\r\n", ERROR_CODE_Protocol_Reboot_Request_Part_Decode_FAIL);
			break;

		//路由器获取AP设备状态失败
		case ERROR_CODE_Protocol_Device_State_Request_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol device state request part decode fail\r\n", ERROR_CODE_Protocol_Device_State_Request_Part_Decode_FAIL);
			break;
		
	//路由器节能模式更改命令失败
		case ERROR_CODE_Protocol_Energy_Saving_Mode_Request_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol energy saving mode request part decode fail\r\n", ERROR_CODE_Protocol_Energy_Saving_Mode_Request_Part_Decode_FAIL);
			break;
		
	//路由器恢复出厂设置命令失败
		case ERROR_CODE_Protocol_Factory_Setting_Reset_Request_Part_Decode_FAIL:
			printf(" Error Code: %d \r\n protocol factory setting reset request part decode fail\r\n", ERROR_CODE_Protocol_Factory_Setting_Reset_Request_Part_Decode_FAIL);
			break;
		

		
		
		
		
		
		
		
		
		//串口1接收溢出
		case ERROR_CODE_UART1_RX_BUFFER_OVERFLOW:
			printf(" Error Code: %d \r\n Uart1 Receive Buffer OverFlow\r\n", ERROR_CODE_UART1_RX_BUFFER_OVERFLOW); 
			break;
		
		//串口2接收溢出
		case ERROR_CODE_UART2_RX_BUFFER_OVERFLOW:
			printf(" Error Code: %d \r\n Uart2 Receive Buffer OverFlow\r\n", ERROR_CODE_UART2_RX_BUFFER_OVERFLOW);
			break;
		
		//串口3接收溢出
		case ERROR_CODE_UART3_RX_BUFFER_OVERFLOW:
			printf(" Error Code: %d \r\n Uart3 Receive Buffer OverFlow\r\n", ERROR_CODE_UART3_RX_BUFFER_OVERFLOW);  
			break;
		
		//EEPROM读取传感器相关的寄存器失败
		case ERROR_CODE_EEPROM_24C256_SENSOR_PARAMETER_READ_FAIL:
			printf(" Error Code: %d \r\n EEPROM 24c256 sensor parameter read fail\r\n", ERROR_CODE_EEPROM_24C256_SENSOR_PARAMETER_READ_FAIL);  
			break; 
		
		//EEPROM读取字节数据失败
		case ERROR_CODE_EEPROM_Read_Bytes_FAIL:
			printf(" Error Code: %d \r\n EEPROM Read Bytes fail\r\n", ERROR_CODE_UART3_RX_BUFFER_OVERFLOW);  
			break;
		
		//EEPROM写入字节数据失败                      
		case ERROR_CODE_EEPROM_Write_Bytes_FAIL:  
			printf(" Error Code: %d \r\n EEPROM Write Bytes fail\r\n", ERROR_CODE_UART3_RX_BUFFER_OVERFLOW);  
			break;
	}
	/*********************测试用        正式生产环境中删除**************/
	//点亮初始化失败灯
	HAL_GPIO_WritePin(GPIOA, state_initialization_failed_LED_Pin, GPIO_PIN_SET);
	while(1);
	/*********************测试用        正式生产环境中删除**************/
	
	return error_code;
}
