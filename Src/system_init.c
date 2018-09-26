#include "system_init.h"
#include "iwdg.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "EEPROM_24C256.h"
#include "system_common.h"
#include "LoRa_SX1278_main_communication.h"
#include "BlueTooth_nRF52832_main_communication.h"
#include "NB_IoT_BC26_main_communication.h"
#include "LoRa_SX1278_main_communication.h"
#include "WiFi_ESP8266_main_communication.h"

#include "sensor_electric_meter.h"


/********************************************************************************************************/
/*                                                                                                      */
/*                                 AP端硬件初始化                                                       */
/*                                   程序段开始                                                         */
/*                                                                                                      */
/*                                                                                                      */
/********************************************************************************************************/

/**
  * @name         AP_side_hardware_init
  * @brief        AP端相关硬件初始化 包括片上外设、通信模块及传感器模块
  * @param        void
  * @retval       void
	* @lastModify   2018/9/26  10:25
	* @author       JackWilliam
  */
void AP_side_hardware_init(void){
	necessary_on_chip_peripherals_init();              //第一阶段  初始化必要片上外设
	communication_module_init();                       //第二阶段  初始化通信模块
	sensor_module_init();                              //第三阶段  初始化传感器模块
  other_initialization_procedure();	                 //第四阶段  其他初始化过程
	printf("System Module Init Success!\r\n");         //输出
}


/**
  * @name         necessary_on_chip_peripherals_init
  * @brief        第一阶段  初始化必要片上外设
  * @param        void
  * @retval       void
	* @lastModify   2018/9/25  14:14
	* @author       JackWilliam
  */
void necessary_on_chip_peripherals_init(void){
	if(DEBUG_USART1_Init())                    //调试串口USART1初始化
		error_code_handle(ERROR_CODE_DEBUG_USART1_Init_FAIL);
	
	if(GPIO_LED_Init())                        //LED相关GPIO管脚初始化
		error_code_handle(ERROR_CODE_GPIO_LED_Init_FAIL);
	
	if(IWDG_TIM3_Init())                        //用于独立看门狗喂狗的1S定时器TIM3初始化
		error_code_handle(ERROR_CODE_IWDG_TIM3_Init_FAIL);
	
	if(IWDG_Init())                             //独立看门狗初始化
		error_code_handle(ERROR_CODE_IWDG_Init_FAIL);
	
	if(EEPROM_24C256_Init())                        //用于24C256 EEPROM 初始化
		error_code_handle(ERROR_CODE_EEPROM_24C256_Init_FAIL);
	
	if(GPIO_DIP_Switch_Init())                     //用于拨码开关相关GPIO管脚初始化
		error_code_handle(ERROR_CODE_GPIO_DIP_Switch_Init_FAIL);
}

/**
  * @name         communication_module_init
  * @brief        第二阶段  初始化通信模块
  * @param        void
  * @retval       void
	* @lastModify   2018/9/25  14:14
	* @author       JackWilliam
  */
void communication_module_init(void){
	if(GPIO_DIP_Switch_Com_Mod_Read(&communication_module_physical_connected))  //读取通信模块选择相关的GPIO口
		error_code_handle(ERROR_CODE_GPIO_DIP_Switch_Com_Mod_Read_FAIL);
	
	if(communication_module_physical_connected.lora_sx1278_connected == 1){
		
	/***************LoRa SX1278驱动初始化       START*******************/
		if(GPIO_LoRa_SX1278_Extra_Init())  //LoRa SX1278模块除SPI通信必要管脚外所需的其他GPIO管脚初始化
			error_code_handle(ERROR_CODE_GPIO_LoRa_SX1278_Extra_Init_FAIL);
		
		if(LoRa_SX1278_SPI_Init())  //LoRa SX1278模块SPI初始化
			error_code_handle(ERROR_CODE_LoRa_SX1278_SPI_Init_FAIL);
			
		if(LoRa_SX1278_Radio_Driver_Init())  //LoRa SX1278模块频率及驱动初始化
			error_code_handle(ERROR_CODE_LoRa_SX1278_Radio_Driver_Init_FAIL);
		
		register_data_basic_settings.ComModuleType = ComModuleType_LoRa_SX1278_Connected;
	}
	/***************LoRa SX1278驱动初始化       END*******************/
	
	
	/***************WiFi ESP8266驱动初始化       START*******************/
	if(communication_module_physical_connected.wifi_esp8266_connected == 1){
		if(GPIO_WiFi_ESP8266_Extra_Init())  //WiFi ESP8266模块除USART3通信必要管脚外所需的其他GPIO管脚初始化
			error_code_handle(ERROR_CODE_GPIO_WiFi_ESP8266_Extra_Init_FAIL);
		
		if(WiFi_ESP8266_USART3_Init())  //WiFi ESP8266模块USART3初始化
			error_code_handle(ERROR_CODE_WiFi_ESP8266_USART3_Init_FAIL);
			
		if(WiFi_ESP8266_Driver_Init())  //WiFi ESP8266模块驱动初始化
			error_code_handle(ERROR_CODE_WiFi_ESP8266_Driver_Init_FAIL);
		
		register_data_basic_settings.ComModuleType = ComModuleType_WiFi_ESP8266_Connected;
	}
	/***************WiFi ESP8266驱动初始化       END*******************/
	
	
	/***************BlueTooth nRF52832驱动初始化       START*******************/
	if(communication_module_physical_connected.bluetooth_nrf52832_connected == 1){	
		if(BlueTooth_nRF52832_USART3_Init())  //WiFi ESP8266模块USART3初始化
			error_code_handle(ERROR_CODE_BlueTooth_nRF52832_USART3_Init_FAIL);
			
		if(BlueTooth_nRF52832_Driver_Init())  //WiFi ESP8266模块驱动初始化
			error_code_handle(ERROR_CODE_BlueTooth_nRF52832_Driver_Init_FAIL);
		
		register_data_basic_settings.ComModuleType = ComModuleType_BlueTooth_nRF52832_Connected;
	}
	/***************BlueTooth nRF52832驱动初始化      END*******************/
	
	
	
	/***************NB-IoT BC26驱动初始化       START   !!!!!!!暂未开发！！*******************/
	if(communication_module_physical_connected.bluetooth_nrf52832_connected == 1){	
		if(NB_IoT_BC26_USART3_Init())  //NB-IoT BC26模块USART3初始化
			error_code_handle(ERROR_CODE_NB_IoT_BC26_USART3_Init_FAIL);
			
		//!!!!!!!暂未开发！！
		if(NB_IoT_BC26_Driver_Init())  //NB-IoT BC26模块驱动初始化     !!!!!!!暂未开发！！
			error_code_handle(ERROR_CODE_NB_IoT_BC26_Driver_Init_FAIL);
		
		register_data_basic_settings.ComModuleType = ComModuleType_NB_IoT_BC26_Connected;
	} 
	/***************NB-IoT BC26驱动初始化       END    !!!!!!!暂未开发！！*******************/	
	
	if(ee_WriteBytes(&register_data_basic_settings.ComModuleType, register_address_basic_settings.ComModuleType, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
}

/**
  * @name         sensor_module_init
  * @brief        第三阶段  初始化传感器模块
  * @param        void
  * @retval       void
	* @lastModify   2018/9/25  14:14
	* @author       JackWilliam
  */
void sensor_module_init(void){
	if(GPIO_DIP_Switch_Sen_Mod_Read(&sensor_module_physical_connected))  //读取通信模块选择相关的GPIO口
		error_code_handle(ERROR_CODE_GPIO_DIP_Switch_Sen_Mod_Read_FAIL);
	
		/**************电表模块初始化       START   DLT-645-2007-多功能电能表通信协议2010*******************/
	if(sensor_module_physical_connected.electric_meter_connected == 1){
		if(Electric_Meter_USART2_Init())  //电表模块USART2初始化
			error_code_handle(ERROR_CODE_Electric_Meter_USART2_Init_FAIL);
		
		if(Electric_Meter_Baudrate_Auto_Get(&sensor_electric_meter_general_parameters))  //电表模块波特率自动识别！！！！！暂未开发，现阶段固定返回2400的波特率
			error_code_handle(ERROR_CODE_Electric_Meter_Baudrate_Auto_Get_FAIL);
	}
	 /**************电表模块初始化       START   DLT-645-2007-多功能电能表通信协议2010*******************/
}

/**
  * @name         other_initialization_procedure
  * @brief        第四阶段  其他初始化过程
  * @param        void
  * @retval       void
	* @lastModify   2018/9/26  10:03
	* @author       JackWilliam
  */
void other_initialization_procedure(void){
/**	
  * @brief  内存中预置的恢复出厂设置密钥
  */
uint8_t init_wipe_key[16] = {0x52, 0XDE, 0x86, 0x6C, 0x7D, 0x15, 0xE8, 0x33,
														 0xF7, 0xF0, 0xA8, 0xA3, 0x19, 0xC5, 0xE1, 0x55};
	if(ee_WriteBytes(register_data_wipe_key.WipeKey, register_address_wipe_key.WipeKey, 16))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	
	if(other_system_initialization_process())
		error_code_handle(ERROR_CODE_Other_System_Initialization_Process_FAIL);

}	


/**
  * @name         other_system_initialization_process
  * @brief        其他系统初始化过程
  * @param        void
  * @retval       其他初始化过程是否顺利完成  成功则返回0  失败则返回1
	* @lastModify   2018/9/26  10:20
	* @author       JackWilliam
  */
uint8_t other_system_initialization_process(void){
	  HAL_TIM_Base_Start_IT(&htim2);                                //初始化中断时基
	__HAL_RCC_PWR_CLK_ENABLE();                                     //初始化功耗控制模块时钟
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);      //保证系统唤醒后使用HSI时钟
	
	return 0;
}

/********************************************************************************************************/
/******************************AP端硬件初始化   程序段结束***********************************************/
/********************************************************************************************************/


/********************************************************************************************************/
/*                                                                                                      */
/*                                 路由器连接初始化                                                     */
/*                                   程序段开始                                                         */
/*                                                                                                      */
/*                                                                                                      */
/********************************************************************************************************/

/**
  * @name         router_connection_initialization
  * @brief        AP端与路由器连接初始化
  * @param        void
  * @retval       void
	* @lastModify   2018/9/26  10:37
	* @author       JackWilliam
  */
void router_connection_initialization(void){
	if(get_STM32L4_chip_ID(unique_ID_STM32L4))
				error_code_handle(ERROR_CODE_Get_STM32L4_Chip_ID_FAIL);
	
	if(read_connection_history_byte_from_eeprom())
			error_code_handle(ERROR_CODE_Read_Connection_History_Byte_From_EEPROM_FAIL);
	
	if(register_data_basic_settings.DataExistFlag == 0x01) AP_side_registration_recovry();
	else AP_side_registration_for_first_time();
	
}

/**
  * @name         get_STM32L4_chip_ID
  * @brief        获取STM32内部的96位产品唯一身份标识
	* @param        *STM32_ID_point:指向保存STM32产品唯一识别码变量的指针
  * @retval       标识获取是否成功  成功返回0  失败返回1
	* @lastModify   2018/9/26  10:37
	* @author       JackWilliam
  */
uint8_t get_STM32L4_chip_ID(uint8_t * STM32_ID_point){
  	uint32_t temp0,temp1,temp2;
	
//STM32L4系列 ID码地址： 0x1FFF7590   0x1FFF7594  0x1FFF7598 ，只需要读取这个地址中的数据就可以了
		temp0 = *(__IO uint32_t*)(0x1FFF7590);    //产品唯一身份标识寄存器（96位）
    temp1 = *(__IO uint32_t*)(0x1FFF7594);
		temp2 = *(__IO uint32_t*)(0x1FFF7598);                       

    STM32_ID_point[0] = (uint8_t)(temp0 & 0x000000FF);
    STM32_ID_point[1] = (uint8_t)((temp0 & 0x0000FF00)>>8);
    STM32_ID_point[2] = (uint8_t)((temp0 & 0x00FF0000)>>16);
    STM32_ID_point[3] = (uint8_t)((temp0 & 0xFF000000)>>24);
    STM32_ID_point[4] = (uint8_t)(temp1 & 0x000000FF);
    STM32_ID_point[5] = (uint8_t)((temp1 & 0x0000FF00)>>8);
    STM32_ID_point[6] = (uint8_t)((temp1 & 0x00FF0000)>>16);
    STM32_ID_point[7] = (uint8_t)((temp1 & 0xFF000000)>>24);
    STM32_ID_point[8] = (uint8_t)(temp2 & 0x000000FF);
    STM32_ID_point[9] = (uint8_t)((temp2 & 0x0000FF00)>>8);
    STM32_ID_point[10] = (uint8_t)((temp2 & 0x00FF0000)>>16);
    STM32_ID_point[11] = (uint8_t)((temp2 & 0xFF000000)>>24);   

		return 0;
}
/**
  * @name         read_connection_history_byte_from_eeprom
  * @brief        读取AP端是否曾成功与路由器建立过连接的EEPROM标志位
	* @param        void
  * @retval       标志位读取是否成功  成功返回0  失败返回1
	* @lastModify   2018/9/26  10:37
	* @author       JackWilliam
  */
uint8_t read_connection_history_byte_from_eeprom(void){
	if(ee_ReadBytes(&register_data_basic_settings.DataExistFlag, register_address_basic_settings.DataExistFlag, 1)) 
		return 1;
	
	return 0;
}

/**
  * @name         AP_side_registration_recovry
  * @brief        AP端设备非首次注册过程，即注册恢复过程
	* @param        void
  * @retval       void
	* @lastModify   2018/9/26  10:37
	* @author       JackWilliam
  */
void AP_side_registration_recovry(void){
	if(EEPROM_24C256_registration_recovry_read_procedure())
		error_code_handle(ERROR_CODE_EEPROM_24C256_REGISTRATION_RECOVRY_READ_PROCEDURE_FAIL);
}

/**
  * @name         AP_side_registration_recovry
  * @brief        AP端设备首次注册过程 
	* @param        void
  * @retval       void
	* @lastModify   2018/9/26  10:37
	* @author       JackWilliam
  */
void AP_side_registration_for_first_time(void){
/**
  * @brief  内存中预置的通信密钥
  */
uint8_t init_communication_key[16] = {0xE6, 0XD8, 0x25, 0x6A, 0x02, 0xE8, 0x46, 0x36,
																			0x9F, 0x21, 0x93, 0xA1, 0x30, 0x2D, 0xBE, 0x73}; 	

																			
}


/**
  * @name         EEPROM_24C256_registration_recovry_read_procedure
  * @brief        非首次注册时从EEPROM读历史注册信息
	* @param        void
  * @retval       历史注册信息读取是否成功  成功返回0  失败返回1
	* @lastModify   2018/9/26  14:24
	* @author       JackWilliam
  */
uint8_t EEPROM_24C256_registration_recovry_read_procedure(void){
/*********从EEPROM中读取出各类基本信息并保存在内存结构体中************************/
	
	/***********************AP_REG_BASIC_SETTINGS******************************/	
	if(ee_ReadBytes(register_data_basic_settings.SlaveID, register_address_basic_settings.SlaveID, 2))
		error_code_handle(ERROR_CODE_EEPROM_Read_Bytes_FAIL);
	
	if(ee_ReadBytes(register_data_basic_settings.MasterID, register_address_basic_settings.MasterID, 2))
		error_code_handle(ERROR_CODE_EEPROM_Read_Bytes_FAIL);
	
	if(ee_ReadBytes(&register_data_basic_settings.SensorNum, register_address_basic_settings.SensorNum, 1))
		error_code_handle(ERROR_CODE_EEPROM_Read_Bytes_FAIL);
	
	if(ee_ReadBytes(&register_data_basic_settings.PowerSupply, register_address_basic_settings.PowerSupply, 1))
		error_code_handle(ERROR_CODE_EEPROM_Read_Bytes_FAIL);
	
	if(ee_ReadBytes(&register_data_basic_settings.SavePower, register_address_basic_settings.SavePower, 1))
		error_code_handle(ERROR_CODE_EEPROM_Read_Bytes_FAIL);
	
	return 0;
	
}
