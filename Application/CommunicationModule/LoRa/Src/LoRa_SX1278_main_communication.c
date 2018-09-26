#include "LoRa_SX1278_main_communication.h"
#include "EEPROM_24C256.h"
#include "system_common.h"

tRadioDriver *Radio = NULL;                            //LoRa驱动指针定义

/**
  * @name         LoRa_SX1278_Radio_Driver_Init
  * @brief        LoRa SX1278模块频率及驱动初始化
  * @param        void
  * @retval       LoRa SX1278模块频率及驱动是否初始化成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  15:27
	* @author       JackWilliam
  */
uint8_t LoRa_SX1278_Radio_Driver_Init(void){
	Radio = RadioDriverInit( );           //LoRa驱动初始化
	Radio->Init( );                        //LoRa频率初始化
	
	return 0;
}
