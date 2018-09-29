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

/**
  * @name         message_send_to_router_by_lora_sx1278
  * @brief        通过LoRa SX1278 发送协议数据消息至路由器
  * @param        void
  * @retval       通过LoRa SX1278 协议数据消息是否发送成功，成功则返回0，失败则返回1
	* @lastModify   2018/9/29  19:14
	* @author       JackWilliam
  */
uint8_t message_send_to_router_by_lora_sx1278(uint8_t * message_entity, uint8_t message_entity_length){

	return 0;
}
