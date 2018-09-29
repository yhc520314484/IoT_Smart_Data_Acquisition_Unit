#include "NB_IoT_BC26_main_communication.h"
#include "EEPROM_24C256.h"
#include "system_common.h"
#include "usart.h"

/**
  * @name         NB_IoT_BC26_Driver_Init
  * @brief        BlueTooth nRF52832模块驱动初始化
  * @param        void
  * @retval       BlueTooth nRF52832模块驱动是否初始化成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  17:40
	* @author       JackWilliam
  */
uint8_t NB_IoT_BC26_Driver_Init(void){

	return 0;
}


/**
  * @name         message_send_to_router_by_lnb_iot_bc26
  * @brief        通过nb_iot bc26 发送协议数据消息至路由器
  * @param        void
  * @retval       通过nb_iot bc26 协议数据消息是否发送成功，成功则返回0，失败则返回1
	* @lastModify   2018/9/29  19:14
	* @author       JackWilliam
  */
uint8_t message_send_to_router_by_nb_iot_bc26(uint8_t * message_entity, uint8_t message_entity_length){

	return 0;
}