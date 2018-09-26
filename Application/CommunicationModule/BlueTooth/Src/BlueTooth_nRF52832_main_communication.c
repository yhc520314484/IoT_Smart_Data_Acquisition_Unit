#include "BlueTooth_nRF52832_main_communication.h"
#include "EEPROM_24C256.h"
#include "system_common.h"
#include "usart.h"

uint8_t BlueTooth_nRF52832_message[300];
uint8_t BlueTooth_nRF52832_message_len;
uint8_t BlueTooth_nRF52832_get_flag = 0;

/**
  * @name         BlueTooth_nRF52832_Driver_Init
  * @brief        BlueTooth nRF52832模块驱动初始化
  * @param        void
  * @retval       BlueTooth nRF52832模块驱动是否初始化成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  17:40
	* @author       JackWilliam
  */
uint8_t BlueTooth_nRF52832_Driver_Init(void){
	uint8_t Rst_Command[] = "AT+RST\r\n";
	uint8_t Test_Command[] = "AT\r\n";
	uint8_t count = 0;
	
	HAL_Delay(200);        //等待nRF52832初始化完毕	
	
	HAL_UART_Transmit(&huart3, Rst_Command, sizeof(Rst_Command),0xFFFF); //将收到的信息发送出去
	HAL_Delay(1000);        //等待nRF52832初始化完毕
	
	while(count <= 10){
		BlueTooth_nRF52832_get_flag = 0;
		memset(BlueTooth_nRF52832_message, 0, sizeof(BlueTooth_nRF52832_message));
		HAL_UART_Transmit(&huart3, Test_Command, sizeof(Test_Command),0xFFFF); //将收到的信息发送出去
		HAL_Delay(100);
		
		if(BlueTooth_nRF52832_get_flag == 1 && (BlueTooth_nRF52832_message[0] == 'B' && 
				 BlueTooth_nRF52832_message[1] == 'L' && BlueTooth_nRF52832_message[2] == 'U' && 
	       BlueTooth_nRF52832_message[3] == 'E' && BlueTooth_nRF52832_message[4] == 'T' && 
	       BlueTooth_nRF52832_message[5] == 'O' && BlueTooth_nRF52832_message[6] == 'O' && 
	       BlueTooth_nRF52832_message[7] == 'T' && BlueTooth_nRF52832_message[8] == 'H' && 
	       BlueTooth_nRF52832_message[9] == 0x0D && BlueTooth_nRF52832_message[10] == 0x0A)){
					if(!clear_alarm_and_count(1, &register_data_warning_store.InitAlarm, register_address_warning_store.InitAlarm, 1,
						&register_data_warning_store.InitResetCount, register_address_warning_store.InitResetCount, 1)) break;
		}
		else{	
			init_instruction_resend_procedure(count);
		}
	}
	
	BlueTooth_nRF52832_message_len = 0;
	BlueTooth_nRF52832_get_flag = 0;
	memset(BlueTooth_nRF52832_message, 0, sizeof(BlueTooth_nRF52832_message));
	
	return 0;
}
