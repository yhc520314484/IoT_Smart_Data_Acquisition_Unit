/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h> 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define unused_button_Pin GPIO_PIN_13
#define unused_button_GPIO_Port GPIOC
#define communication_spi2_MISO_Pin GPIO_PIN_2
#define communication_spi2_MISO_GPIO_Port GPIOC
#define communication_spi2_MOSI_Pin GPIO_PIN_3
#define communication_spi2_MOSI_GPIO_Port GPIOC
#define sensor_electric_meter_rs485_EN_Pin GPIO_PIN_0
#define sensor_electric_meter_rs485_EN_GPIO_Port GPIOA
#define module_select_button_COM_LoRa_Pin GPIO_PIN_1
#define module_select_button_COM_LoRa_GPIO_Port GPIOA
#define sensor_usart2_TX_Pin GPIO_PIN_2
#define sensor_usart2_TX_GPIO_Port GPIOA
#define sensor_usart2_RX_Pin GPIO_PIN_3
#define sensor_usart2_RX_GPIO_Port GPIOA
#define module_select_button_COM_WiFi_Pin GPIO_PIN_4
#define module_select_button_COM_WiFi_GPIO_Port GPIOA
#define state_connected_LED_Pin GPIO_PIN_5
#define state_connected_LED_GPIO_Port GPIOA
#define module_select_button_COM_BlueTooth_Pin GPIO_PIN_6
#define module_select_button_COM_BlueTooth_GPIO_Port GPIOA
#define module_select_button_COM_NB_IoT_Pin GPIO_PIN_7
#define module_select_button_COM_NB_IoT_GPIO_Port GPIOA
#define communication_usart3_TX_Pin GPIO_PIN_4
#define communication_usart3_TX_GPIO_Port GPIOC
#define communication_usart3_RX_Pin GPIO_PIN_5
#define communication_usart3_RX_GPIO_Port GPIOC
#define module_select_button_SEN_ElectricMeter_Pin GPIO_PIN_0
#define module_select_button_SEN_ElectricMeter_GPIO_Port GPIOB
#define module_select_button_SEN_WaterMeter_Pin GPIO_PIN_1
#define module_select_button_SEN_WaterMeter_GPIO_Port GPIOB
#define module_select_button_SEN_GasMeter_Pin GPIO_PIN_2
#define module_select_button_SEN_GasMeter_GPIO_Port GPIOB
#define communication_spi2_SCK_Pin GPIO_PIN_10
#define communication_spi2_SCK_GPIO_Port GPIOB
#define communication_wifi_EN_Pin GPIO_PIN_14
#define communication_wifi_EN_GPIO_Port GPIOB
#define communication_wifi_RST_Pin GPIO_PIN_15
#define communication_wifi_RST_GPIO_Port GPIOB
#define DEBUG_usart1_TX_Pin GPIO_PIN_9
#define DEBUG_usart1_TX_GPIO_Port GPIOA
#define DEBUG_usart1_RX_Pin GPIO_PIN_10
#define DEBUG_usart1_RX_GPIO_Port GPIOA
#define state_initialization_failed_LED_Pin GPIO_PIN_12
#define state_initialization_failed_LED_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define communication_lora_DIO0_Pin GPIO_PIN_4
#define communication_lora_DIO0_GPIO_Port GPIOB
#define communication_lora_DIO2_Pin GPIO_PIN_5
#define communication_lora_DIO2_GPIO_Port GPIOB
#define communication_lora_NSS_Pin GPIO_PIN_7
#define communication_lora_NSS_GPIO_Port GPIOB
#define eeprom_SCL_Pin GPIO_PIN_8
#define eeprom_SCL_GPIO_Port GPIOB
#define eeprom_SDA_Pin GPIO_PIN_9
#define eeprom_SDA_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/**
  * @brief  系统所连接的通信模块定义，该结构体中的值由通信模块选择相关的拨码开关决定
  */
typedef struct{
	uint8_t lora_sx1278_connected;
	uint8_t wifi_esp8266_connected;
	uint8_t bluetooth_nrf52832_connected;
	uint8_t nb_iot_bc26_connected;
}com_mod_phy_connected;


/**
  * @brief  系统所连接的通信模块定义，该结构体中的值由通信模块选择相关的拨码开关决定
  */
typedef struct{
	uint8_t electric_meter_connected;
	uint8_t water_meter_connected;
	uint8_t gas_meter_connected;
}sen_mod_phy_connected;




/**
  * @brief  系统所处状态定义 其中UP代表数据包已成功由AP端发送上行至路由器  DOWN表示路由器下发至AP端的数据包已被成功接收
  */
typedef enum {
	SYSTEM_STATE_UNREGISTERED,                             //系统尚未在路由器上进行注册
	SYSTEM_STATE_REGISTERED,                               //系统已在路由器上成功进行注册
	SYSTEM_STATE_SENSOR_PARAMETER_REQUEST_SEND_UP,         //传感器参数请求已发送  等待路由器回应 
	SYSTEM_STATE_SENSOR_PARAMETER_GET_DOWN,                //系统已得到传感器参数
	SYSTEM_STATE_TIME_SYNC_REQUEST_SEND_UP,                //时间同步请求已发送  等待路由器回应 
	SYSTEM_STATE_TIME_SYNC_ACK_GET_DOWN,                   //系统已与路由器完成时间同步
	SYSTEM_STATE_DATA_ENTITY_SEND_UP,                      //数据已发送  等待路由器回应
	SYSTEM_STATE_DATA_ENTITY_ACK_GET_DOWN,                 //路由器端回应已收到
	SYSTEM_STATE_CONTROL_COMMAND_GET_DOWN,                 //收到路由器端控制数据包
	SYSTEM_STATE_CONTROL_COMMAND_ACK_SEND_UP,              //已回应路由器收到控制数据包并做出相应的响应
	SYSTEM_STATE_SENDING_PROCESS_TIMEOUT_UP,               //数据包上载超时
	SYSTEM_STATE_DOWN_ERROR                                //收到错误类型的路由器回应
}SystemStateDefinition;


/**
  * @brief  系统任务的工作状态标志位
  */
typedef struct{
	uint8_t task_sensor_data_collect;                 //数据采集任务
	uint8_t task_sensor_data_update;                  //数据上传任务
	uint8_t task_control_command_get;                 //控制命令接收任务
	uint8_t task_sleep;                               //休眠任务（停止模式2）	
}sys_task_flag;
	


/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
