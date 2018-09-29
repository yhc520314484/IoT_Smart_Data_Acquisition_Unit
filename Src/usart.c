/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "system_common.h"
#include "ESP8266_driver.h"

uint8_t  uart1_one_byte_Rx_buffer;			  //接收中断缓冲
uint8_t  uart1_entire_Rx_buffer[1024];		//接收缓冲
uint16_t uart1_entire_Rx_count = 0;		  //接收缓冲计数

uint8_t  uart2_one_byte_Rx_buffer;			  //接收中断缓冲
uint8_t  uart2_entire_Rx_buffer[1024];		//接收缓冲
uint16_t uart2_entire_Rx_count = 0;		  //接收缓冲计数

uint8_t  uart3_one_byte_Rx_buffer;			  //接收中断缓冲
uint8_t  uart3_entire_Rx_buffer[1024];		//接收缓冲
uint16_t uart3_entire_Rx_count = 0;		  //接收缓冲计数


/*************BlueTooth部分程序段****************/
extern uint8_t BlueTooth_nRF52832_message[300];
extern uint8_t BlueTooth_nRF52832_message_len;
extern uint8_t BlueTooth_nRF52832_get_flag;
/*************BlueTooth部分程序段****************/


//extern uint8_t WiFi_ESP8266_get_data_flag;       //WiFi_ESP8266接收到数据

extern com_mod_phy_connected communication_module_physical_connected;
extern sen_mod_phy_connected sensor_module_physical_connected;


/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2400;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = DEBUG_usart1_TX_Pin|DEBUG_usart1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = sensor_usart2_TX_Pin|sensor_usart2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PC4     ------> USART3_TX
    PC5     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = communication_usart3_TX_Pin|communication_usart3_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, DEBUG_usart1_TX_Pin|DEBUG_usart1_RX_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, sensor_usart2_TX_Pin|sensor_usart2_RX_Pin);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PC4     ------> USART3_TX
    PC5     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOC, communication_usart3_TX_Pin|communication_usart3_RX_Pin);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/**
  * @name         DEBUG_USART1_Init
  * @brief        调试串口USART1初始化
  * @param        void
  * @retval       调试串口USART1是否初始化成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  10:21
	* @author       JackWilliam
  */
uint8_t DEBUG_USART1_Init(void){
	MX_USART1_UART_Init();
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart1_one_byte_Rx_buffer, 1);
	
	return 0;
}


/**
  * @name         WiFi_NB_BT_USART3_Init
  * @brief        WIFI、蓝牙、NB模块所使用的USART3初始化
  * @param        void
  * @retval       WIFI、蓝牙、NB模块所使用的USART3是否初始化成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  10:21
	* @author       JackWilliam
  */
uint8_t WiFi_NB_BT_USART3_Init(void){
	MX_USART3_UART_Init();
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart3_one_byte_Rx_buffer, 1);
	
	return 0;
}


/**
  * @name         Electric_Meter_USART2_Init
  * @brief        电表模块所使用的RS485接口通过SP3485模块连接至USART2的USART2初始化 波特率2400B/每秒  字长9位 偶检验 1停止位
  * @param        void
  * @retval       SP3485模块所使用的USART2是否初始化成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/26  9:07
	* @author       JackWilliam
  */
uint8_t Electric_Meter_USART2_Init(void){
	MX_USART2_UART_Init();
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_one_byte_Rx_buffer, 1);

	return 0;
}




/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	uint16_t i; 
	
	//调试用   串口1   USART1
	if(huart == &huart1){
			if(uart1_entire_Rx_count >= 1024)  //溢出判断
			{
				uart1_entire_Rx_count = 0;
				memset(uart1_entire_Rx_buffer,0x00,sizeof(uart1_entire_Rx_buffer));
				error_code_handle(ERROR_CODE_UART1_RX_BUFFER_OVERFLOW);
			}
			else
			{
				uart1_entire_Rx_buffer[uart1_entire_Rx_count++] = uart1_one_byte_Rx_buffer;   //接收数据转存
			
				if((uart1_entire_Rx_buffer[uart1_entire_Rx_count-1] == 0x0A)&&(uart1_entire_Rx_buffer[uart1_entire_Rx_count-2] == 0x0D)) //判断结束位
				{
					HAL_UART_Transmit(&huart1, (uint8_t *)&uart1_entire_Rx_buffer, uart1_entire_Rx_count,0xFFFF); //将收到的信息发送出去
					uart1_entire_Rx_count = 0;
					memset(uart1_entire_Rx_buffer,0x00,sizeof(uart1_entire_Rx_buffer)); //清空数组
				}
			}
		
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart1_one_byte_Rx_buffer, 1);   //再开启接收中断
	}	
	
	//传感器数据接入 串口2   USART2
	if(huart == &huart2){
			if(uart2_entire_Rx_count >= 1024)  //溢出判断
			{
				uart2_entire_Rx_count = 0;
				memset(uart2_entire_Rx_buffer,0x00,sizeof(uart2_entire_Rx_buffer));
				error_code_handle(ERROR_CODE_UART2_RX_BUFFER_OVERFLOW);
			}
			else
			{
				uart2_entire_Rx_buffer[uart2_entire_Rx_count++] = uart2_one_byte_Rx_buffer;   //接收数据转存
			
				if((uart2_entire_Rx_buffer[uart2_entire_Rx_count-1] == 0x0A)&&(uart2_entire_Rx_buffer[uart2_entire_Rx_count-2] == 0x0D)) //判断结束位
				{
					HAL_UART_Transmit(&huart2, (uint8_t *)&uart2_entire_Rx_buffer, uart2_entire_Rx_count,0xFFFF); //将收到的信息发送出去
					uart2_entire_Rx_count = 0;
					memset(uart2_entire_Rx_buffer,0x00,sizeof(uart2_entire_Rx_buffer)); //清空数组
				}
			}
		
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_one_byte_Rx_buffer, 1);   //再开启接收中断
	}	
	
	//通信模块接入 串口3   USART3
	if(huart == &huart3){
			if(uart3_entire_Rx_count >= 1024)  //溢出判断
			{
				uart3_entire_Rx_count = 0;
				memset(uart3_entire_Rx_buffer,0x00,sizeof(uart3_entire_Rx_buffer));
				error_code_handle(ERROR_CODE_UART3_RX_BUFFER_OVERFLOW);
			}
			else
			{
				uart3_entire_Rx_buffer[uart3_entire_Rx_count++] = uart3_one_byte_Rx_buffer;   //接收数据转存
				
				/*************Wifi部分程序段****************/
				if(communication_module_physical_connected.wifi_esp8266_connected == 1){
					if( strEsp8266_Fram_Record .InfBit .FramLength < ( RX_BUF_MAX_LEN - 1 ) )                       //预留1个字节写结束符
						strEsp8266_Fram_Record .Data_RX_BUF [ strEsp8266_Fram_Record .InfBit .FramLength ++ ]  = uart3_one_byte_Rx_buffer;
				}
				/*************Wifi部分程序段****************/
				
				if((uart3_entire_Rx_buffer[uart3_entire_Rx_count-1] == 0x0A)&&(uart3_entire_Rx_buffer[uart3_entire_Rx_count-2] == 0x0D)){ //判断结束位
					/*************Wifi部分程序段****************/
					if(communication_module_physical_connected.wifi_esp8266_connected == 1){
//						WiFi_ESP8266_get_data_flag = 1;
						strEsp8266_Fram_Record .InfBit .FramFinishFlag = 1;
					}
					/*************Wifi部分程序段****************/
					
					
					
					/*************BlueTooth部分程序段****************/
					if(communication_module_physical_connected.bluetooth_nrf52832_connected == 1){	
						for(i = 0; i < uart3_entire_Rx_count; i++) BlueTooth_nRF52832_message[i] = uart3_entire_Rx_buffer[i];
						BlueTooth_nRF52832_message_len = uart3_entire_Rx_count;
						BlueTooth_nRF52832_get_flag = 1;
					}
					/*************BlueTooth部分程序段****************/
						
						

					
					//HAL_UART_Transmit(&huart3, (uint8_t *)&uart3_entire_Rx_buffer, uart3_entire_Rx_count,0xFFFF); //将收到的信息发送出去
					uart3_entire_Rx_count = 0;
					memset(uart3_entire_Rx_buffer,0x00,sizeof(uart3_entire_Rx_buffer)); //清空数组
				}
			}
		
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart3_one_byte_Rx_buffer, 1);   //再开启接收中断
	}	
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
