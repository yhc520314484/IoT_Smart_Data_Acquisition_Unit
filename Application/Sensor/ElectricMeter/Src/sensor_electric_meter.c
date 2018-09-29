#include "sensor_electric_meter.h"
#include "system_common.h"
#include "malloc.h"
#include "usart.h"


#define MAX_RS485_BUFFER_LEN               64

//extern reg_data_sensor_parm_and_ID    register_data_sensor_parm_and_ID;
uint8_t electric_meter_rs485_bus_send_buffer[MAX_RS485_BUFFER_LEN];	        //RS485发送缓存
uint8_t electric_meter_rs485_bus_send_buffer_len = 0;                       //RS485发送缓存长度
uint8_t rs485_cs=0;                                                         //RS485校验位

uint8_t electric_meter_rs485_bus_read_buffer[MAX_RS485_BUFFER_LEN];	        //RS485接收缓存
uint8_t electric_meter_rs485_bus_read_buffer_len = 0;                       //RS485接收缓存长度

sen_electric_meter_gen_para sensor_electric_meter_general_parameters = {
	.baudrate             =     2400,             //缺省默认波特率为2400
	.amount               =     1,                //缺省默认电表数量为1
	.ID_single_length     =     6,                //缺省默认单个电表ID长度为6
	.ID_total_length      =     6,                //缺省默认电表ID总长度为6
};


/**
  * @name         RS485_Send_Data
	* @brief        RS485总线发送len个字节的数据
	* @param        *buf:指向发送缓存区首地址的指针
	* @param        len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
  * @retval       RS485总线发送len个字节的数据是否成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/29  9:23
	* @author       JackWilliam
  */
uint8_t RS485_Send_Data(uint8_t *buf,uint8_t len){
	uint8_t t;
	uint8_t RS485_RX_CNT = 0;
	
	electric_meter_rs485_TX_EN;			//设置为发送模式
	memset(electric_meter_rs485_bus_read_buffer,0,MAX_RS485_BUFFER_LEN);//清空接收区
	RS485_RX_CNT=0;		//清零
  for(t=0;t<len;t++)		//循环发送数据
	{
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf[t], 1); //将收到的信息发送出去		
//		USART_SendData(USART2,buf[t]);
//    while( USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET );
		HAL_Delay(5);
	}	 	
//	HAL_UART_Transmit(&huart3, (uint8_t *)&buf, len,0xFFFF);
	RS485_RX_CNT=0;	  
	electric_meter_rs485_RX_EN;				//设置为接收模式	
	
	return 0;
}


/**
  * @name         RS485_Send_Data
	* @brief        RS485查询接收到的数据
	* @param        *buf:指向接收缓存区首地址的指针
	* @param        *len:指向接收缓存长度的指针
  * @retval       RS485总线数据接收是否成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/29  9:23
	* @author       JackWilliam
  */
uint8_t RS485_Receive_Data(uint8_t *buf,uint8_t *len){
	uint8_t RS485_RX_CNT = 0;
	uint8_t rxlen=RS485_RX_CNT;
	uint8_t i=0;
	
	electric_meter_rs485_RX_EN;				//设置为接收模式
	memset(buf,0,MAX_RS485_BUFFER_LEN);
	*len=0;				//默认为0
	HAL_Delay(200);		//等待200ms,连续超过200ms没有接收到一个数据,则认为接收结束
	if(rxlen==RS485_RX_CNT&&rxlen)//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=electric_meter_rs485_bus_read_buffer[i];	
		}		
		*len=RS485_RX_CNT;	//记录本次数据长度
		memset(electric_meter_rs485_bus_read_buffer,0,MAX_RS485_BUFFER_LEN);//清空接收区
		RS485_RX_CNT=0;		//清零
		return 1;
	}
	return 0;
}


/**
  * @name         Electric_Meter_Baudrate_Auto_Get（暂未开发!!!）
  * @brief        电表模块波特率自动获取（暂未开发!!!）
	* @param        *sen_para:指向电表相关的通用参数设置结构体的指针
  * @retval       电表模块波特率是否自动获取成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/26  9:58
	* @author       JackWilliam
  */
uint8_t Electric_Meter_Baudrate_Auto_Get(sen_electric_meter_gen_para * sen_para){
	sen_para->baudrate = 2400;
	
	
	sprintf((char *)register_data_sensor_parm.SensorComSpeed, "%d", sen_para->baudrate);
	
	if(ee_WriteBytes(register_data_sensor_parm.SensorComSpeed, register_address_sensor_parm.SensorComSpeed, 5))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	return 0;
}


/**
  * @name         electric_meter_address_auto_get
  * @brief        自动模式下电表地址获取
	* @param        *sensor_parm_and_ID:指向电表相关的通用参数设置与ID一体的结构体的指针
	* @param        retry_times:地址获取操作的重试次数
	* @retval       电表模块波特率是否自动获取成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/26  9:58
	* @author       JackWilliam
  */
uint8_t electric_meter_address_auto_get(uint8_t *sensor_ID, uint8_t retry_times)
{
	uint8_t i=0;
	uint8_t rx_done=0;//接收完成标志
	char* loc=0;
	uint8_t ID_LEN = 0;        //电表ID长度
	
	memset(electric_meter_rs485_bus_send_buffer,0,MAX_RS485_BUFFER_LEN);
	electric_meter_rs485_bus_send_buffer_len=0;
	
	for(i=0;i<4;i++)
	{
		electric_meter_rs485_bus_send_buffer[i]=0xFE;
		electric_meter_rs485_bus_send_buffer_len++;
	}			
	
	RS485_Send_Data(electric_meter_rs485_bus_send_buffer,4);//发送16个字节
	
	electric_meter_rs485_bus_send_buffer[4]=0x68;
	electric_meter_rs485_bus_send_buffer_len++;
	
	for(i=5;i<11;i++)
	{
		electric_meter_rs485_bus_send_buffer[i]=0xAA;
		electric_meter_rs485_bus_send_buffer_len++;
	}
	
	electric_meter_rs485_bus_send_buffer[11]=0x68;
	electric_meter_rs485_bus_send_buffer_len++;
	
	electric_meter_rs485_bus_send_buffer[12]=0x13;
	electric_meter_rs485_bus_send_buffer_len++;
	
	electric_meter_rs485_bus_send_buffer[13]=0x00;
	electric_meter_rs485_bus_send_buffer_len++;
	
	for(i=4;i<14;i++)
		rs485_cs=rs485_cs+electric_meter_rs485_bus_send_buffer[i];			
	electric_meter_rs485_bus_send_buffer[14]=rs485_cs;
	electric_meter_rs485_bus_send_buffer_len++;
	
	electric_meter_rs485_bus_send_buffer[15]=0x16;
	electric_meter_rs485_bus_send_buffer_len++;
	
	RS485_Send_Data(electric_meter_rs485_bus_send_buffer,electric_meter_rs485_bus_send_buffer_len);//发送16个字节
	
	HAL_Delay(200);
//	sensor_and_ID = mymalloc(sizeof(sensor_parm_and_ID->sensor_ID));
//	memset(sensor_ID,0,sizeof(sensor_ID));
	memset(sensor_ID,0,MAX_RS485_BUFFER_LEN);
	ID_LEN=0;
	while(retry_times)//接收数据
	{
		retry_times--;
		rx_done = RS485_Receive_Data(electric_meter_rs485_bus_read_buffer,&electric_meter_rs485_bus_read_buffer_len);
		if(rx_done)
		{
			loc=strchr( (const char *)electric_meter_rs485_bus_read_buffer,0x68);//查找首次出现0x68的位置
			if(loc!=NULL)
			{
				while(1)
				{
					loc++;
					if(*loc==0x68)
						break;
					else
						sensor_ID[ID_LEN++]=*loc; //取电表地址
				}
				memset(electric_meter_rs485_bus_read_buffer,0,MAX_RS485_BUFFER_LEN);
				electric_meter_rs485_bus_read_buffer_len=0;
				
				while(ID_LEN)
					electric_meter_rs485_bus_read_buffer[electric_meter_rs485_bus_read_buffer_len++]=sensor_ID[--ID_LEN];				
				memset(sensor_ID,0,MAX_RS485_BUFFER_LEN);
				
				for(ID_LEN=0;ID_LEN<electric_meter_rs485_bus_read_buffer_len;ID_LEN++)
					sensor_ID[ID_LEN]=electric_meter_rs485_bus_read_buffer[ID_LEN];
				memset(electric_meter_rs485_bus_read_buffer,0,MAX_RS485_BUFFER_LEN);
				electric_meter_rs485_bus_read_buffer_len=0;
				return 1;
			}
			return 0;
		}
	}
	return 0;
}





