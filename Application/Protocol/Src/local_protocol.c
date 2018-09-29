#include "local_protocol.h"
#include "system_common.h"
#include "linked_list.h"
#include "sensor_electric_meter.h"
#include "rtc.h"
#include "tim.h"
#include "encyption_aes.h"
#include "encyption_util.h"

#define   PROTOCOL_HEADER               0xFF          //协议数据包帧头
#define   PROTOCOL_VERSION              0x01          //协议数据包版本号
#define   PROTOCOL_MINIMUM_LENGTH       11            //协议数据包最小长度，即不带数据部分的长度
#define   SENSOR_ADDRESS_FINDING_TIMES  10            //传感器寻址重复次数
#define   MAX_ACK_MESSAGE_LEN           255           //发送至路由器端ACK包的最长长度

#define   ENCYPTION_ENABLE               1            //开启消息加密解密

extern SystemStateDefinition system_state_definition; 
extern reg_data_sensor_parm_and_ID    register_data_sensor_parm_and_ID;
extern Linked_List *memory_data_sensor_point_start_address;
extern RTC_HandleTypeDef hrtc;
extern tim_data_update_time_settings system_data_update_time_settings;
/********************************************************************************************************/
/*                            封包程序段        START                                                   */
/********************************************************************************************************/

/********************************************************************************************************/
/*                                                                                                      */
/*                      AP端上行数据至路由器段协议定义    AP端发起                                      */
/*                                   程序段开始                                                         */
/*                                                                                                      */
/*                                                                                                      */
/********************************************************************************************************/

/**
  * @name         protocol_main_package_encode
  * @brief        封装将要发给路由器的主协议数据包
  * @param        *msg_serial_ID:指向协议中所含有的用于校验数据包到达顺序的随机序号的指针
  * @param        *srcID: 指向发送数据包一方的设备ID的指针
  * @param        *destID: 指向接收数据包一方的设备ID的指针
  * @param        type: 协议数据包类型定义
  * @param        datalen: 协议数据包的数据部分所占长度
  * @param        *data: 指向协议数据包数据部分的指针
  * @param        *package_entity: 指向欲发送数据包所拼接转换完成的字符串的指针
  * @param        *package_len: 指向欲发送数据包所拼接转换完成的字符串的长度的指针
  * @retval       协议数据包是否成功制成  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_main_package_encode(uint8_t * msg_serial_ID, uint8_t *srcID, uint8_t *destID, 
				uint8_t type, uint8_t datalen, uint8_t *data, uint8_t *package_entity, uint8_t *package_len)
{
	ProtocolDefinition package;
	
	uint8_t i;
	uint8_t len;
	static uint8_t package_sequence_number = 0x00;
	
#if ENCYPTION_ENABLE                  //使能了加密算法后的封包过程
	uint8_t encyption_message_buf[MAX_ACK_MESSAGE_LEN];
	uint8_t j;
	AES_KEY aes_key;
	uint8_t cipher[MAX_ACK_MESSAGE_LEN];
#endif	
	
	
	//顺序ID
	package_sequence_number++;
	if(package_sequence_number == 0xFF) package_sequence_number = 0x00;
	if(package_sequence_number == 0x0D) package_sequence_number++;
	
	package.frame_header.message_header = PROTOCOL_HEADER;
	package.frame_header.protocol_version = PROTOCOL_VERSION;
	package.frame_header.message_serial_number = package_sequence_number;
	for(i = 0; i < 2; i++) package.frame_header.source_ID[i] = srcID[i];
	for(i = 0; i < 2; i++) package.frame_header.destination_ID[i] = destID[i];
	package.frame_header.message_type = type;
	package.frame_header.date_entity_length = datalen;
	for(i = 0; i < datalen; i++) package.date_entity[i] = data[i];
	
	
	package_entity[0] = package.frame_header.message_header;
	package_entity[1] = package.frame_header.protocol_version;
	package_entity[2] = package.frame_header.message_serial_number;
	for(i = 0; i < 2; i++) package_entity[3 + i] = package.frame_header.source_ID[i];
	for(i = 0; i < 2; i++) package_entity[5 + i] = package.frame_header.destination_ID[i];
	package_entity[7] = package.frame_header.message_type;
	package_entity[8] = package.frame_header.date_entity_length;
	for(i = 0; i < datalen; i++) package_entity[9 + i] = package.date_entity[i];
	
#if ENCYPTION_ENABLE
	if(datalen != 0){
		for(i = 9, j = 0; i < 9 + datalen; i++, j++)
			encyption_message_buf[j] = package_entity[i];
		AES_set_encrypt_key(register_data_communication_key.CommunicationKey, 128, &aes_key);
		AES_encrypt(encyption_message_buf, cipher, &aes_key);
		for(i = 9, j = 0; i < 9 + datalen; i++, j++)
			package_entity[i] = cipher[j];
	}
#endif
	

	package_entity[9 + datalen] = 0x0D;
	len = 9 + datalen + 1;
	
	/**********LoRa发送时  尾部自带一个0x0A   其他通信模块无自带帧尾********/
	if(communication_module_physical_connected.bluetooth_nrf52832_connected == 1 || 
		 communication_module_physical_connected.nb_iot_bc26_connected == 1 ||
	   communication_module_physical_connected.wifi_esp8266_connected == 1){
		package_entity[10 + datalen] = 0x0A;
		len++;	 
	}
//	len = 9 + datalen;

	*package_len = len;
	*msg_serial_ID = package_sequence_number;
	
	return 0;
}

/**
  * @name         protocol_device_registere_request_part_encode
  * @brief        封装将要发给路由器的注册请求主协议数据包中的数据字段
  * @param        AP_device_unique_ID_len:AP端主控的唯一识别码的长度
  * @param        *AP_device_unique_ID: 指向AP端主控的唯一识别码的指针
  * @param        communication_module_num: AP端所连接的不同的通信设备类型数
  * @param        *communication_module_type: 指向AP端所连接的不同的通信设备类型的指针
  * @param        sensor_module_num: AP端所连接的不同的传感器类型数
  * @param        *sensor_module_type: 指向AP端所连接的不同的传感器类型的指针
  * @param        *part_entity: 指向数据字段所拼接转换完成的字符串的指针
  * @param        *part_len: 指向数据字段所拼接转换完成的字符串的长度的指针
  * @retval       协议数据包是否成功制成  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  19:13
  */
uint8_t protocol_device_registere_request_part_encode(uint8_t AP_device_unique_ID_len,
			uint8_t * AP_device_unique_ID, uint8_t communication_module_num, uint8_t * communication_module_type, 
			uint8_t sensor_module_num, uint8_t * sensor_module_type, uint8_t *part_entity, uint8_t *part_len)
{	
	uint8_t i;
	uint8_t len;

	part_entity[0] = AP_device_unique_ID_len;
	for(i = 0; i < AP_device_unique_ID_len; i++) part_entity[i + 1] = AP_device_unique_ID[i];
	part_entity[AP_device_unique_ID_len + 1] = communication_module_num;
	for(i = 0; i < communication_module_num; i++) part_entity[AP_device_unique_ID_len + 2 + i] = communication_module_type[i];
	part_entity[AP_device_unique_ID_len + communication_module_num + 2] = sensor_module_num;
	for(i = 0; i < sensor_module_num; i++) part_entity[AP_device_unique_ID_len + communication_module_num + 3 + i] = sensor_module_type[i];
	
	len = AP_device_unique_ID_len + communication_module_num + sensor_module_num + 3;
	*part_len = len;

	return 0;
}

/**
  * @name         protocol_data_entity_upload_part_encode
  * @brief        封装将要发给路由器的数据上传主协议数据包中的数据字段
	* @param        *senser_data_entity_linked_list:指向欲发送给路由器的各传感器组合数据包链表
  * @param        *part_entity: 指向数据字段所拼接转换完成的字符串的指针
  * @param        *part_len: 指向数据字段所拼接转换完成的字符串的长度的指针
  * @retval       协议数据包是否成功制成  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/27  09:50    
  */
uint8_t protocol_data_entity_upload_part_encode(Linked_List * senser_data_entity_linked_list, uint8_t *part_entity, uint8_t *part_len)
{	
	uint16_t i, j, k;
	uint16_t q = 0;
	uint8_t len;
	Linked_List *senser_data_entity_point;
	uint8_t sensor_count = GetNodeNum(senser_data_entity_linked_list);

	part_entity[0] = sensor_count;
	
	for(i = 0; i < sensor_count; i++){
		senser_data_entity_point = GetNode(senser_data_entity_linked_list, i);
		part_entity[1+q] = senser_data_entity_point->sensor_data_package_point->entity_length; q++;
		part_entity[1+q] = senser_data_entity_point->sensor_data_package_point->sensor_type;   q++;
		part_entity[1+q] = senser_data_entity_point->sensor_data_package_point->data_num;      q++;
		for(j = 0; j < senser_data_entity_point->sensor_data_package_point->data_num; j++){
			part_entity[1+j+q] = senser_data_entity_point->sensor_data_package_point->sensor_data_entity_detail_point->sensor_data_item_type; q++;
			part_entity[1+j+q] = senser_data_entity_point->sensor_data_package_point->sensor_data_entity_detail_point->sensor_data_item_length; q++;
			for(k = 0; k < senser_data_entity_point->sensor_data_package_point->sensor_data_entity_detail_point->sensor_data_item_length; k++){
				part_entity[1+j+k+q] = senser_data_entity_point->sensor_data_package_point->sensor_data_entity_detail_point->sensor_data_item_entity[k];
			}
			q += k;
		}
	}
	
	len = q;
	*part_len = len;
	
	return 0;
}


/**
  * @name         protocol_warning_upload_part_encode
  * @brief        封装将要发给路由器的告警信息上报主协议数据包中的数据字段
	* @param        warning_count:欲向路由器上报的警告数量
	* @param        *warning_list:指向欲向路由器上报的警告实体的指针
  * @param        *part_entity: 指向数据字段所拼接转换完成的字符串的指针
  * @param        *part_len: 指向数据字段所拼接转换完成的字符串的长度的指针
  * @retval       协议数据包是否成功制成  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/27  09:50    
  */
uint8_t protocol_warning_upload_part_encode(uint8_t warning_count, uint8_t * warning_list, uint8_t *part_entity, uint8_t *part_len)
{	
	uint16_t i;
	uint8_t len;

	part_entity[0] = warning_count;
	
	for(i = 0; i < warning_count; i++) part_entity[1 + i] = warning_list[i];
	
	len = warning_count + 1;
	*part_len = len;
	
	return 0;
}
/********************************************************************************************************/
/*************************AP端上行数据至路由器段协议定义   程序段结束************************************/
/********************************************************************************************************/



/********************************************************************************************************/
/*                                                                                                      */
/*                        AP端上行数据至路由器段协议定义    AP端发起                                    */
/*                                   程序段开始                                                         */
/*                                                                                                      */
/*                                                                                                      */
/********************************************************************************************************/
/**
  * @name         protocol_system_status_upload_part_encode(待开发)
  */
uint8_t protocol_system_status_upload_part_encode()
{	
	
	return 0;
}

/********************************************************************************************************/
/***********************AP端上行数据至路由器段协议定义    路由器发起   程序段结束************************/
/********************************************************************************************************/

/********************************************************************************************************/
/*                            封包程序段        END                                                     */
/********************************************************************************************************/



/********************************************************************************************************/
/*                            解包程序段        START                                                   */
/********************************************************************************************************/
/********************************************************************************************************/
/*                                                                                                      */
/*                        路由器下行数据至AP端段协议定义    AP端发起                                    */
/*                                   程序段开始                                                         */
/*                                                                                                      */
/*                                                                                                      */
/********************************************************************************************************/
/**
  * @name         protocol_device_registere_responce_part_decode
  * @brief        路由器发来的主协议数据包中注册响应数据实体解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_device_registere_responce_part_decode(uint8_t *package_entity, uint8_t package_len){
	register_data_basic_settings.DataExistFlag = 0x01;
	register_data_basic_settings.MasterID[0] = package_entity[3];
	register_data_basic_settings.MasterID[1] = package_entity[4];
	register_data_basic_settings.SlaveID[0] = package_entity[5];
	register_data_basic_settings.SlaveID[1] = package_entity[6];
	
	//所有数据读取成功后统一写入
	if(ee_WriteBytes(&register_data_basic_settings.DataExistFlag, register_address_basic_settings.DataExistFlag, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(register_data_basic_settings.MasterID, register_address_basic_settings.MasterID, 2))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(register_data_basic_settings.SlaveID, register_address_basic_settings.SlaveID, 2))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	
	system_state_definition = SYSTEM_STATE_REGISTERED;
	
	return 0;
}

/**
  * @name         protocol_sensor_parameter_responce_part_decode
  * @brief        路由器发来的主协议数据包中传感器参数确认/返回数据实体解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/29  15:25
  */
uint8_t protocol_sensor_parameter_responce_part_decode(uint8_t *package_entity, uint8_t package_len, Linked_List *sensor_parameter_recovery_linked_list_header){
//	reg_data_sensor_parm sensor_parm;
//	reg_data_sensor_ID sensor_ID;
	uint16_t offset_address = 0;                                      //EEPROM偏移地址 
	
	uint8_t sensor_parameter_message_length = 0;       //传感器配置参数总长度
	uint8_t start_stone = 0;                           //每个数据段起点标记
	uint8_t i, j;


	register_data_basic_settings.SensorNum = package_entity[9];
	register_data_basic_settings.SenConAuto = package_entity[10];
	
	if(register_data_basic_settings.SenConAuto == 0x01){
		register_data_basic_settings.ComModuleType = package_entity[12];
		register_data_sensor_parm.SensorType = SensorType_Electric_Meter_Connected;
		register_data_sensor_parm.SensorParaNum = 1;
		register_data_sensor_parm.SensorIDLen = 6;
		register_data_sensor_parm.sensorParameter[0] = 0x01;
		
		electric_meter_address_auto_get(register_data_sensor_ID.sensorID, SENSOR_ADDRESS_FINDING_TIMES);  //电表ID自动获取
	}
	else if(register_data_basic_settings.SenConAuto == 0x02){
		start_stone = 11;

		for(i = 0; i < register_data_basic_settings.SensorNum; i++){
			sensor_parameter_recovery_linked_list_header->sensor_data_package_point = NULL;
			sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID = mymalloc(sizeof(sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID));
			sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm = mymalloc(sizeof(sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm));
			sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_ID = mymalloc(sizeof(sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_ID));
			
			/*************参数解析部分****************/
			sensor_parameter_message_length = package_entity[start_stone];
			sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->SensorType = package_entity[start_stone + 1];
			sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->SensorIDLen = package_entity[start_stone + 3];
			for(j = 0; j < sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->SensorIDLen; j++) sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_ID->sensorID[j] = package_entity[start_stone + 4 + j];
			sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->SensorParaNum = package_entity[start_stone + sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->SensorIDLen + 4];
			
			memset(sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->sensorParameter, 0, sizeof(sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->sensorParameter));
			for(j = 0; j < sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->SensorParaNum; j++) {                      //暂时只处理了正向有功参数的转换
				switch(package_entity[start_stone + sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->SensorIDLen + 5 + j]){
					case 0x01:
						sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->sensorParameter[0] |= 0x01;
						break;
				}
			}
			
			if(register_data_basic_settings.SensorNum != 0) InsertNode(sensor_parameter_recovery_linked_list_header, TAIL);    //在尾部插入节点
			start_stone += sensor_parameter_message_length;
			/*************参数解析部分****************/
			
			
			/*************数据存储部分  写入EEPROM****************/
			if(ee_WriteBytes(&sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->SensorType, offset_address + register_address_sensor_parm.SensorType, 1))
				error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
			offset_address += AP_BYTE_OFFSET;
			
			offset_address += AP_BYTE_OFFSET;
			
			if(ee_WriteBytes(&sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->SensorIDLen, offset_address + register_address_sensor_parm.SensorIDLen, 1))
				error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
			offset_address += AP_BYTE_OFFSET;
			
			offset_address += AP_BYTE_OFFSET * 5;
			
			if(ee_WriteBytes(sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_parm->sensorParameter, offset_address + register_address_sensor_parm.sensorParameter, 8))
				error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
			offset_address += AP_BYTE_OFFSET * 8;
			
			if(ee_WriteBytes(sensor_parameter_recovery_linked_list_header->sensor_parm_and_ID->sensor_ID->sensorID, offset_address + register_address_sensor_ID.sensorID - 16, 16))
				error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
			offset_address += AP_BYTE_OFFSET * 16;
			/*************数据存储部分  写入EEPROM****************/
		}
	
	}
	
	system_state_definition = SYSTEM_STATE_SENSOR_PARAMETER_GET_DOWN;
	
	return 0;
}

/**
  * @name         protocol_time_sync_responce_part_decode
  * @brief        路由器发来的主协议数据包中时间同步确认解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/29  15:25
  */
uint8_t protocol_time_sync_responce_part_decode(uint8_t *package_entity, uint8_t package_len){
	//时间格式校验	
		if(package_entity[9] > 0x99 || 
		 package_entity[10] < 0x01 || package_entity[10] > 0x12 ||
  	 package_entity[11] < 0x01   || package_entity[11] > 0x31 || 
		 package_entity[12] > 0x24 ||
		 package_entity[13] > 0x60 ||
		 package_entity[14] > 0x60 ||
		 package_entity[15] < 0x01   || package_entity[15] > 0x07) return 1;  
//	if(package_entity[9] < 0x00 || package_entity[9] > 0x99 || 
//		 package_entity[10] < 0x01 || package_entity[10] > 0x12 ||
//  	 package_entity[11] < 0x01   || package_entity[11] > 0x31 || 
//		 package_entity[12] < 0x00   || package_entity[12] > 0x24 ||
//		 package_entity[13] < 0x00 || package_entity[13] > 0x60 ||
//		 package_entity[14] < 0x00 || package_entity[14] > 0x60 ||
//		 package_entity[15] < 0x01   || package_entity[15] > 0x07) return 1;  
	
	register_data_time_settings.Year = package_entity[9];
	register_data_time_settings.Month = package_entity[10];
	register_data_time_settings.Day = package_entity[11];
	register_data_time_settings.Hour = package_entity[12];
	register_data_time_settings.Minute = package_entity[13];
	register_data_time_settings.Second = package_entity[14];
	register_data_time_settings.Week = package_entity[15];
	
	if(RTC_CalendarConfig(&hrtc, &register_data_time_settings))
		error_code_handle(ERROR_CODE_RTC_Init_Set_FAIL);
	
	register_data_time_settings.GapTime[0] = package_entity[16];
	register_data_time_settings.GapTime[1] = package_entity[17];
	register_data_time_settings.GapTime[2] = package_entity[18];
	
	register_data_time_settings.WindowLength = package_entity[19];
       
	system_data_update_time_settings.gap_time = (uint32_t)register_data_time_settings.GapTime[0] << 16 | 
																							(uint32_t)register_data_time_settings.GapTime[1] << 8  |
	                                            (uint32_t)register_data_time_settings.GapTime[2];
	
	system_data_update_time_settings.window_length = register_data_time_settings.WindowLength;
	
	if(system_data_update_time_settings.window_length > system_data_update_time_settings.gap_time){
		error_code_handle(ERROR_CODE_Data_UpdateTime_Settings_Set_FAIL);
		return 1;
	}
	
	//所有数据读取成功后统一写入
	if(ee_WriteBytes(&register_data_time_settings.Year, register_address_time_settings.Year, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(&register_data_time_settings.Month, register_address_time_settings.Month, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(&register_data_time_settings.Day, register_address_time_settings.Day, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(&register_data_time_settings.Hour, register_address_time_settings.Hour, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(&register_data_time_settings.Minute, register_address_time_settings.Minute, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(&register_data_time_settings.Second, register_address_time_settings.Second, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(&register_data_time_settings.Week, register_address_time_settings.Week, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(register_data_time_settings.GapTime, register_address_time_settings.GapTime, 3))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	if(ee_WriteBytes(&register_data_time_settings.WindowLength, register_address_time_settings.WindowLength, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);

	system_state_definition = SYSTEM_STATE_TIME_SYNC_ACK_GET_DOWN;
	
	return 0;
}

/**
  * @name         protocol_data_entity_ack_part_decode
  * @brief        路由器发来的主协议数据包中数据上传确认解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_data_entity_ack_part_decode(uint8_t *package_entity, uint8_t package_len){
	//暂无需任何解码操作
	system_state_definition = SYSTEM_STATE_DATA_ENTITY_ACK_GET_DOWN;
	
	return 0;
}

/**
  * @name         protocol_system_warning_ack_part_decode
  * @brief        路由器发来的主协议数据包中告警信息确认解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_system_warning_ack_part_decode(uint8_t *package_entity, uint8_t package_len){
	//暂无需任何解码操作
	
	return 0;
}



/*******************************服务器发起的数据包对***********************************/


/**
  * @name         protocol_period_change_request_decode
  * @brief        路由器发来的主协议数据包中周期更改请求解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_period_change_request_decode(uint8_t package_serial_ID, uint8_t *package_entity, uint8_t package_len){
	uint8_t ack_message_entity[MAX_ACK_MESSAGE_LEN];
	uint8_t ack_message_length;
	
	if(protocol_time_sync_responce_part_decode(package_entity, package_len))
			error_code_handle(ERROR_CODE_Period_Change_FAIL);
	
	protocol_main_package_encode(&package_serial_ID, register_data_basic_settings.SlaveID, register_data_basic_settings.MasterID,
			PROTOCOL_PERIOD_CHANGE_RESPONCE_UP, 0, NULL, ack_message_entity, &ack_message_length);
	message_send_to_router(ack_message_entity, ack_message_length);
	
	return 0;
}


/**
  * @name         protocol_sensor_parameter_change_request_part_decode
  * @brief        路由器发来的主协议数据包中传感器参数变更请求解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_sensor_parameter_change_request_part_decode(uint8_t package_serial_ID, uint8_t *package_entity, uint8_t package_len, Linked_List *sensor_parameter_recovery_linked_list_header){
	uint8_t ack_message_entity[MAX_ACK_MESSAGE_LEN];
	uint8_t ack_message_length;
	
	if(protocol_sensor_parameter_responce_part_decode(package_entity, package_len, sensor_parameter_recovery_linked_list_header))
		error_code_handle(ERROR_CODE_Sensor_Parameter_Change_FAIL);
	
	protocol_main_package_encode(&package_serial_ID, register_data_basic_settings.SlaveID, register_data_basic_settings.MasterID,
			PROTOCOL_SENSOR_PARAMETER_CHANGE_RESPONCE_UP, 0, NULL, ack_message_entity, &ack_message_length);
	message_send_to_router(ack_message_entity, ack_message_length);
	
	return 0;
}

/**
  * @name         protocol_communication_key_update_request_part_decode
  * @brief        路由器发来的主协议数据包中更新通信密钥请求解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_communication_key_update_request_part_decode(uint8_t package_serial_ID, uint8_t *package_entity, uint8_t package_len){
	uint8_t i;
	uint8_t ack_message_entity[MAX_ACK_MESSAGE_LEN];
	uint8_t ack_message_length;
	
	for(i = 0; i < 16; i++) register_data_communication_key.CommunicationKey[i] = package_entity[9 + i];
	
	if(ee_WriteBytes(register_data_communication_key.CommunicationKey, register_address_communication_key.CommunicationKey, 16))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	
	protocol_main_package_encode(&package_serial_ID, register_data_basic_settings.SlaveID, register_data_basic_settings.MasterID,
			PROTOCOL_COMMUNICATION_KEY_UPDATE_RESPONCE_UP, 0, NULL, ack_message_entity, &ack_message_length);
	message_send_to_router(ack_message_entity, ack_message_length);
	
	return 0;
}

/**
  * @name         protocol_network_switching_request_part_decode(尚未开发！！！)
  * @brief        路由器发来的主协议数据包中退网换区命令解析(尚未开发！！！)
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_network_switching_request_part_decode(uint8_t package_serial_ID, uint8_t *package_entity, uint8_t package_len){
	//该功能尚未开发！！！
	
	return 0;
}

/**
  * @name         protocol_reboot_request_part_decode
  * @brief        路由器发来的主协议数据包中重启命令解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_reboot_request_part_decode(uint8_t package_serial_ID, uint8_t *package_entity, uint8_t package_len){
	uint8_t ack_message_entity[MAX_ACK_MESSAGE_LEN];
	uint8_t ack_message_length;
	
	protocol_main_package_encode(&package_serial_ID, register_data_basic_settings.SlaveID, register_data_basic_settings.MasterID,
			PROTOCOL_REBOOT_RESPONCE_UP, 0, NULL, ack_message_entity, &ack_message_length);
	message_send_to_router(ack_message_entity, ack_message_length);
	
	NVIC_SystemReset();
	
	return 0;
}

/**
  * @name         protocol_device_state_request_part_decode
  * @brief        路由器发来的主协议数据包中获取AP设备状态解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_device_state_request_part_decode(uint8_t package_serial_ID, uint8_t *package_entity, uint8_t package_len){
		//该功能尚未开发！！！
	
	return 0;
}

/**
  * @name         protocol_energy_saving_mode_request_part_decode
  * @brief        路由器发来的主协议数据包中节能模式更改命令解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_energy_saving_mode_request_part_decode(uint8_t package_serial_ID, uint8_t *package_entity, uint8_t package_len){
	uint8_t ack_message_entity[MAX_ACK_MESSAGE_LEN];
	uint8_t ack_message_length;
	
	register_data_basic_settings.SavePower = package_entity[9];
	
	if(ee_WriteBytes(&register_data_basic_settings.SavePower, register_address_basic_settings.SavePower, 1))
		error_code_handle(ERROR_CODE_EEPROM_Write_Bytes_FAIL);
	
	protocol_main_package_encode(&package_serial_ID, register_data_basic_settings.SlaveID, register_data_basic_settings.MasterID,
		PROTOCOL_ENERGY_SAVING_MODE_RESPONCE_UP, 0, NULL, ack_message_entity, &ack_message_length);
	message_send_to_router(ack_message_entity, ack_message_length);
	
	return 0;
}

/**
  * @name         protocol_factory_setting_reset_request_part_decode
  * @brief        路由器发来的主协议数据包中恢复出厂设置命令解析
  * @param        *package_entity: 指向接收到的数据包数据实体部分的指针
  * @param        package_len: 接收到的数据包实体部分的长度
  * @retval       协议数据包实体部分是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_factory_setting_reset_request_part_decode(uint8_t package_serial_ID, uint8_t *package_entity, uint8_t package_len, Linked_List *sensor_parameter_recovery_linked_list_header){
	uint8_t i;
	uint8_t sensor_node_num = 0;
	uint8_t ack_message_entity[MAX_ACK_MESSAGE_LEN];
	uint8_t ack_message_length;
	
	ee_ReadBytes(register_data_wipe_key.WipeKey, register_address_wipe_key.WipeKey, 16);
	for(i = 0; i < 16; i++){
		if(register_data_wipe_key.WipeKey[i] != package_entity[9 + i]){
			error_code_handle(ERROR_CODE_Wipe_Key_Check_FAIL);
			return 1;
		}
	}
	
	protocol_main_package_encode(&package_serial_ID, register_data_basic_settings.SlaveID, register_data_basic_settings.MasterID,
		PROTOCOL_FACTORY_SETTING_RESET_RESPONCE_UP, 0, NULL, ack_message_entity, &ack_message_length);
	message_send_to_router(ack_message_entity, ack_message_length);
	
	sensor_node_num = GetNodeNum(sensor_parameter_recovery_linked_list_header);
	
	ee_Erase(AP_REG_BASIC_SETTINGS, (5 * 16 + 2 * 16 * sensor_node_num));
	NVIC_SystemReset();
	
	return 0;
}



/**
  * @name         protocol_main_package_decode
  * @brief        解码由路由器发来的主协议数据包
  * @param        package_serial_ID:协议中所含有的用于校验数据包到达顺序的随机序号
  * @param        *package_entity: 指向接收到的数据包实体的指针
  * @param        package_len: 接收到的数据包的长度
  * @retval       协议数据包是否成功解析  成功则返回0  失败则返回1
	* @author       JackWilliam
	* @lastModify   2018/9/26  17:20
  */
uint8_t protocol_main_package_decode(uint8_t package_serial_ID, uint8_t *package_entity, uint8_t package_len)
{
#if ENCYPTION_ENABLE                  //使能了加密算法后的解包过程
	uint8_t decyption_message_buf[MAX_ACK_MESSAGE_LEN];
	uint8_t i, j;
	AES_KEY aes_key;
	uint8_t datelen = 0;
	uint8_t cipher[MAX_ACK_MESSAGE_LEN];
#endif	
	
	/***************数据包校验********************/
	//收到的数据包长度小于最小协议包长度
	if(package_len < PROTOCOL_MINIMUM_LENGTH){ 
		error_code_handle(ERROR_CODE_Protocol_Main_Package_Decode_Header_Length_Check_FAIL);   
		return 1;
	}  
  
	//收到的数据包的帧头不正确	
	if(package_entity[0] != PROTOCOL_HEADER){ 
		error_code_handle(ERROR_CODE_Protocol_Main_Package_Decode_Header_Top_Check_FAIL);   
		return 1;
	}   
	
	//收到的数据包的版本号不正确
	if(package_entity[1] != PROTOCOL_VERSION){ 
		error_code_handle(ERROR_CODE_Protocol_Main_Package_Decode_Header_Version_Check_FAIL);   
		return 1;
	}    
	
	//收到的数据包的序号不正确	
	if(package_entity[2] != package_serial_ID){ 
		error_code_handle(ERROR_CODE_Protocol_Main_Package_Decode_Header_Serial_ID_Check_FAIL);   
		return 1;
	}    
	
	if(system_state_definition != SYSTEM_STATE_UNREGISTERED){   
	//已注册状态下收到的数据包源地址错误		
		if(package_entity[3] != register_data_basic_settings.MasterID[0]  || 
				package_entity[4] != register_data_basic_settings.MasterID[1]){
			error_code_handle(ERROR_CODE_Protocol_Main_Package_Decode_Header_Master_ID_Check_FAIL);   
			return 1;
				
		}
	
	//已注册状态下收到的数据包目的地址错误		
		if(package_entity[5] != register_data_basic_settings.SlaveID[0]  || 
				package_entity[6] != register_data_basic_settings.SlaveID[1]){ 
					error_code_handle(ERROR_CODE_Protocol_Main_Package_Decode_Header_Slave_ID_Check_FAIL);   
					return 1;
		}  
	}	
	
	
#if ENCYPTION_ENABLE                  //使能了加密算法后的解包过程
	datelen = package_entity[8];
	if(datelen != 0){
		for(i = 9, j = 0; i < 9 + datelen; i++, j++)
			cipher[j] = package_entity[i];
		AES_set_decrypt_key(register_data_communication_key.CommunicationKey, 128, &aes_key);
		AES_decrypt(cipher, decyption_message_buf, &aes_key);
		for(i = 9, j = 0; i < 9 + datelen; i++, j++)
			package_entity[i] = decyption_message_buf[j];
	}
#endif
	
	/***************数据包校验********************/

		
	/***************数据包解析  通过协议中的Type位********************/
	switch(package_entity[7]){
		//AP端主动发起
		case PROTOCOL_DEVICE_REGISTERE_RESPONCE_DOWN:
			if(protocol_device_registere_responce_part_decode(package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_Device_Registere_Responce_Part_Decode_FAIL);   //路由器注册响应解码失败
			break;
	
		case PROTOCOL_SENSOR_PARAMETER_RESPONCE_DOWN:
			if(protocol_sensor_parameter_responce_part_decode(package_entity, package_len, memory_data_sensor_point_start_address))
				error_code_handle(ERROR_CODE_Protocol_Sensor_Parameter_Responce_Part_Decode_FAIL);   //路由器传感器参数确认/返回失败			
			break;
	
		case PROTOCOL_TIME_SYNC_RESPONCE_DOWN:
			if(protocol_time_sync_responce_part_decode(package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_Time_Sync_Responce_Part_Decode_FAIL);   //路由器时间同步确认失败			
			break;

		case PROTOCOL_DATA_ENTITY_ACK_DOWN:
			if(protocol_data_entity_ack_part_decode(package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_Data_Entity_Ack_Decode_FAIL);                 //路由器数据上传确认失败			
			break;	

		case PROTOCOL_SYSTEM_WARNING_ACK_DOWN:
			if(protocol_system_warning_ack_part_decode(package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_System_Warning_Ack_Part_Decode_FAIL);   //路由器告警信息确认失败			
			break;	
			
		//路由器主动发起
		case PROTOCOL_PERIOD_CHANGE_REQUEST_DOWN:
			if(protocol_period_change_request_decode(package_serial_ID, package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_Period_Change_Request_Part_Decode_FAIL);    //路由器周期更改请求失败			
			break;

		case PROTOCOL_SENSOR_PARAMETER_CHANGE_REQUEST_DOWN:
			if(protocol_sensor_parameter_change_request_part_decode(package_serial_ID, package_entity, package_len, memory_data_sensor_point_start_address))
				error_code_handle(ERROR_CODE_Protocol_Sensor_Parameter_Change_Request_Part_Decode_FAIL);   //路由器传感器参数变更请求失败			
			break;	

		case PROTOCOL_COMMUNICATION_KEY_UPDATE_REQUEST_DOWN:
			if(protocol_communication_key_update_request_part_decode(package_serial_ID, package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_Communication_Key_Update_Request_Part_Decode_FAIL);   //路由器更新通信密钥请求失败			
			break;	

		case PROTOCOL_NETWORK_SWITCHING_REQUEST_DOWN:
			if(protocol_network_switching_request_part_decode(package_serial_ID, package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_Network_Switching_Request_Part_Decode_FAIL);   //路由器退网换区命令失败			
			break;		
		
		case PROTOCOL_REBOOT_REQUEST_DOWN:
			if(protocol_reboot_request_part_decode(package_serial_ID, package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_Reboot_Request_Part_Decode_FAIL);   //路由器重启命令失败			
			break;

		case PROTOCOL_DEVICE_STATE_REQUEST_DOWN:
			if(protocol_device_state_request_part_decode(package_serial_ID, package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_Device_State_Request_Part_Decode_FAIL);   //路由器获取AP设备状态失败			
			break;

		case PROTOCOL_ENERGY_SAVING_MODE_REQUEST_DOWN:
			if(protocol_energy_saving_mode_request_part_decode(package_serial_ID, package_entity, package_len))
				error_code_handle(ERROR_CODE_Protocol_Energy_Saving_Mode_Request_Part_Decode_FAIL);    //路由器节能模式更改命令失败			
			break;		
		
		case PROTOCOL_FACTORY_SETTING_RESET_REQUEST_DOWN:
			if(protocol_factory_setting_reset_request_part_decode(package_serial_ID, package_entity, package_len, memory_data_sensor_point_start_address))
				error_code_handle(ERROR_CODE_Protocol_Factory_Setting_Reset_Request_Part_Decode_FAIL);    //路由器恢复出厂设置命令失败			
			break;	
		
	}
	/***************数据包解析  通过协议中的Type位********************/
	
	
	return 0;
}
