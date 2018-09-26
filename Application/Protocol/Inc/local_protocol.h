#ifndef __LOCAL_PROTOCOL_H__
#define __LOCAL_PROTOCOL_H__

#include "main.h"

/**
  * @brief  本地网络协议数据包头定义  对应《智能路由器本地网络协议手册 v1.0.1》
  */
typedef struct {
	uint8_t message_header;                  //帧头 0xFF
	uint8_t protocol_version;                //版本信息 
	uint8_t message_serial_number;           //消息序号
	uint8_t source_ID[2];                    //消息来源的设备ID
	uint8_t destination_ID[2];               //消息目的地的设备ID
	uint8_t message_type;                    //消息类别
	uint8_t date_entity_length;              //消息所携带的数据长度  
}ProtocolHeaderDefinition;

/**
  * @brief  本地网络协议数据包定义  对应《智能路由器本地网络协议手册 v1.0.1》
  */
typedef struct {
	ProtocolHeaderDefinition frame_header;                 //消息头部
	uint8_t date_entity[255];                            //消息内容
}ProtocolDefinition;

//消息上下行类别  对应协议包中的type
//UP  AP->Router    DOWN  Router->AP
/**
  * @brief  本地网络协议数据包类型  由包头中的message_type决定 对应《智能路由器本地网络协议手册 v1.0.1》其中UP代表数据包由AP端发送上行至路由器  DOWN表示数据包由路由器下发至AP端
  */
typedef enum {
	PROTOCOL_DEVICE_REGISTERE_REQUEST_UP                  = 0x01,              //AP端注册请求
	PROTOCOL_DEVICE_REGISTERE_RESPONCE_DOWN               = 0x02,              //路由器注册响应
	PROTOCOL_SENSOR_PARAMETER_REQUEST_UP                  = 0x03,              //AP端传感器参数请求
	PROTOCOL_SENSOR_PARAMETER_RESPONCE_DOWN               = 0x04,              //路由器传感器参数确认/返回
	PROTOCOL_TIME_SYNC_REQUEST_UP                         = 0x05,              //AP端时间同步请求
	PROTOCOL_TIME_SYNC_RESPONCE_DOWN                      = 0x06,              //路由器时间同步确认                   
	PROTOCOL_DATA_ENTITY_UPLOAD_UP                        = 0x07,              //AP端数据上传
	PROTOCOL_DATA_ENTITY_ACK_DOWN                         = 0x08,              //路由器数据上传确认
	PROTOCOL_SYSTEM_WARNING_UPLOAD_UP                     = 0x09,              //AP端告警信息上报
	PROTOCOL_SYSTEM_WARNING_ACK_DOWN                      = 0x0A,              //路由器告警信息确认
	
	PROTOCOL_PERIOD_CHANGE_REQUEST_DOWN                   = 0x10,              //路由器周期更改请求
	PROTOCOL_PERIOD_CHANGE_RESPONCE_UP                    = 0x11,              //AP端周期更改确认
	PROTOCOL_SENSOR_PARAMETER_CHANGE_REQUEST_DOWN         = 0x12,              //路由器传感器参数变更请求
	PROTOCOL_SENSOR_PARAMETER_CHANGE_RESPONCE_UP          = 0x13,              //AP端传感器参数变更确认
	PROTOCOL_COMMUNICATION_KEY_UPDATE_REQUEST_DOWN        = 0x14,              //路由器更新通信密钥请求
	PROTOCOL_COMMUNICATION_KEY_UPDATE_RESPONCE_UP         = 0x15,              //AP端更新通信密钥确认	
	PROTOCOL_NETWORK_SWITCHING_REQUEST_DOWN               = 0x16,              //路由器退网换区命令
	PROTOCOL_NETWORK_SWITCHING_RESPONCE_UP                = 0x17,              //AP端退网换区确认	
	PROTOCOL_REBOOT_REQUEST_DOWN                          = 0x18,              //路由器重启命令	
	PROTOCOL_REBOOT_RESPONCE_UP                           = 0x19,              //AP端重启命令确认	
	PROTOCOL_DEVICE_STATE_REQUEST_DOWN                    = 0x1A,              //路由器获取AP设备状态
	PROTOCOL_DEVICE_STATE_RESPONCE_UP                     = 0x1B,              //AP端设备状态返回		
	PROTOCOL_ENERGY_SAVING_MODE_REQUEST_DOWN              = 0x1C,              //路由器节能模式更改命令
	PROTOCOL_ENERGY_SAVING_MODE_RESPONCE_UP               = 0x1E,              //AP端节能模式更改确认	
	
	PROTOCOL_FACTORY_SETTING_RESET_REQUEST_DOWN           = 0x30,              //路由器恢复出厂设置命令
	PROTOCOL_FACTORY_SETTING_RESET_RESPONCE_UP            = 0x31,              //AP端恢复出厂设置确认	
}ProtocolMessageTypeDefinition;			

#endif
