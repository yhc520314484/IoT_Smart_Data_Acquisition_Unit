#ifndef __EEPROM_24C256_H__
#define __EEPROM_24C256_H__

#include "main.h"
#include "stm32l4xx_hal.h"


/* 
 * AT24C02 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address
 * 1 0 1 0 A2 A1 A0 R/W
 * 1 0 1 0 0  0  0  0 = 0XA0
 * 1 0 1 0 0  0  0  1 = 0XA1 
 */

/* AT24C01/02每页有8个字节 
 * AT24C04/08A/16A每页有16个字节 
 */

//#define EEPROM_DEV_ADDR			0xA0		/* 24xx02的设备地址 */
//#define EEPROM_PAGE_SIZE		  16			  /* 24xx02的页面大小 */
//#define EEPROM_SIZE				  400			  /* 24xx02总容量 */

//#define EEPROM_DEV_ADDR			0xA0		/* 24xx02的设备地址 */
//#define EEPROM_PAGE_SIZE		  16			  /* 24xx02的页面大小 */
//#define EEPROM_SIZE				  256			  /* 24xx02总容量 */

#define EEPROM_DEV_ADDR			0xA0		/* 24xx02的设备地址 */
#define EEPROM_PAGE_SIZE		  16			  /* 24xx02的页面大小 */
#define EEPROM_SIZE				  32768			  /* 24xx02总容量 */

uint8_t EEPROM_24C256_Init(void);
uint8_t ee_CheckOk(void);
uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize);
uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize);
uint8_t ee_Erase(uint16_t _usAddress, uint16_t _usSize);
uint8_t ee_Test(void);

/******  EEPROM定义区  对应《智能路由器AP端EEPROM规划文档 v1.0.1》 *******/
/** @defgroup EEPROM寄存器规划 
  * @{
  */
#define AP_REG_BASIC_SETTINGS                        ((uint16_t)0x0000)                   //AP端基本配置寄存器       
#define AP_REG_TIME_SETTINGS                         ((uint16_t)0x0080)                   //AP端时间寄存器
#define AP_REG_WARNING_STORE                         ((uint16_t)0x0100)                   //AP端错误告警寄存器
#define AP_REG_COMMUNICATION_KEY                     ((uint16_t)0x0180)                   //通信密钥存储寄存器
#define AP_REG_WIPE_KEY                              ((uint16_t)0x0200)                   //恢复出厂设置密钥存储寄存器

#define AP_REG_SENSOR_PARM                          ((uint16_t)0x2000)                   //传感器参数存储寄存器
#define AP_REG_SENSOR_ID                            ((uint16_t)0x2080)                   //传感器 ID存储寄存器

#define AP_REG_OFFSET                                ((uint16_t)0x0080)                   //寄存器偏移地址
#define AP_BYTE_OFFSET                               ((uint16_t)0x0008)                   //字节偏移地址

/**
  * @brief AP端基本配置寄存器单独字节地址定义
  */

typedef struct
{
  uint16_t DataExistFlag;          /*!< EEPROM中是否保存有曾经成功向路由器申请的ID等信息的标志位, 地址偏移: 0x00 */
	uint16_t SlaveID;         		   /*!< 由路由器所分配的AP端内网ID,                               地址偏移: 0x08 */
	uint16_t MasterID;         		   /*!< 路由器端的内网ID,                                         地址偏移: 0x18 */
	uint16_t SensorNum;              /*!< AP端所连接的传感器数量,                                   地址偏移: 0x28 */
	uint16_t PowerSupply;            /*!< AP端的电源供电方式，默认为市电供电,                       地址偏移: 0x30 */
	uint16_t SavePower;              /*!< AP端是否设置为节能模式，默认为节能模式,                   地址偏移: 0x38 */
	uint16_t ComModuleType;          /*!< AP端所连接的通信模块类型,                                 地址偏移: 0x40 */
	uint16_t SenConAuto;            /*!< AP端所接收到的传感器参数配置模式是否为自动模式,            地址偏移: 0x48 */
} RegAddBasicSettings;



/**
  * @brief AP端基本配置寄存器变量定义
  */



typedef struct
{
  uint8_t DataExistFlag;          /*!< EEPROM中是否保存有曾经成功向路由器申请的ID等信息的标志位, 地址偏移: 0x00 */
	uint8_t SlaveID[2];         		/*!< 由路由器所分配的AP端内网ID,                               地址偏移: 0x08 */
	uint8_t MasterID[2];         	  /*!< 路由器端的内网ID,                                         地址偏移: 0x18 */
	uint8_t SensorNum;              /*!< AP端所连接的传感器数量,                                   地址偏移: 0x28 */
	uint8_t PowerSupply;            /*!< AP端的电源供电方式，默认为市电供电,                       地址偏移: 0x30 */
	uint8_t SavePower;              /*!< AP端是否设置为节能模式，默认为节能模式,                   地址偏移: 0x38 */
	uint8_t ComModuleType;          /*!< AP端所连接的通信模块类型,                                 地址偏移: 0x40 */
	uint8_t SenConAuto;            /*!< AP端所接收到的传感器参数配置模式是否为自动模式,            地址偏移: 0x48 */
} reg_data_basic_settings;

/**
  * @brief ComModuleType相关连接的宏定义
  */
#define ComModuleType_LoRa_SX1278_Connected                 0x01
#define ComModuleType_WiFi_ESP8266_Connected                0x02
#define ComModuleType_BlueTooth_nRF52832_Connected          0x03
#define ComModuleType_NB_IoT_BC26_Connected                 0x04


/**
  * @brief AP端时间寄存器单独字节地址定义
  */

typedef struct
{
  uint16_t GapTime;          /*!< 每个数据上报周期的长度，即数据上报的间隔时间，以秒为单位,            地址偏移: 0x00 */
	uint16_t WindowLength;     /*!< 每个数据上报的窗口时间，以秒为单位,                                  地址偏移: 0x18 */
	uint16_t Year;         	   /*!< 最后一次与路由器进行时间同步的时刻信息中的年份信息，以BCD码表示,     地址偏移: 0x20 */
	uint16_t Month;            /*!< 最后一次与路由器进行时间同步的时刻信息中的月份信息，以BCD码表示,     地址偏移: 0x28 */
	uint16_t Day;              /*!< 最后一次与路由器进行时间同步的时刻信息中的月份信息，以BCD码表示,     地址偏移: 0x30 */
	uint16_t Hour;             /*!< 最后一次与路由器进行时间同步的时刻信息中的日期信息，以BCD码表示,     地址偏移: 0x38 */
	uint16_t Minute;           /*!< 最后一次与路由器进行时间同步的时刻信息中的分钟信息，以BCD码表示,     地址偏移: 0x40 */
	uint16_t Second;           /*!< 最后一次与路由器进行时间同步的时刻信息中的秒钟信息，以BCD码表示,     地址偏移: 0x48 */
	uint16_t Week;             /*!< 最后一次与路由器进行时间同步的时刻信息中的星期信息，以BCD码表示,     地址偏移: 0x50 */
} RegAddTimeSettings;



/**
  * @brief AP端时间寄存器变量定义
  */

typedef struct
{
  uint8_t GapTime[3];       /*!< 每个数据上报周期的长度，即数据上报的间隔时间，以秒为单位,            地址偏移: 0x00 */
	uint8_t WindowLength;     /*!< 每个数据上报的窗口时间，以秒为单位,                                  地址偏移: 0x18 */
	uint8_t Year;         	  /*!< 最后一次与路由器进行时间同步的时刻信息中的年份信息，以BCD码表示,     地址偏移: 0x20 */
	uint8_t Month;            /*!< 最后一次与路由器进行时间同步的时刻信息中的月份信息，以BCD码表示,     地址偏移: 0x28 */
	uint8_t Day;              /*!< 最后一次与路由器进行时间同步的时刻信息中的月份信息，以BCD码表示,     地址偏移: 0x30 */
	uint8_t Hour;             /*!< 最后一次与路由器进行时间同步的时刻信息中的日期信息，以BCD码表示,     地址偏移: 0x38 */
	uint8_t Minute;           /*!< 最后一次与路由器进行时间同步的时刻信息中的分钟信息，以BCD码表示,     地址偏移: 0x40 */
	uint8_t Second;           /*!< 最后一次与路由器进行时间同步的时刻信息中的秒钟信息，以BCD码表示,     地址偏移: 0x48 */
	uint8_t Week;             /*!< 最后一次与路由器进行时间同步的时刻信息中的星期信息，以BCD码表示,     地址偏移: 0x50 */
} reg_data_time_settings;

/**
  * @brief AP端错误告警寄存器单独字节地址定义
  */

typedef struct
{
	/*!< AP端是否已将告警信息上报至路由器，将在收到路由器告警信息确认后，
				则会被软件重新清零,            
				地址偏移: 0x00 */
  uint16_t AlarmUpdata;
	
	/*!< AP端是否在外设与通信模块初始化过程中出现需要软件复位才能修正的错误状态，
				若错误状态在软件复位后被正确修正，则会被软件重新清零,                                  
				地址偏移: 0x08 */	
	uint16_t InitAlarm;     
	
	/*!< AP端在将实际数据上传至路由器的过程中，整个窗口时间内均未收到来自路由器的数据上传确认，
				若在接下来数据周期的窗口时间内收到来自路由器的响应，则会被软件重新清零,     
				地址偏移: 0x10 */
	uint16_t DataUploadAlarm;         	
	
	/*!< AP端在与路由器的传感器参数请求或时间同步过程中出现错误状态，
				若错误状态在重新接收数据包后被正确修正，则会被软件重新清零,     
				地址偏移: 0x18 */
	uint16_t SyncAlarm;   
	
	/*!< AP端供电所使用的电池电量已无法满足AP端正常功能运行的要求，
				若电池被成功更换，则该位将被清零,     
				地址偏移: 0x20 */
	uint16_t LowPowerAlarm;              
	
	/*!< AP端告警信息上传无应答的次数，
				若收到一次来自路由器的告警信息确认后，则会被软件重新清零,     
				地址偏移: 0x40 */
	uint16_t AlarmUploadNoACKCount;             
	
	/*!< AP端在外设与通信模块初始化过程中出现需要软件复位才能修正的错误状态后进行软件复位的次数，
				若错误状态在软件复位后被正确修正，则会被软件重新清零,     
				地址偏移: 0x48 */
	uint16_t InitResetCount;           
	
	/*!< AP端数据上传无应答的次数，若收到一次来自路由器的数据上传确认后，
				则会被软件重新清零,     
				地址偏移: 0x50 */
	uint16_t DataUploadNoACKCount; 
} RegAddWarningStore;


/**
  * @brief AP端错误告警寄存器变量定义
  */

typedef struct
{
	/*!< AP端是否已将告警信息上报至路由器，将在收到路由器告警信息确认后，
				则会被软件重新清零,            
				地址偏移: 0x00 */
  uint8_t AlarmUpdata;
	
	/*!< AP端是否在外设与通信模块初始化过程中出现需要软件复位才能修正的错误状态，
				若错误状态在软件复位后被正确修正，则会被软件重新清零,                                  
				地址偏移: 0x08 */	
	uint8_t InitAlarm;     
	
	/*!< AP端在将实际数据上传至路由器的过程中，整个窗口时间内均未收到来自路由器的数据上传确认，
				若在接下来数据周期的窗口时间内收到来自路由器的响应，则会被软件重新清零,     
				地址偏移: 0x10 */
	uint8_t DataUploadAlarm;         	
	
	/*!< AP端在与路由器的传感器参数请求或时间同步过程中出现错误状态，
				若错误状态在重新接收数据包后被正确修正，则会被软件重新清零,     
				地址偏移: 0x18 */
	uint8_t SyncAlarm;   
	
	/*!< AP端供电所使用的电池电量已无法满足AP端正常功能运行的要求，
				若电池被成功更换，则该位将被清零,     
				地址偏移: 0x20 */
	uint8_t LowPowerAlarm;              
	
	/*!< AP端告警信息上传无应答的次数，
				若收到一次来自路由器的告警信息确认后，则会被软件重新清零,     
				地址偏移: 0x40 */
	uint8_t AlarmUploadNoACKCount;             
	
	/*!< AP端在外设与通信模块初始化过程中出现需要软件复位才能修正的错误状态后进行软件复位的次数，
				若错误状态在软件复位后被正确修正，则会被软件重新清零,     
				地址偏移: 0x48 */
	uint8_t InitResetCount;           
	
	/*!< AP端数据上传无应答的次数，若收到一次来自路由器的数据上传确认后，
				则会被软件重新清零,     
				地址偏移: 0x50 */
	uint8_t DataUploadNoACKCount; 
} reg_data_warning_store;

/**
  * @brief 通信密钥存储寄存器单独字节地址定义
  */
typedef struct
{
	  uint16_t CommunicationKey;          /*!< AP端与路由器通信时使用的AES-128bit信息加密的密钥，该密钥可由路由器进行更新,            地址偏移: 0x00 */
} RegAddCommunicationKey;


/**
  * @brief 通信密钥存储寄存器变量定义
  */
typedef struct
{
	  uint8_t CommunicationKey[16];          /*!< AP端与路由器通信时使用的AES-128bit信息加密的密钥，该密钥可由路由器进行更新,            地址偏移: 0x00 */
} reg_data_communication_key;

/**
  * @brief 恢复出厂设置密钥存储寄存器单独字节地址定义
  */
typedef struct
{
	  uint16_t WipeKey;          /*!< AP端恢复出厂设置时所需要验证的16位密钥，该密钥不可由路由器进行更新,            地址偏移: 0x00 */
} RegAddWipeKey;



/**
  * @brief 恢复出厂设置密钥存储寄存器变量定义
  */
typedef struct
{
	  uint8_t WipeKey[16];          /*!< AP端恢复出厂设置时所需要验证的16位密钥，该密钥不可由路由器进行更新,            地址偏移: 0x00 */
} reg_data_wipe_key;


/**
  * @brief 传感器参数存储寄存器单独字节地址定义
  */

typedef struct
{ 
  uint16_t SensorType;            /*!< AP端所连接的传感器类型,            地址偏移: 0x00 */
	uint16_t SensorParaNum;         /*!< AP端所连接的传感器需要采集的数据类型数目,         地址偏移: 0x08 */
	uint16_t SensorIDLen;         	/*!< AP端所连接的传感器的ID长度,     地址偏移: 0x10 */
	
	/*!< AP端所连接的不同传感器正常通信所需要的通信速率，其中若传感器为电表，
				则该项所代表的是电表对应的RS485串口波特率（十六进制数）,     
				地址偏移: 0x18 */
	uint16_t SensorComSpeed; 

	/*!< 路由器所要求AP端连接的第一个传感器所需要上报的数据类型，每一位代表一种传感器参数，
				1代表该参数需要采集上传，0代表该参数无需采集上传,     
				地址偏移: 0x40 */	
	uint16_t sensorParameter;     
} RegAddsensorParm;


/**
  * @brief 传感器参数存储寄存器变量定义
  */

typedef struct
{ 
  uint8_t SensorType;            /*!< AP端所连接的传感器类型,            地址偏移: 0x00 */
	uint8_t SensorParaNum;         /*!< AP端所连接的传感器需要采集的数据类型数目,         地址偏移: 0x08 */
	uint8_t SensorIDLen;         	/*!< AP端所连接的传感器的ID长度,     地址偏移: 0x10 */
	
	/*!< AP端所连接的不同传感器正常通信所需要的通信速率，其中若传感器为电表，
				则该项所代表的是电表对应的RS485串口波特率（十六进制数）,     
				地址偏移: 0x18 */
	uint8_t SensorComSpeed[5]; 

	/*!< 路由器所要求AP端连接的第一个传感器所需要上报的数据类型，每一位代表一种传感器参数，
				1代表该参数需要采集上传，0代表该参数无需采集上传,     
				地址偏移: 0x40 */	
	uint8_t sensorParameter[8];     
} reg_data_sensor_parm;


/**
  * @brief 传感器 ID存储寄存器单独字节地址定义
  */
typedef struct
{
	  uint16_t sensorID;          /*!< 传感器ID地址定义,            地址偏移: 0x00 */
} RegAddsensorID;


/**
  * @brief SensorType相关的宏定义
  */
#define SensorType_Electric_Meter_Connected                 0x01
#define SensorType_Water_Meter_Connected                    0x02
#define SensorType_Gas_Meter_Connected                      0x03
#define SensorType_Image_Source_Connected                   0x04
#define SensorType_Other_Senser_Connected                   0x05

/**
  * @brief 传感器 ID存储寄存器变量定义
  */
typedef struct
{
	  uint8_t sensorID[16];          /*!< 传感器ID实体数据定义,            地址偏移: 0x00 */
} reg_data_sensor_ID;


/**
  * @brief 传感器参数及ID共同定义存储寄存器变量定义
  */

typedef struct
{ 
	reg_data_sensor_parm   * sensor_parm;
	reg_data_sensor_ID     * sensor_ID;
} reg_data_sensor_parm_and_ID;

extern reg_data_basic_settings        register_data_basic_settings;
extern reg_data_time_settings         register_data_time_settings;
extern reg_data_warning_store         register_data_warning_store;
extern reg_data_communication_key     register_data_communication_key;
extern reg_data_wipe_key              register_data_wipe_key;
extern reg_data_sensor_parm          register_data_sensor_parm;
extern reg_data_sensor_ID            register_data_sensor_ID;



extern RegAddBasicSettings register_address_basic_settings;
extern RegAddTimeSettings register_address_time_settings;
extern RegAddWarningStore register_address_warning_store;
extern RegAddCommunicationKey register_address_communication_key;
extern RegAddWipeKey register_address_wipe_key;
extern RegAddsensorParm register_address_sensor_parm;
extern RegAddsensorID register_address_sensor_ID;
extern RegAddsensorParm register_address_sensor_parm;

/******  EEPROM定义区  对应《智能路由器AP端EEPROM规划文档 v1.0.1》 *******/

#endif
