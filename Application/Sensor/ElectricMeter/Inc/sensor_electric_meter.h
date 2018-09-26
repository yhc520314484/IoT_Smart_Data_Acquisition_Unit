#ifndef __SENSOR_ELECTRIC_METER_H__
#define __SENSOR_ELECTRIC_METER_H__

#include "main.h"
#include "stm32l4xx_hal.h"

#define ID_MAX_LENGTH         1300

/**
  * @brief  电表相关的通用参数设置结构体
  */
typedef struct{
	uint64_t    baudrate;                 //电表所在的RS485总线波特率
	uint16_t    amount;                   //需采集数据的电表个数
	uint16_t    ID_single_length;         //单个电表的ID长度
	uint16_t    ID_total_length;          //单个电表的ID长度
	uint8_t     ID_data[ID_MAX_LENGTH];   //电表ID实际数据
}sen_electric_meter_gen_para;

extern sen_electric_meter_gen_para sensor_electric_meter_general_parameters;


uint8_t Electric_Meter_Baudrate_Auto_Get(sen_electric_meter_gen_para * sen_para);



#endif
