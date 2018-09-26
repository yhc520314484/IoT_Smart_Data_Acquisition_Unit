#include "sensor_electric_meter.h"

sen_electric_meter_gen_para sensor_electric_meter_general_parameters = {
	.baudrate             =     2400,             //缺省默认波特率为2400
	.amount               =     1,                //缺省默认电表数量为1
	.ID_single_length     =     6,                //缺省默认单个电表ID长度为6
	.ID_total_length      =     6,                //缺省默认电表ID总长度为6
};


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
	
	return 0;
}
