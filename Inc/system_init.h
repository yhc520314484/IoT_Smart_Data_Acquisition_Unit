#ifndef __SYSTEM_INIT_H__
#define __SYSTEM_INIT_H__

#include "main.h"
#include "stm32l4xx_hal.h"
#include "linked_list.h"

/******************************AP端硬件初始化************************************/
void AP_side_hardware_init(void);                          //AP端相关硬件初始化 包括片上外设、通信模块及传感器模块
void necessary_on_chip_peripherals_init(void);             //第一阶段  初始化必要片上外设
void communication_module_init(void);                      //第二阶段  初始化通信模块
void sensor_module_init(void);                             //第三阶段  初始化传感器模块
void other_initialization_procedure(void);                 //第四阶段  其他初始化过程
uint8_t other_system_initialization_process(void);         //其他系统初始化过程
/******************************AP端硬件初始化************************************/

/******************************路由器连接初始化************************************/
void router_connection_initialization(void);               //AP端与路由器连接初始化
uint8_t get_STM32L4_chip_ID(uint8_t * STM32_ID_point);        //获取STM32内部的96位产品唯一身份标识
uint8_t read_connection_history_byte_from_eeprom(void);      //读取AP端是否曾成功与路由器建立过连接的EEPROM标志位
void AP_side_registration_recovry(void);                   //AP端设备非首次注册过程，即注册恢复过程
void AP_side_registration_for_first_time(void);            //AP端设备首次注册过程 
uint8_t EEPROM_24C256_registration_recovry_read_procedure(void); //非首次注册时从EEPROM读历史注册信息
//uint8_t EEPROM_24C256_sensor_parameter_read(uint8_t sensor_count);   //EEPROM读取传感器相关的寄存器
uint8_t EEPROM_24C256_sensor_parameter_read(uint8_t sensor_count, Linked_List *sensor_parameter_recovery_linked_list_header);   //EEPROM读取传感器相关的寄存器
/******************************路由器连接初始化************************************/

#endif
