#ifndef __NB_IOT_BC26_MAIN_COMMUNICATION_H__
#define __NB_IOT_BC26_MAIN_COMMUNICATION_H__

#include "main.h"
#include "stm32l4xx_hal.h"

uint8_t NB_IoT_BC26_Driver_Init(void);
uint8_t message_send_to_router_by_nb_iot_bc26(uint8_t * message_entity, uint8_t message_entity_length);
#endif

