#ifndef __LORA_SX1278_MAIN_COMMUNICATION_H__
#define __LORA_SX1278_MAIN_COMMUNICATION_H__

#include "main.h"
#include "stm32l4xx_hal.h"

#include "platform.h"
#include "radio.h"


uint8_t LoRa_SX1278_Radio_Driver_Init(void);
uint8_t message_send_to_router_by_lora_sx1278(uint8_t * message_entity, uint8_t message_entity_length);

#endif
