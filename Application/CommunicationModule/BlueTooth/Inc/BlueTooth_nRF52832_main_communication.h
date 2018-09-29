#ifndef __BLUETOOTH_NRF52832_MAIN_COMMUNICATION_H__
#define __BLUETOOTH_NRF52832_MAIN_COMMUNICATION_H__

#include "main.h"
#include "stm32l4xx_hal.h"

uint8_t BlueTooth_nRF52832_Driver_Init(void);
uint8_t message_send_to_router_by_bluetooth_nrf52832(uint8_t * message_entity, uint8_t message_entity_length);

#endif
