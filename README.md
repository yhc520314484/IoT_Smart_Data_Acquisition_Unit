# IoT Smart Data Acquisition Unit  v1.1  2018/8/13:

## 1. Hardware Platform

- MCU: STM32L476RGT6 (STMicroelectronics)
  - Arm® 32-bit Cortex®-M4 CPU with FPU
  - 1MB Flash, 128 KB SRAM
  - 64 Pin, Clock Speed 80Mhz
  - 3x SPIs, (3+2)x USARTs
- Bluetooth: nRF52832 (Nordic Semiconductor)
  - Arm® 32-bit Cortex®-M4 CPU with FPU
  - Single chip, highly flexible, 2.4 GHz multi-protocol SoC
  - 1MB Flash, 64 KB RAM
  - Bluetooth® Low Energy 4.2
  - UART (RTS/CTS), 3 x Master/Slave SPI
- NB-IoT: BC95-B5 (Quectel)
  - Use USART to connected with MCU
  - AT Command Control
- RoLa: SX1278 (Semtech)
  - Use SPI to connected with MCU
- WiFi: ESP8266 (Espressif)
  - Use USART to connected with MCU
  - AT Command Control
- Ethernet: W5500 (WIZnet)
  - Use SPI to connected with MCU
  - Hardwired TCP/IP embedded Ethernet controller 

## 2. Development Environment

- IDE: Keil uVision v5.24.2.0
- Initialization Code Generator: STM32CubeMX 4.26.0
- Serial Assistant: XCOM v2.0
- Network Debug Assistant: USR-TCP232-Test
- J-Link Driver: 4.15e

## 3.System Structure Overview (Beside Bluetooth)

- Implement Finite state machine by *Timer2* and *NVIC* *(Under Construction)*
- Implement Private Protocol (ver 1.0) to communicate with Smart Router (https://github.com/kyhao/Router)
- 
- 



- STM32CubeL4 Firmware Package V1.12.0
- ARM.CMSIS v5.4.0
- Keil.STM32L4xx DFP v2.0.0
- nRF5x for MDK Keil4 v8.17.0
- S132 nRF52 SoftDevice 6.0.0 (Protocol Stacks on nRF52832)
- nRF5_SDK_15.0.0_a53641a (SDK on nRF52832)

