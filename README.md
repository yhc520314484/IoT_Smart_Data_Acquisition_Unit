# IoT Smart Data Acquisition Unit

### Version: 1.0   Change Data: 2018/8/13

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
- Other Software Support:
  - STM32CubeL4 Firmware packet V1.12.0
  - ARM.CMSIS v5.4.0
  - Keil.STM32L4xx DFP v2.0.0
  - nRF5x for MDK Keil4 v8.17.0
  - S132 nRF52 SoftDevice 6.0.0 (Protocol Stacks on nRF52832)
  - nRF5_SDK_15.0.0_a53641a (SDK on nRF52832)

## 3.System Structure Overview (Beside Bluetooth)

- Implement Finite state machine by *Timer2* and *NVIC* *(Under Construction)*
- Implement Private Protocol (version 1.0) to communicate with Smart Router (https://github.com/kyhao/Router)
- USART1 for DAU debug, USART2 for sensor data collect, USART3 for communication module packet exchange
- Main Program Flow 
  - Step1: Initialize communication peripherals and its channels to Master(Smart Router)(Beside LoRa)
  - Step2: Read Master's ID from MCU's flash. If it not existed, sending an registered request  without ID of Master to the channel. If the Master's ID has been stored in flash, sending an registered request instead
  - Step3: Wait for several minutes(Using *Timer3* to achieve this*(Under Construction)*) to ensure Slave(this DAU) has got a response of success registered from master and store the master id into MCU's flash. If not get, back to Step2 send registered request again
  - Step4*(Under Construction)*: Processed with the finite state machine to get sensor's data from Usart2. When those data has been collected completely, sending an channel request to ask Master whether it has resource for handle sensor data or not periodically. If Master has this ability, DAU will get an response of channel allowed and send sensor data packet back to Master. If DAU not get this response, it will wait for several seconds to send channel request again
  - Step5*(Under Construction)*: After the sending process of sensor data packet, DAU will wait the reply from Master for a short time. If DAU received data got response, it will reset its state and wait finite state machine enable some flags to guide system restart the process of sensor data collection and sending. If not, DAU will resend sensor data packet immediately

## 4.Protocol and Packet Define

### Protocol Define

| Byte    | Contain                                           | Meaning          |
| ------- | ------------------------------------------------- | ---------------- |
| 0       | 0xFF                                              | Protocol Header  |
| 1       | 0x01                                              | Protocol Version |
| 2 ~ 13  | 12 Bytes Unique Device ID of Message Source       | Source ID        |
| 14 ~ 25 | 12 Bytes Unique Device ID of Message Destination  | Destination ID   |
| 26      | 0x01 ~ 0x06, 0xFF  Odd for Master, Even for Slave | Message Type     |
| 27      | 0x01 ~ 0xFF                                       | Real Data Length |
| 28      | Up to 255 bytes                                   | Real Data        |

### Packet Define (Based on 26th byte(Message Type))

- Master Side (Router)
  - 0x01 Success Registered Response
    - Success Demo: 0xFF 0x01 (12 Bytes Master ID) (12 Bytes Slave ID) 0x01 0x00
  - 0x03 Channel Allowed Response 
    - Success Demo: 0xFF 0x01 (12 Bytes Master ID) (12 Bytes Slave ID) 0x03 0x00
  - 0x05 Data Got Response
    - Success Demo: 0xFF 0x01 (12 Bytes Master ID) (12 Bytes Slave ID) 0x05 0x00
- Slave Side (DAU)
  - 0x02 Registered Request
    - Success Demo: 0xFF 0x01 (12 Bytes Slave ID) (12 Bytes 0x00) 0x02 0x00
  - 0x04 Channel Request
    - Success Demo: 0xFF 0x01 (12 Bytes Slave ID) (12 Bytes Master ID) 0x04 0x00
  - 0x05 Sensor Data Packet
    - Success Demo: 0xFF 0x01 (12 Bytes Slave ID) (12 Bytes Master ID) 0x06 (Datalen) (Data)
- **Tips1: 0x0d 0x0a should attach to the end of each packet because of DAU Serial Limited(Plan to corrected in the further version)**
- **Tips2: Debug command will be add in the further version. Debug information will NOT send to Master** 