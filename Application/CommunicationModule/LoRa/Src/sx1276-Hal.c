/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
//#include <stdint.h>
//#include <stdbool.h> 

#include "platform.h"

#if defined( USE_SX1276_RADIO )

//#include "ioe.h"
#include "spi.h"
#include "sx1276-Hal.h"
#include "stm32l4xx_hal.h"
//#include "main.h"
//#include "../../radio/sx1276-Hal.h"

///*!
// * SX1276 RESET I/O definitions
// */
//#define RESET_IOPORT GPIOE
//#define RESET_PIN GPIO_Pin_1

///*!
// * SX1276 SPI NSS I/O definitions
// */
//#define NSS_IOPORT GPIOB
//#define NSS_PIN GPIO_Pin_7     //ԭGPIO_Pin_15

///*!
// * SX1276 DIO pins  I/O definitions
// */
//#define DIO0_IOPORT GPIOB
//#define DIO0_PIN GPIO_Pin_4

//#define DIO1_IOPORT GPIOD
//#define DIO1_PIN GPIO_Pin_5

//#define DIO2_IOPORT GPIOB
//#define DIO2_PIN GPIO_Pin_5

//#define DIO3_IOPORT GPIOD
//#define DIO3_PIN GPIO_Pin_11

//#define DIO4_IOPORT GPIOE
//#define DIO4_PIN GPIO_Pin_15

//#define DIO5_IOPORT GPIOD
//#define DIO5_PIN GPIO_Pin_8

//#define RXTX_IOPORT GPIOA                                 
//#define RXTX_PIN FEM_CTX_PIN


//#define RXE_PORT GPIOA
//#define RXE_PIN GPIO_Pin_2

//#define RXE_HIGH() HAL_GPIO_WritePin(RXE_PORT, RXE_PIN, GPIO_PIN_SET)
//#define RXE_LOW() HAL_GPIO_WritePin(RXE_PORT, RXE_PIN, GPIO_PIN_RESET)
//#define RXE_STATE() HAL_GPIO_ReadPin(RXE_PORT,RXE_PIN)

//#define TXE_PORT GPIOA
//#define TXE_PIN GPIO_Pin_3
//#define TXE_HIGH() HAL_GPIO_WritePin(TXE_PORT,TXE_PIN, GPIO_PIN_SET)
//#define TXE_LOW()	HAL_GPIO_WritePin(TXE_PORT,TXE_PIN, GPIO_PIN_RESET)
//#define TXE_STATE() HAL_GPIO_ReadPin(TXE_PORT,TXE_PIN)

/*!
 * SX1276 RESET I/O definitions
 */
#define RESET_IOPORT GPIOE
#define RESET_PIN ((uint16_t)0x0002)

/*!
 * SX1276 SPI NSS I/O definitions
 */
#define NSS_IOPORT GPIOB
#define NSS_PIN ((uint16_t)0x0080)     //ԭGPIO_Pin_15

/*!
 * SX1276 DIO pins  I/O definitions
 */
#define DIO0_IOPORT GPIOB
#define DIO0_PIN ((uint16_t)0x0010)

#define DIO1_IOPORT GPIOD
#define DIO1_PIN ((uint16_t)0x0020) 

#define DIO2_IOPORT GPIOB
#define DIO2_PIN ((uint16_t)0x0020) 

#define DIO3_IOPORT GPIOD
#define DIO3_PIN ((uint16_t)0x0800)

#define DIO4_IOPORT GPIOE
#define DIO4_PIN ((uint16_t)0x8000)

#define DIO5_IOPORT GPIOD
#define DIO5_PIN ((uint16_t)0x0100)

#define RXTX_IOPORT GPIOA                                 
#define RXTX_PIN FEM_CTX_PIN


#define RXE_PORT GPIOA
#define RXE_PIN ((uint16_t)0x0004)

#define RXE_HIGH() HAL_GPIO_WritePin(RXE_PORT, RXE_PIN, GPIO_PIN_SET)
#define RXE_LOW() HAL_GPIO_WritePin(RXE_PORT, RXE_PIN, GPIO_PIN_RESET)
#define RXE_STATE() HAL_GPIO_ReadPin(RXE_PORT,RXE_PIN)

#define TXE_PORT GPIOA
#define TXE_PIN ((uint16_t)0x0008)
#define TXE_HIGH() HAL_GPIO_WritePin(TXE_PORT,TXE_PIN, GPIO_PIN_SET)
#define TXE_LOW()	HAL_GPIO_WritePin(TXE_PORT,TXE_PIN, GPIO_PIN_RESET)
#define TXE_STATE() HAL_GPIO_ReadPin(TXE_PORT,TXE_PIN)


void Set_RF_Switch_RX(void)
{
	RXE_HIGH();
	TXE_LOW();
}

void Set_RF_Switch_TX(void)
{
	RXE_LOW();
	TXE_HIGH();
}


void SX1276SetReset( uint8_t state )
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if( state == RADIO_RESET_ON )
    {
        // Configure RESET as output
		GPIO_InitStructure.Pin = RESET_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;;
    HAL_GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
		
		// Set RESET pin to 0
        HAL_GPIO_WritePin( RESET_IOPORT, RESET_PIN, GPIO_PIN_RESET );
    }
    else
    {
		GPIO_InitStructure.Pin =  RESET_PIN;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
		
		// Set RESET pin to 1
        HAL_GPIO_WritePin( RESET_IOPORT, RESET_PIN, GPIO_PIN_SET );

    }
}



void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    HAL_GPIO_WritePin( NSS_IOPORT, NSS_PIN, GPIO_PIN_RESET );

    SpiInOut( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }

    //NSS = 1;
    HAL_GPIO_WritePin( NSS_IOPORT, NSS_PIN, GPIO_PIN_SET );
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    HAL_GPIO_WritePin( NSS_IOPORT, NSS_PIN, GPIO_PIN_RESET );

    SpiInOut( addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }

    //NSS = 1;
    HAL_GPIO_WritePin( NSS_IOPORT, NSS_PIN, GPIO_PIN_SET );
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

uint8_t SX1276ReadDio0( void )
{
    return HAL_GPIO_ReadPin( DIO0_IOPORT, DIO0_PIN );
}

uint8_t SX1276ReadDio1( void )
{
    return HAL_GPIO_ReadPin( DIO1_IOPORT, DIO1_PIN );
}

uint8_t SX1276ReadDio2( void )
{
    return HAL_GPIO_ReadPin( DIO2_IOPORT, DIO2_PIN );
//	return HAL_GPIO_ReadPin( DIO2_IOPORT, ((uint16_t)0x0004) );
	
}

uint8_t SX1276ReadDio3( void )
{
    return HAL_GPIO_ReadPin( DIO3_IOPORT, DIO3_PIN );
}

uint8_t SX1276ReadDio4( void )
{
    return HAL_GPIO_ReadPin( DIO4_IOPORT, DIO4_PIN );
}

uint8_t SX1276ReadDio5( void )
{
    return HAL_GPIO_ReadPin( DIO5_IOPORT, DIO5_PIN );
}

//射频芯片收发切换
void SX1276WriteRxTx( uint8_t txEnable )
{
    if( txEnable != 0 )
    {
		Set_RF_Switch_TX(); //单片机将射频开关芯片切换成发射状态
//        IoePinOn( FEM_CTX_PIN );
//        IoePinOff( FEM_CPS_PIN );
    }
    else
    {
		Set_RF_Switch_RX();  //单片机将射频开关芯片切换成接收状态
//        IoePinOff( FEM_CTX_PIN );
//        IoePinOn( FEM_CPS_PIN );
    }
}

#endif // USE_SX1276_RADIO

