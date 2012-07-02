///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// Copyright Nurve Networks LLC 2009
// 
// Filename: CHAM_AVR_TWI_SPI_DRV_V010.C
//
// Original Author: Andre' LaMothe
// 
// Last Modified: 4.28.09
//
// Description: SPI/TWI library file
//  
// Overview:
//
// I/O pin map SPI/TWI interfaces
// 
// SPI_CSn  | PB2 | Pin 16 | Output
// SPI_MOSI | PB3 | Pin 17 | Output
// SPI_MISO | PB4 | Pin 18 | Input
// SPI_SCLK | PB5 | Pin 19 | Output
// 
// Specially added for SPI mux addressing 
//
// SPI_CS0  | PC2 | Pin 25 | Output
// SPI_CS1  | PC3 | Pin 26 | Output
//
// SCL      | PC5 | Pin 28 | Output
// SDA      | PC4 | Pin 28 | Input/Output (controlled by peripheral once enabled)
//
// Note: Atmel calls the I2C interface "TWI" two wire interface since it has extra features, but to us its just 
// plain old I2C.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDES ///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// this processor include needs to happen in the master file
// #define __AVR_ATmega168__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <compat/twi.h>
#include <avr/pgmspace.h>

// include local Chameleon AVR API header files
#include "CHAM_AVR_SYSTEM_IDRIVINO_V0.h"
#include "CHAM_AVR_TWI_SPI_DRV_IDRIVINO_V0.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MACROS /////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEFINES/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TYPES/CLASSES //////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBALS ////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// EXTERNALS //////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PROTOTYPES /////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS  /////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TWI/I2C INTERFACE //////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int TWI_Init(long twi_clock)
{
	// set up TWI interface and I/O ports, default TWI_CLOCK is 100K HZ in header

	// set I2C hardware to outputs, once hardware is enabled, the peripheral will control the I/O directions
	// and overide the settings of the I2C data line

	// note: the SETPORTBITS() macro can only write 1's to ports, all 0's are ignored
	TWI_DDR  =  SETPORTBITS(0,0,0,0,0,0,1,1);

	// enable pull ups on I2C etc.
	TWI_PORT =  SETPORTBITS(0,0,0,0,0,0,1,1);

	// no prescaler, otherwise a factor of 4^twps value needs to be in the denominator next to the 2.
	TWSR = 0;                         

	// must be > 10 for stable operation
	TWBR = ((F_CPU/twi_clock)-16)/2;

	// return success, later add some more relevant error codes
	return(1);

} // end TWI_Init

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TWI_Error()
{
// send message to error system

} // End TWI_Error

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TWI_Status()
{

} // End TWI_Status

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TWI_Success(int code)
{

} // End TWI_Success

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

char TWI_Action(unsigned char command)
{
	// write command to TWCR and make sure TWINT is set
	TWCR = (command | SET(TWINT));

	//now wait for TWINT to be set again (when the operation is completed)
	while (!(TWCR & SET(TWINT)));

	return (TW_STATUS);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPI FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#if 0 /////////////////////////////////////////////////

// 644
#define SPCR0	_SFR_IO8(0x2C)
#define SPIE0	7
#define SPE0	6
#define DORD0	5
#define MSTR0	4
#define CPOL0	3
#define CPHA0	2
#define SPR01	1
#define SPR00	0

#define SPSR0	_SFR_IO8(0x2D)
#define SPIF0	7
#define WCOL0	6
#define SPI2X0	0
#define SPDR0	_SFR_IO8(0X2E)



// 168 
#define SPCR    _SFR_IO8 (0x2C)

#define SPIE    7
#define SPE     6
#define DORD    5
#define MSTR    4
#define CPOL    3
#define CPHA    2
#define SPR1    1
#define SPR0    0

#define SPSR    _SFR_IO8 (0x2D)

#define SPIF    7
#define WCOL    6
#define SPI2X   0

#define SPDR    _SFR_IO8 (0x2E)

#define SPI_CS_DDR  DDRC
#define SPI_CS_PORT PORTC
#define SPI_CS0     PC2
#define SPI_CS1     PC3


#endif ////////////////////////////////////////////

int SPI_Init(unsigned char spi_rate)
{
	// initializes dedicated SPI hardware

	/*
	#ifdef __AVR_ATmega644__

	// set CS to HIGH, disable
	SET_BIT(SPI_PORT, SPI_CS);

	// set MOSI, SCLK, CS to outputs, MISO to input
	SPI_DDR = (1 << SPI_MOSI) | (1 << SPI_SCLK) | (1 << SPI_CS) | (0 << SPI_MISO);

	// enable pull up on MISO
	SET_BIT(SPI_PORT, SPI_MISO);

	// enable master, set clock rate to fclk/16, 0 polarity, 0 phase, MSB first, etc.
	SPCR0 = (0 << SPIE0) | (1 << SPE0) | (0 << DORD0) | (1 << MSTR0) | (0 << CPOL0) | (0 << CPHA0) | (1 << SPR01) | (1 << SPR00);
	#endif
	*/

	// __AVR_ATmega168__ OR  __AVR_ATmega328P__

	// set CS to SPI select channel 0 (CAN channel) default
	SPI_CS_PORT = (0 << SPI_CS1) | (0 << SPI_CS0);
	// set CS to SPI select channel 1 (SD channel)
	//SPI_CS_PORT = (0 << SPI_CS1) | (1 << SPI_CS0);
	// set CS to SPI select channel 2 (Propeller channel)
	//SPI_CS_PORT = (1 << SPI_CS1) | (0 << SPI_CS0);
	// set CS to SPI select channel 3 (Flash channel)
	//SPI_CS_PORT = (1 << SPI_CS1) | (1 << SPI_CS0);

	// set CS lines to outputs
	SPI_CS_DDR = (1 << SPI_CS1) | (1 << SPI_CS0);

	// set CS to HIGH, disable (SPI hardware on 168 "seems" to need this to be set HIGH before enabling the hardware?)
	SET_BIT(SPI_PORT, SPI_CS);

	// set MOSI, SCLK, to outputs, MISO to input, chip selects are handled seperately due to SPI 4-channel mux
	SPI_DDR = (1 << SPI_MOSI) | (1 << SPI_SCLK)  | (0 << SPI_MISO) | (0 << SPI_CS);

	// enable pull up on MISO
	SET_BIT(SPI_PORT, SPI_MISO);

	// the settings for the SPI interface can be found in the data sheet or the 644, page 166,table 16-5 roughly
	// CD-ROM file:
	// weblink: 
	// The table is re-produced below for reference
	//
	// SPI2X   SPR1    SPR0        SCK frequency
	//
	// 0       0       0           Fosc/4
	// 0       0       1           Fosc/16
	// 0       1       0           Fosc/64
	// 0       1       1           Fosc/128
	// 1       0       0           Fosc/2
	// 1       0       1           Fosc/8
	// 1       1       0           Fosc/32
	// 1       1       1           Fosc/64
	// 
	// Thus, by controlling the SPI2X bit and the SPR1/0 bits you can get any clock divider from 2..128

	// enable master, set clock rate to fclk/16, 0 polarity, 0 phase, MSB first, etc.
	SPCR = (0 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA) |  ( (spi_rate & 0x03) << SPR0);

	// set 2x speed
	//SET_BIT(SPSR, SPI2X);

	// return success, later add some more relevant error codes
	return(1);

} // end SPI_Init

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int SPI_Set_Speed(unsigned char spi_rate)
{
	// this function resets the speed of the SPI interface
	// spi_rate - [ xxxxx 2x | SPR1 | SPR0 ] - bit encoded value for the SPI2X bit and SPR0|1 rate bits
	//
	// The table is re-produced below for reference
	//
	// SPI2X   SPR1    SPR0        SCK frequency
	//
	// 0       0       0           Fosc/4
	// 0       0       1           Fosc/16
	// 0       1       0           Fosc/64
	// 0       1       1           Fosc/128
	// 1       0       0           Fosc/2
	// 1       0       1           Fosc/8
	// 1       1       0           Fosc/32
	// 1       1       1           Fosc/64

	// enable master, set clock rate to fclk/16, 0 polarity, 0 phase, MSB first, etc.
	SPCR = (0 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA) | ( (spi_rate & 0x03) << SPR0);


	// now shift the 2X rate bit to the lsb and then write it, whatever it is 0|1 to the SPI2X register bit
	WRITE_BIT(SPSR, SPI2X, (spi_rate >> 2) );

	// return success, later add some more relevant error codes
	return(1);

} // end SPI_Set_Speed

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// some alias names, if you like these better
unsigned char SPI_Write(unsigned char data8)
{
	// this function writes a data8 to the SPI transmitter


	// return success
	return(1);

} // end SPI_Write

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char SPI_Read(unsigned char data8)
{
	// this function reads a value from the SPI receiver

	// return success
	return(1);

} // end SPI_Read

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char SPI_WriteRead(unsigned char data8)
{
	// this functions writes data8 to the SPI interace while reading the next 8-bits from the slave

	// send the data out
	SPDR = data8;

	// wait for the data to transmit and for the receiver to shift in results

	while (!(SPSR & (1 << SPIF)));

	// return value from SPI interface
	return(SPDR);

} // end SPI_WriteRead

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char SPI_Send_Recv_Byte(unsigned char data8)
{
	// this functions writes data8 to the SPI interace while reading the next 8-bits from the slave

	// send the data out
	SPDR = data8;

	// wait for the data to transmit and for the receiver to shift in results

	while (!(SPSR & (1 << SPIF)));

	// return value from SPI interface
	return(SPDR);

} // end SPI_Send_Recv_Byte

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int SPI_Prop_Print_String(char *string)
{
	// print the string to Prop over SPI channel

	static int   data   = 0, 
				 status = 0;

	// print the string until NULL reached
	while ((data = *string++) != NULL)
	{
		status = SPI_Prop_Send_Cmd( GPU_GFX_TERMINAL_PRINT, data, status);
		_delay_us( SPI_PROP_STRING_DELAY_US ); 
	} 

	// return last status value
	return ( status );

} // end SPI_Prop_Print_String

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int SPI_Prop_Print_CAN_String(char *string)
{
	// print the string to Prop over SPI channel

	static int   data   = 0, 
				 status = 0;

	// print the string until NULL reached
	while ((data = *string++) != NULL)
	{
		status = SPI_Prop_Send_Cmd( GPU_GFX_CAN_SNIFFER, data, status);
		_delay_us( SPI_PROP_STRING_DELAY_US ); 
	} 

	// return last status value
	return ( status );

} // end SPI_Prop_Print_String

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0

long SPI_Prop_Send_Cmd(int cmd, int data, int status)
{
	// this function is used to send SPI commands to the Propeller slave processor

	long status_low, status_mid, status_high;

	// set CS to SPI select, select Prop SPI channel = 2 
	SPI_CS_PORT = (1 << SPI_CS1) | (0 << SPI_CS0);

	// send command byte and retrieve low byte of result
	status_low = SPI_WriteRead( cmd );                        
	_delay_us(SPI_PROP_CMD_DELAY_US);

	// send command byte and retrieve mid byte of result                     
	status_mid = SPI_WriteRead( data );                        
	_delay_us(SPI_PROP_CMD_DELAY_US);

	status_high = SPI_WriteRead( status );
	_delay_us(SPI_PROP_CMD_DELAY_US);

	// set CS to SPI select channel 0 (null)
	SPI_CS_PORT = (0 << SPI_CS1) | (0 << SPI_CS0);
	//SPI_CS_PORT = (0 << SPI_CS1) | (1 << SPI_CS0);
	_delay_us(SPI_PROP_CMD_DELAY_US);

	// return status
	// note: ignore high byte for now, so we don't have to deal with sign issues, right now, the Prop driver sends back data in 24-bit format
	// but we ignore the upper 8-bits, thus the return values have a range of +-32768 and a total mag of 65535, however, signs are correct
	// but, if we go to 24-bit data then a negative 24-bit number when casted to a long will be incorrect, since there are no 24-bit data
	// types in C, thus we would have to test the sign bit and do a 24->31 bit sign extension, blah -- so for now, 16-bit signed/unsigned return
	// values. Bottom line, Prop driver ALWAYS returns all 24-bits, but all returns in the driver are 8/16 bit, so its not going to break anything
	// but the ability is there if you need 3-bytes back each time.
	//
	// if you want to add 24-bit returns, then uncomment the line below
	// return( (status_mid << 16) | (status_mid << 8) | (status_low) ); // 24-bit return, only + works right, negatives will have trouble
	return( (status_mid << 8) | (status_low) ); // 16-bit return, signs work out

} // end SPI_Prop_Send_Cmd

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// this function is used to send SPI commands to the Propeller slave processor
long SPI_Prop_Send_Cmd2(int cmd, int data, int status)
{
    long status_low, status_high, status_busy;
 
    // set CS to SPI select, select Prop SPI channel = 2 
    SPI_CS_PORT = (1 << SPI_CS1) | (0 << SPI_CS0);
 
    // send command byte and retrieve low byte of result
    status_low = SPI_WriteRead( cmd );                        
    _delay_us(SPI_PROP_CMD_DELAY_US);
 
    // send command byte and retrieve high byte of result                     
    status_high = SPI_WriteRead( data );                        
    _delay_us(SPI_PROP_CMD_DELAY_US);
    
    // send command byte and retrieve busy byte of result                     
    status_busy = SPI_WriteRead( status );
    _delay_us(SPI_PROP_CMD_DELAY_US);
 
    // set CS to SPI select channel 0 (null)
    SPI_CS_PORT = (0 << SPI_CS1) | (0 << SPI_CS0);
	//SPI_CS_PORT = (0 << SPI_CS1) | (1 << SPI_CS0);
    _delay_us(SPI_PROP_CMD_DELAY_US);
 
    return( (status_busy << 16) | (status_high << 8) | (status_low) );
 
} // end SPI_Prop_Send_Cmd2
 
// this function waits for any previous command to complete and then initiates a new command
long SPI_Prop_Send_Cmd(int cmd, int data, int status)
{
    // wait for previous command to complete
    while (SPI_Prop_Send_Cmd2(CMD_NULL, 0, 0) & 0x00ff0000)
        ;
 
    // initiate the new command
    return SPI_Prop_Send_Cmd2(cmd, data, status) & 0x0000ffff;
 
} // end SPI_Prop_Send_Cmd

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int SPI_Prop_Send_Gauge(int cmd, int gauge_first, int gauge_last)
{
	
	return SPI_Prop_Send_Cmd( cmd, gauge_first, gauge_last);

} // end SPI_Prop_Send_Two_Gauge

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
int SPI_Prop_Send_Param_List(int cmd, int *param_list[], int max_params)
{
	int status = 0;
	int i, tempint;
	char byte1,byte2 = 0;
	char buff[20];

	if (max_params != -99)
	{
		for (i=0;i<max_params;i++)
		{
			status = SPI_Prop_Send_Cmd( cmd, -99, i);
			_delay_us(SPI_PROP_CMD_DELAY_US);
			tempint = param_list[i];
			byte1 = tempint & 0xFF;
			byte2 = (tempint >> 8) & 0xFF;
			status = SPI_Prop_Send_Cmd(cmd, byte1, byte2);
			_delay_us(SPI_PROP_CMD_DELAY_US);
		}
	}
	else
	{
	    status = SPI_Prop_Send_Cmd( cmd, 0x00, max_params);
	}

	// return last status value
	return ( status );

} // end SPI_Prop_Send_Param_List*/