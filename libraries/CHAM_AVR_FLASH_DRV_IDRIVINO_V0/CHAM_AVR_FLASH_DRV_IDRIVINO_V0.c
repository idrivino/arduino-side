///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// Copyright Nurve Networks LLC 2009
// 
// Filename: CHAM_AVR_FLASH_DRV_V010.C
//
// Original Author: Andre' LaMothe
// 
// Last Modified: 8.28.09
//
// Description:  External FLASH and internal EEPROM driver. Not written yet.
//  
// Overview:
//
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
#include "CHAM_AVR_FLASH_DRV_IDRIVINO_V0.h"
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

// conditional compilation for FLASH memory variants
#if (FLASH_ROM == AT26F004)

// we could use PGM memory for these and or do it algorithmically, but this is cleaner for now
// 11 sectors [64K, 64K, 64K, 64K, 64K, 64K, 64K, 32K, 8K, 8k, 16K ]
//            |<------- (7) 64K sectors ------->|  1      2     1 



    long sector_addresses[NUM_FLASH_ROM_SECTORS] = {    0x00000,   // 64K sector 0
                                                        0x10000,   // 64K sector 1
                                                        0x20000,   // 64K sector 2
                                                        0x30000,   // 64K sector 3
                                                        0x40000,   // 64K sector 4
                                                        0x50000,   // 64K sector 5
                                                        0x60000,   // 64K sector 6
                                                        0x70000,   // 32K sector 7
                                                        0x78000,   // 8K sector 8  
                                                        0x7A000,   // 8K sector 9
                                                        0x7D000 }; // 8K sector 10
#endif


#if (FLASH_ROM == AT26DF081A)

// we could use PGM memory for these and or do it algorithmically, but this is cleaner for now
// 19 sectors [64K, 64K, 64K, ..., 64K, 64K, 64K, 64K, 16K, 8K, 8k, 32K ]
//            |<----------- (15) 64K sectors -------->| 1     2     1 


    long sector_addresses[NUM_FLASH_ROM_SECTORS] =    { 0x00000,   // 64K sector 0 
                                                        0x10000,   // 64K sector 1
                                                        0x20000,   // 64K sector 2 
                                                        0x30000,   // 64K sector 3
                                                        0x40000,   // 64K sector 4
                                                        0x50000,   // 64K sector 5 
                                                        0x60000,   // 64K sector 6
                                                        0x70000,   // 64K sector 7 
                                                        0x80000,   // 64K sector 8
                                                        0x90000,   // 64K sector 9
                                                        0xA0000,   // 64K sector 10
                                                        0xB0000,   // 64K sector 11
                                                        0xC0000,   // 64K sector 12
                                                        0xD0000,   // 64K sector 13
                                                        0xE0000,   // 64K sector 14

                                                        0xF0000,   // 16K sector 15
                                                        0xF4000,   //  8K sector 16
                                                        0xF6000,   //  8K sector 17

                                                        0xF8000 }; // 32K sector 18
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// EXTERNALS //////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PROTOTYPES /////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS  /////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// flash memory commands (not implmented yet)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Flash_Open()
{
int index; 
unsigned char spi_data;
long flash_start_addr;

// unprotect all sectors starting at address 0 ///////////////////////////////////
for (index = 0; index <  NUM_FLASH_ROM_SECTORS; index++)
    {
    // STEP 1: enable writing to chip  ////////////////////////////////////////////////////////
    // enable SPI interface

    // set CS to SPI select channel 3 (FLASH)
    SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
	spi_data = SPI_WriteRead( WRITE_ENABLE );

    // disable SPI interface
    // set CS to SPI select channel 0 (null)
    SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
	
    _delay_ms( 16 );

    // unprotect sector  ///////////////////////////////////

    // enable SPI interface

    // set CS to SPI select channel 3 (FLASH)
    SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
    spi_data = SPI_WriteRead( UNPROTECT_SECTOR );

    // get sector address
    flash_start_addr = sector_addresses[ index ];

    // write sector address, there are only a handful of sectors since each is 4-64K
    spi_data = SPI_WriteRead( flash_start_addr >> 16 );
    spi_data = SPI_WriteRead( flash_start_addr >> 8 );
    spi_data = SPI_WriteRead( flash_start_addr >> 0 );

    // disable SPI interface
    // set CS to SPI select channel 0 (null)
    SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
	
    _delay_ms( 16 );

    } // end for index

// return success always
return( 1 );

} // end Flash_Open

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int Flash_Close()
{
// In the flash close function protects all of the sectors on the flash chip.
volatile unsigned char spi_data = 0;
int index;

// protecting the sectors seems to have a side effect, we are not considering
// we need to read the data sheet more, for now, just return, and don't protect 
// all the sectors, this is overkill anyway for now ... 
return(1);

for (index = 0; index < NUM_FLASH_ROM_SECTORS; index++)
    {
	// step 1: send WRITE_ENABLE command
	
	// set CS to SPI select channel 3 (FLASH)
	SPI_CS_PORT = (1 << SPI_CS1) | (1 << SPI_CS0);
	spi_data = SPI_WriteRead( WRITE_ENABLE );
	
	// disable SPI interface
	// set CS to SPI select channel 0 (null)
	SPI_CS_PORT = (1 << SPI_CS1) | (0 << SPI_CS0);
	
	_delay_ms(100);

	// step 2: protect all sectors from sector address list

    // enable SPI interface
    // set CS to SPI select channel 3 (FLASH)
    SPI_CS_PORT = (1 << SPI_CS1) | (1 << SPI_CS0);

    spi_data = SPI_WriteRead( PROTECT_SECTOR );

    // write sector address, there are only a handful of sectors since each is 4-64K
    spi_data = SPI_WriteRead( sector_addresses[ index ] >> 16 );
    spi_data = SPI_WriteRead( sector_addresses[ index ] >> 8 );
    spi_data = SPI_WriteRead( sector_addresses[ index ] >> 0 );

    // disable SPI interface
    // set CS to SPI select channel 0 (null)
    SPI_CS_PORT = (1 << SPI_CS1) | (0 << SPI_CS0);
	
    _delay_ms(100);

    } // end for index

// return success for now
return(1);

} // end Flash_Close

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Flash_Erase( unsigned int block)
{

int i = 0; 
unsigned char spi_data;
long flash_start_addr;

// STEP 1: enable writing to chip again /////////////////////////////////////////////////////
// enable SPI interface

// set CS to SPI select channel 3 (FLASH)
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( WRITE_ENABLE );

// disable SPI interface

// set CS to SPI select channel 0 (null)
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
//_delay_ms( 16 );
__asm__("nop\n\t""nop\n\t");

// STEP 2: erase 4K block of chip
flash_start_addr = FLASH_BLOCK_TO_ADDR ( block );

// enable SPI interface

// set CS to SPI select channel 3 (FLASH)
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( BLOCK_ERASE4 );

// write block address, there are only a handful of blocks since each is 4-64K
spi_data = SPI_WriteRead( flash_start_addr >> 16 );
spi_data = SPI_WriteRead( flash_start_addr >> 8 );
spi_data = SPI_WriteRead( flash_start_addr >> 0 );

// disable SPI interface
// set CS to SPI select channel 0 (null)
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);

// erasure takes a long time... 
//_delay_ms( 200 );

// Poll Read Status Register for RDY bit 0
__asm__("nop\n\t""nop\n\t");
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( READ_STATUS );
do 
{
  _delay_ms(10);
  spi_data = SPI_WriteRead( 0x00 );
  i++;
}
while ( ((spi_data & 0x1) == 0x1) && (i<100));
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
__asm__("nop\n\t""nop\n\t");

return(1);

} // end Flash_Erase

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Flash_Erase64(unsigned int block)
{

int i = 0; 
unsigned char spi_data;
long flash_start_addr;

// STEP 1: enable writing to chip again /////////////////////////////////////////////////////
// enable SPI interface

// set CS to SPI select channel 3 (FLASH)
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( WRITE_ENABLE );

// disable SPI interface

// set CS to SPI select channel 0 (null)
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
//_delay_ms( 16 );
__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");

// STEP 2: erase 64K block of chip
flash_start_addr = FLASH_BLOCK_TO_ADDR64 ( block );

// enable SPI interface

// set CS to SPI select channel 3 (FLASH)
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( BLOCK_ERASE64 );

// write block address, there are only a handful of blocks since each is 4-64K
spi_data = SPI_WriteRead( flash_start_addr >> 16 );
spi_data = SPI_WriteRead( flash_start_addr >> 8 );
spi_data = SPI_WriteRead( flash_start_addr >> 0 );

// disable SPI interface
// set CS to SPI select channel 0 (null)
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);

// Poll Read Status Register for RDY bit 0
__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( READ_STATUS );
_delay_ms(100);
do 
{
  _delay_ms(10);
  spi_data = SPI_WriteRead( 0x00 );
  i++;
}
while ( ((spi_data & 0x1) == 0x1) && (i<100));
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");

return(1);

} // end Flash_Erase64


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Flash_ChipErase(void)
{

int i = 0; 
unsigned char spi_data;
long flash_start_addr;

// STEP 1: enable writing to chip again /////////////////////////////////////////////////////
// enable SPI interface

// set CS to SPI select channel 3 (FLASH)
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( WRITE_ENABLE );

// disable SPI interface

// set CS to SPI select channel 0 (null)
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
//_delay_ms( 16 );
__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");

// enable SPI interface

// set CS to SPI select channel 3 (FLASH)
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( CHIP_ERASE );

// disable SPI interface
// set CS to SPI select channel 0 (null)
// Chip erase starts after CS unasserted
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);

// Poll Read Status Register for RDY bit 0
__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( READ_STATUS );
_delay_ms(100);
do 
{
  _delay_ms(100);
  spi_data = SPI_WriteRead( 0x00 );
  i++;
}
while ( ((spi_data & 0x1) == 0x1) && (i<140));
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
__asm__("nop\n\t""nop\n\t");


} // end Flash_ChipErase


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Flash_Write(unsigned long flash_start_addr, unsigned char *buffer, unsigned int num_bytes)
{
int index,i = 0; 
unsigned char spi_data;

// STEP 1: enable writing to the chip again
// enable SPI interface

// set CS to SPI select channel 3 (FLASH)
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( WRITE_ENABLE );

// disable SPI interface

// set CS to SPI select channel 0 (null)
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
//_delay_ms( 16 );
__asm__("nop\n\t""nop\n\t");

// STEP 2: write buffer to address in flash //////////////////

// set CS to SPI select channel 3 (FLASH)
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);

// initiate seq program command
spi_data = SPI_WriteRead( SEQ_PROGRAM );

// write starting address byte by byte, MSB first...
spi_data = SPI_WriteRead( flash_start_addr >> 16 );
spi_data = SPI_WriteRead( flash_start_addr >> 8 );
spi_data = SPI_WriteRead( flash_start_addr >> 0 );

// write first byte
spi_data = SPI_WriteRead( buffer[ 0 ] );

// de-assert set CS to SPI select channel 0 (null)
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);

 // now write remaining data (if any)
 for (index = 1; index < num_bytes; index++)
     {

     // set CS to SPI select channel 3 (FLASH)
     SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);

     // re-issue seq program command, but NO address needed
     spi_data = SPI_WriteRead( SEQ_PROGRAM );
 
     // now data
     spi_data = SPI_WriteRead( buffer[ index ] );
	 
	 // set CS to SPI select channel 0 (null)
     SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);
	 
	 } // end for index

// disable SPI interface

// set CS to SPI select channel 0 (null)
//SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);

// finally send write disable command???

// set CS to SPI select channel 3 (FLASH)
SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
spi_data = SPI_WriteRead( WRITE_DISABLE );

// set CS to SPI select channel 0 (null)
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);

__asm__("nop\n\t""nop\n\t");

// return success
return(1);

} // end Flash_Write

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Flash_Read(unsigned long flash_start_addr, unsigned char *buffer, unsigned int num_bytes)
{

int index; 
unsigned char spi_data;

 // set CS to SPI select channel 3 (FLASH)
 SPI_CS_PORT = (1 << SPI_CS0) | (1 << SPI_CS1);
  
 // initiate read command
 spi_data = SPI_WriteRead( READ_DATA );

 // write starting address byte by byte, MSB first...
 spi_data = SPI_WriteRead( flash_start_addr >> 16 );
 spi_data = SPI_WriteRead( flash_start_addr >> 8 );
 spi_data = SPI_WriteRead( flash_start_addr >> 0 );

  // now read data 
 for (index = 0; index < num_bytes; index++)
     {
     buffer[index] = SPI_WriteRead( 0x00 );

     } // end for index

// disable SPI interface

// set CS to SPI select channel 0 (null)
SPI_CS_PORT = (0 << SPI_CS0) | (1 << SPI_CS1);

__asm__("nop\n\t""nop\n\t");

// return success
return(1);

} // end Flash_Read

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
