/* Copyright (c) 2007 Fabian Greif
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
// ----------------------------------------------------------------------------


#include <avr/io.h>
#include <util/delay.h>
#include "WConstants.h"
#include "global.h"
#include "mcp2515.h"
#include "mcp2515_defs.h"


#include "defaults.h"

int runcount = 0;

// -------------------------------------------------------------------------
// Schreibt/liest ein Byte ueber den Hardware SPI Bus

uint8_t spi_putc( uint8_t data )
{
	// put byte in send-buffer
	SPDR = data;
	
	// wait until byte was send
	while( !( SPSR & (1<<SPIF) ) )
		;
	
	return SPDR;
}

// -------------------------------------------------------------------------
void mcp2515_write_register( uint8_t adress, uint8_t data )
{
	//RESET(MCP2515_CS);
	PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
	
	spi_putc(SPI_WRITE);
	spi_putc(adress);
	spi_putc(data);
	
	//SET(MCP2515_CS);
	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
}

// -------------------------------------------------------------------------
uint8_t mcp2515_read_register(uint8_t address)
{
	uint8_t data;
	
	//RESET(MCP2515_CS);
	PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
	
	spi_putc(SPI_READ);
	spi_putc(address);
	
	data = spi_putc(0xff);	
	
	//SET(MCP2515_CS);
	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
	
	return data;
}

// -------------------------------------------------------------------------
void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data)
{
	//RESET(MCP2515_CS);
	PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
		
	spi_putc(SPI_BIT_MODIFY);
	spi_putc(address);
	spi_putc(mask);
	spi_putc(data);
	
	//SET(MCP2515_CS);
	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
}

// ----------------------------------------------------------------------------
uint8_t mcp2515_read_status(uint8_t type)
{
	uint8_t data;
	
	//RESET(MCP2515_CS);
	PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
	
	spi_putc(type);
	data = spi_putc(0xff);
	
	//SET(MCP2515_CS);
	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
	
	return data;
}

// -------------------------------------------------------------------------
uint8_t mcp2515_init(uint8_t speed,uint8_t filter)
{
	//NTSC_Term_Print("$");
	//NTSC_Term_Print("start init command");	
	
	//SET(MCP2515_CS);
	//SET_OUTPUT(MCP2515_CS);
	//PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
/*	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
	DDRC = (1 << SPI_CS1) | (1 << SPI_CS0);
	
	//_delay_ms(10000);
	
	//NTSC_Term_Print("$");
	//NTSC_Term_Print("reset dio lines");	
	
	RESET(P_SCK);
	RESET(P_MOSI);
	RESET(P_MISO);
	
	//NTSC_Term_Print("$");
	//NTSC_Term_Print("set io config");	
	
	SET_OUTPUT(P_SCK);
	SET_OUTPUT(P_MOSI);
	SET_INPUT(P_MISO);
*/	
	SET_INPUT(MCP2515_INT);
	SET(MCP2515_INT);
	
	//NTSC_Term_Print("$");
	//NTSC_Term_Print("setting SPI master interface");
	
	// active SPI master interface
	//SPCR = (1<<SPE)|(1<<MSTR) | (0<<SPR1)|(1<<SPR0);
	// Activate SPI master interface, make Chameleon FOSC setting match FOSC/16
	//###CY  Remove activation of SPI interface in CANbus shield, already done in Chameleon SPI driver
	//SPCR = (1<<SPE)|(1<<MSTR) | (0b001 & 0x03<<SPR0);
	//SPSR = 0;
	
	//NTSC_Term_Print("$");
	//NTSC_Term_Print("reset mcp2515");	
	
	// reset MCP2515 by software reset.
	// After this he is in configuration mode.
	//RESET(MCP2515_CS);
	PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
	spi_putc(SPI_RESET);
	//SET(MCP2515_CS);
	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
	
	// wait a little bit until the MCP2515 has restarted
	_delay_us(10);
	
	// load CNF1..3 Register
	//RESET(MCP2515_CS);
	PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
	spi_putc(SPI_WRITE);
	spi_putc(CNF3);
	
/*	spi_putc((1<<PHSEG21));		// Bitrate 125 kbps at 16 MHz
	spi_putc((1<<BTLMODE)|(1<<PHSEG11));
	spi_putc((1<<BRP2)|(1<<BRP1)|(1<<BRP0));

	spi_putc((1<<PHSEG21));		// Bitrate 250 kbps at 16 MHz
	spi_putc((1<<BTLMODE)|(1<<PHSEG11));
	spi_putc((1<<BRP1)|(1<<BRP0));

	spi_putc((1<<PHSEG21));		// Bitrate 500 kbps at 16 MHz
	spi_putc((1<<BTLMODE)|(1<<PHSEG11));
	spi_putc(1<<BRP0);
*/	
	//Original code:
/*	spi_putc((1<<PHSEG21));		// Bitrate set to 'speed' at 16MHz
	spi_putc((1<<BTLMODE)|(1<<PHSEG11));
	spi_putc(speed);
	//Bit Timing Calculator: http://intrepidcs.com/support/mbtime.htm
*/

	//runcount has to be global int
	//if (runcount < 4)
	if (runcount == 4000)
	{
		//TQ = 8
		if (speed == 0x09)
		{
				spi_putc(0x02);  //CNF3
				spi_putc(0x90);  //CNF2
		}
		//TQ = 10
		else if (speed == 0x07)
		{
				spi_putc(0x02);  //CNF3
				spi_putc(0xA0);  //CNF2
		}
		//TQ = 16
		else if (speed == 0x04)
		{
				spi_putc(0x05);  //CNF3
				spi_putc(0xB8);  //CNF2
		}
		//TQ = 20
		else if (speed == 0x03)
		{
				spi_putc(0x07);  //CNF3
				spi_putc(0xBA);  //CNF2
		}
		runcount++;
	}
	else
	{
		spi_putc((1<<PHSEG21));
		spi_putc((1<<BTLMODE)|(1<<PHSEG11));
	}

	spi_putc(speed); //CNF1

	// activate interrupts
	spi_putc((1<<RX1IE)|(1<<RX0IE));
	//SET(MCP2515_CS);
	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
	
	// test if we could read back the value => is the chip accessible?
	if (mcp2515_read_register(CNF1) != speed) 
	//if (mcp2515_read_register(CNF1) != 0x09) 
	{
		//SET(LED2_HIGH);

		//NTSC_Term_Print("$");
		//NTSC_Term_Print("cannot read mcp2515 chip");	
		return false;
	}
	
	//NTSC_Term_Print("$");
	//NTSC_Term_Print("mcp2515 active");	
	
	// deaktivate the RXnBF Pins (High Impedance State)
	mcp2515_write_register(BFPCTRL, 0);
	
	// set TXnRTS as inputs
	mcp2515_write_register(TXRTSCTRL, 0);
	
	// turn off filters => receive any message (default setting)
//	mcp2515_write_register(RXB0CTRL, (1<<RXM1)|(1<<RXM0));
//	mcp2515_write_register(RXB1CTRL, (1<<RXM1)|(1<<RXM0));
	
	if (filter == 0)
	{
		// Filters off except for std identifier, rollover on, no filter masks:
		mcp2515_write_register(RXB0CTRL, (0<<RXM1)|(1<<RXM0)|(1<<BUKT)); //no filters, enable rollover BUKT
	}
	else if (filter == 1)
	{
		// Receive buffer - Filter on for std identifier
		// RXM1 = 0, RXM0 = 1 (std identifier)
		mcp2515_write_register(RXB0CTRL, (0<<RXM1)|(1<<RXM0));
		mcp2515_write_register(RXB1CTRL, (0<<RXM1)|(1<<RXM0));
		// Set RX0 filter to 0x1B8, so std 11-bit identifier
		//   binary = 001|1011|1000 (11 bits)
		//   high byte = 0011 0111
		//   low byte = 000
		// RXF0 and RXF1 (Mask RXM0) apply to buffer 0
		// RXF2 thru RXF5 (Mask RXM1) apply to buffer 1
		mcp2515_write_register(RXF0SIDH, 0x37);
		mcp2515_write_register(RXF0SIDL, 0x00);
		mcp2515_write_register(RXF2SIDH, 0x37);
		mcp2515_write_register(RXF2SIDL, 0x00);
		// Set RX0 mask to all 11 bits
		mcp2515_write_register(RXM0SIDH, 0xff);
		mcp2515_write_register(RXM0SIDL, 0xe0);
		mcp2515_write_register(RXM1SIDH, 0xff);
		mcp2515_write_register(RXM1SIDL, 0xe0);
	}
	
	//clear overflow bits (optional?)
	//mcp2515_write_register(EFLG,(0<<RX1OVR)|(0<<RX0OVR));
		
	
	//listen only mode for autobaud/filtered
     if (filter == 1)
	{
        mcp2515_write_register(CANCTRL, 0x60);  //0x60 = listen only
//	SET(LED2_HIGH);
     }
	else
	{
	// set device to normal mode (r+w)
	  mcp2515_write_register(CANCTRL, 0);
	}

	//mcp2515_loopback();

	return true;
}

// Need unique cases for CANSPEED_100x settings in normal init
//-------------------------------------------------------------------------
uint8_t mcp2515_initauto()
{
    //                       x09               x7         x4             x3
	int cspeed[7] =	//{CANSPEED_100A,CANSPEED_100B,CANSPEED_100C,CANSPEED_100D,CANSPEED_125,CANSPEED_250,CANSPEED_500};
		{0x09,0x07,0x04,0x03,0x07,0x03,0x01};
	uint8_t interruptFlags = 0;
	int firstfound = -1;
	char buff[32];
	int i,j;
	tCAN message;

	runcount = 0;
	for (i=0;i<7;i++)
	{
        if(mcp2515_init(cspeed[i],1))
        {
			sprintf(buff,"Init ok @ speed index %d/",i);
			NTSC_Term_Print(buff);
			// check for bus activity
			mcp2515_write_register(CANINTF, 0);
			_delay_ms(2000);
			if(mcp2515_check_message())
			{
				if (mcp2515_get_message(&message))
				{
					for (j=0;j<8;j++)
					{
						sprintf(buff,"id %x, l %d",message.id, message.header.length);
						NTSC_Term_Print(buff);
					}
					NTSC_Term_Print("/");
				
					// determine which interrupt flags have been set
					interruptFlags = mcp2515_read_register(CANINTF);
					if(!(interruptFlags & 0x80))
					{
						// to get here we must have received something without errors
						//sprintf(buff,"Auto init found i=%d/",i);
						NTSC_Term_Print("Auto init OK/");
						if (firstfound < 0)
							firstfound = i;
						continue;
					}
				}
			}
			sprintf(buff,"Speed %d no good/",i);
			NTSC_Term_Print(buff);
		}
	}

	//Set back to normal mode
	mcp2515_write_register(CANCTRL, 0);
	return firstfound;
} //end mcp2515_initauto()

//-------------------------------------------------------------------------
void mcp2515_loopback()
{
	tCAN test_data,message;
	char buff[32];
	int i,j;
	int timeout = 0;

	test_data.id = 0x7DF;
	test_data.header.rtr = 0;
	test_data.header.length = 8;
	test_data.data[0] = 0x02;
	test_data.data[1] = 0x01;
	test_data.data[2] = 0x05;
	test_data.data[3] = 0x55;
	test_data.data[4] = 0xAA;
	test_data.data[5] = 0x80;
	test_data.data[6] = 0xFE;
	test_data.data[7] = 0xFF;

	//call this after init only, loopback mode setting
	mcp2515_write_register(CANCTRL, 0x40);
	_delay_ms(100);
	
	for (i=0;i<5;i++)
	{
		//send message
		//mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
		mcp2515_send_message(&test_data);
				
		timeout = 0;
		while (timeout < 4000)
		{
			timeout++;
			if (mcp2515_check_message())
			{
				if (mcp2515_get_message(&message))
				{
					for (j=0;j<8;j++)
					{
						sprintf(buff,"wd %d = 0x%x ",j,message.data[j]);
						NTSC_Term_Print(buff);
					}
					NTSC_Term_Print("/");
					_delay_ms(10000);
					break;
				}
			}
		}
		if (timeout >= 3999)
			NTSC_Term_Print("Loopback timed out!/");
	}

	//normal mode
	mcp2515_write_register(CANCTRL, 0);

} //end mcp2515_loopback()

// ----------------------------------------------------------------------------
// check if there are any new messages waiting

uint8_t mcp2515_check_message(void) 
{
	//clear any overflow conditions
	/*uint8_t status = mcp2515_read_status(EFLG);
	if (bit_is_set(status, 6)) 
	{
		mcp2515_bit_modify(EFLG, (1<<RX0OVR), 0);
		//NTSC_Term_Print("o");
	}
	if (bit_is_set(status, 7))
	{
		mcp2515_bit_modify(EFLG, (1<<RX1OVR), 0);
		//NTSC_Term_Print("o");
	}
	*/
	return (!IS_SET(MCP2515_INT));
}

// ----------------------------------------------------------------------------
// check if there is a free buffer to send messages

uint8_t mcp2515_check_free_buffer(void)
{
	uint8_t status = mcp2515_read_status(SPI_READ_STATUS);
	
	if ((status & 0x54) == 0x54) {
		// all buffers used
		return false;
	}
	
	return true;
}

// ----------------------------------------------------------------------------
uint8_t mcp2515_get_message(tCAN *message)
{

	// read status
	uint8_t status = mcp2515_read_status(SPI_RX_STATUS);
	uint8_t ovf_status = mcp2515_read_status(EFLG);
	uint8_t addr;
	uint8_t t;
	if (bit_is_set(status,6)) {
		// message in buffer 0
		addr = SPI_READ_RX;
		//NTSC_Term_Print(" msg in buf0");
	}
	else if (bit_is_set(status,7)) {
		// message in buffer 1
		addr = SPI_READ_RX | 0x04;
		//NTSC_Term_Print(" msg in buf1");
	}
	else {
		// Error: no message available
		//NTSC_Term_Print(" msg not in buf");
		return 0;
	}
	
	//RESET(MCP2515_CS);
	PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
	spi_putc(addr);
	
	// read id
	message->id  = (uint16_t) spi_putc(0xff) << 3;
	message->id |=            spi_putc(0xff) >> 5;
	
	spi_putc(0xff);
	spi_putc(0xff);
	
	// read DLC
	uint8_t length = spi_putc(0xff) & 0x0f;
	
	message->header.length = length;
	message->header.rtr = (bit_is_set(status, 3)) ? 1 : 0;
	
	// read data
	for (t=0;t<length;t++) {
		message->data[t] = spi_putc(0xff);
	}
	//NTSC_Term_Print(" data read");
	//SET(MCP2515_CS);
	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
	
	// clear overflow flag
	if (bit_is_set(ovf_status,6)) {
		mcp2515_bit_modify(EFLG, (1<<RX0OVR), 0);
	}
	else if (bit_is_set(ovf_status,7)) {
		mcp2515_bit_modify(EFLG, (1<<RX1OVR), 0);
	}
	//NTSC_Term_Print(" ovrflw clr, leaving mcp2515");
	
	// clear interrupt flag
	if (bit_is_set(status, 6)) {
		mcp2515_bit_modify(CANINTF, (1<<RX0IF), 0);
	}
	else if (bit_is_set(status, 7)) {
		mcp2515_bit_modify(CANINTF, (1<<RX1IF), 0);
	}
	//NTSC_Term_Print(" int clr");
	
	return (status & 0x07) + 1;
}

// ----------------------------------------------------------------------------
uint8_t mcp2515_send_message(tCAN *message)
{
	uint8_t status = mcp2515_read_status(SPI_READ_STATUS);
	
	/* Statusbyte:
	 *
	 * Bit	Function
	 *  2	TXB0CNTRL.TXREQ
	 *  4	TXB1CNTRL.TXREQ
	 *  6	TXB2CNTRL.TXREQ
	 */
	uint8_t address;
	uint8_t t;
//	SET(LED2_HIGH);
	if (bit_is_clear(status, 2)) {
		address = 0x00;
	}
	else if (bit_is_clear(status, 4)) {
		address = 0x02;
	} 
	else if (bit_is_clear(status, 6)) {
		address = 0x04;
	}
	else {
		// all buffer used => could not send message
		return 0;
	}
	
	//RESET(MCP2515_CS);
	PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
	spi_putc(SPI_WRITE_TX | address);
	
	spi_putc(message->id >> 3);
    spi_putc(message->id << 5);
	
	spi_putc(0);
	spi_putc(0);
	
	uint8_t length = message->header.length & 0x0f;
	
	if (message->header.rtr) {
		// a rtr-frame has a length, but contains no data
		spi_putc((1<<RTR) | length);
	}
	else {
		// set message length
		spi_putc(length);
		
		// data
		for (t=0;t<length;t++) {
			spi_putc(message->data[t]);
		}
	}
	//SET(MCP2515_CS);
	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
	
	_delay_us(1);
	
	// send message
	//RESET(MCP2515_CS);
	PORTC = (0 << SPI_CS1) | (0 << SPI_CS0);
	address = (address == 0) ? 1 : address;
	spi_putc(SPI_RTS | address);
	//SET(MCP2515_CS);
	PORTC = (1 << SPI_CS1) | (0 << SPI_CS0);
	
	return address;
}
