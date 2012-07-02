///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// Copyright Nurve Networks LLC 2009
// 
// Filename: CHAM_AVR_NTSC_DRV_V010.C
//
// Original Author: Andre' LaMothe
// 
// Last Modified: 8.28.09
//
// Description: NTSC API for standard "terminal" mode operations
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
#include "CHAM_AVR_TWI_SPI_DRV_IDRIVINO_V0.h"
#include "CHAM_AVR_NTSC_DRV_IDRIVINO_V0.h"


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
int i;

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
// NTSC terminal support
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int NTSC_Term_Print(char *string)
{
// this prints a string to the NTSC terminal (all drivers Default1,2, etc.)
//SPI_Prop_Print_String(DEVICE_NTSC, string);
SPI_Prop_Print_String(string);

// wait a bit, driver takes time to respond...
//_delay_us(SPI_PROP_DELAY_SHORT_US);

// return success
return(1);

} // end NTSC_Term_Print

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*int NTSC_Term_Char(char ch)
{
// this prints a single character to the NTSC terminal (all drivers Default1,2, etc.)
// also supports translation commands supported by the NTSC terminals 
//     $00 = clear screen
//     $01 = home
//     $08 = backspace
//     $09 = tab (8 spaces per)
//     $0A = set X position (X follows)
//     $0B = set Y position (Y follows)
//     $0C = set color (color follows)
//     $0D = return
// other characters are not translated, but simply printed to the terminal

SPI_Prop_Send_Cmd(GFX_CMD_NTSC_PRINTCHAR, ch, 0x00);

// wait a bit, driver takes time to respond...
//_delay_us(SPI_PROP_DELAY_SHORT_US);

// return success
return(1);

} // end NTSC_Term_Char
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int NTSC_SetXY(int x, int y)
{
// sets the x,y (column, row) position of the terminal cursor
// TODO: might needs some delays between commands

// send driver set x command, [$0A, x 0..39]           
SPI_Prop_Send_Cmd(GFX_CMD_NTSC_PRINTCHAR, 0x0A, 0x00);
//_delay_us(SPI_PROP_DELAY_SHORT_US);
SPI_Prop_Send_Cmd(GFX_CMD_NTSC_PRINTCHAR, x, 0x00);
//_delay_us(SPI_PROP_DELAY_LONG_US);

// send driver set x command, [$0B, y 0..29]                           
SPI_Prop_Send_Cmd(GFX_CMD_NTSC_PRINTCHAR, 0x0B, 0x00);
//_delay_us(SPI_PROP_DELAY_SHORT_US);
SPI_Prop_Send_Cmd(GFX_CMD_NTSC_PRINTCHAR, y, 0x00);
//_delay_us(SPI_PROP_DELAY_LONG_US);

// return success
return(1);

} // end NTSC_SetXY

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int NTSC_GetXY(int *x, int *y)
{
// gets the x,y (column, row) of the terminal cursor

// send driver get x command
SPI_Prop_Send_Cmd(GFX_CMD_NTSC_GETX, 0x00, 0x00);

//_delay_us(SPI_PROP_DELAY_LONG_US);

// now read the data back, note 0x00, 0x00 are dummy data, mask lower 8-bits
*x = SPI_Prop_Send_Cmd( READ_CMD, 0x00, 0x00) & 0xFF;

//_delay_us(SPI_PROP_DELAY_LONG_US);

SPI_Prop_Send_Cmd(GFX_CMD_NTSC_GETY, 0x00, 0x00);

//_delay_us(SPI_PROP_DELAY_LONG_US);

// now read the data back, note 0x00, 0x00 are dummy data, mask lower 8-bits
*y = SPI_Prop_Send_Cmd( READ_CMD, 0x00, 0x00) & 0xFF;

// wait a bit, driver takes time to respond...
//_delay_us(SPI_PROP_DELAY_LONG_US);

// return success
return(1);

} // end NTSC_GetXY

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int NTSC_ClearScreen(void)
{
// clears the screen of the terminal and fills with spaces

SPI_Prop_Send_Cmd(GFX_CMD_NTSC_CLS, 0x00, 0x00);  
//_delay_ms(1);
 
// return success
return(1);

} // end NTSC_ClearScreen

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int NTSC_Color(int col)
{
// sets the "color" of the character, means different things under different drivers
            
// send driver set color command, [$0C, col 0..7]                           
SPI_Prop_Send_Cmd(GFX_CMD_NTSC_PRINTCHAR, 0x0C, 0x00);
//_delay_us(SPI_PROP_DELAY_SHORT_US);
SPI_Prop_Send_Cmd(GFX_CMD_NTSC_PRINTCHAR, col & 0x07, 0x00);

// wait a bit, driver takes time to respond...
//_delay_us(SPI_PROP_DELAY_LONG_US);

// return success
return(1);

} // end NTSC_Color

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_ClearMainWindow(void)
{
	// clears the main window of the iDrivino (not the bottom menu)

	SPI_Prop_Send_Cmd(GPU_GFX_CLS_MAIN, 0x00, 0x00);  
	
	// return success
	return(1);

} // end NTSC_ClearScreen

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Draw/redraw gauges: GFX_One_Gauge(gauge_type,-99)
//  Update gauges:      GFX_One_Gauge(-99,gauge_position_to_update)        "step 1"
//                      GFX_One_Gauge(new_gauge_valMSB,new_gauge_valLSB)  "step 2"
int GFX_One_Gauge(int param1, int param2)
{

	//SPI_Prop_Send_Cmd(GPU_GFX_ONE_GAUGE, gauge_1, 0x00);
	
	if (param2 != -99)
	{
		SPI_Prop_Send_Cmd( GPU_GFX_ONE_GAUGE, -99, param2);
		_delay_us(SPI_PROP_CMD_DELAY_US);
		SPI_Prop_Send_Cmd( GPU_GFX_ONE_GAUGE, (param1 & 0xFF), ((param1 >> 8) & 0xFF));
	}
	else
	{
		SPI_Prop_Send_Cmd( GPU_GFX_ONE_GAUGE, param1, param2);
	}

	// return success
	return(1);

} // end GFX_One_Gauge

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Draw/redraw gauges: GFX_Two_Gauge(gauge_type,-99)
//  Update gauges:      GFX_Two_Gauge(-99,gauge_position_to_update)        "step 1"
//                      GFX_Two_Gauge(new_gauge_valMSB,new_gauge_valLSB)  "step 2"
int GFX_Two_Gauge(int param1, int param2)
{
	//SPI_Prop_Send_Cmd(GPU_GFX_TWO_GAUGE, gauge_1_2&0xFF, 0x00);
	if (param2 != -99)
	{
		SPI_Prop_Send_Cmd( GPU_GFX_TWO_GAUGE, -99, param2);
		_delay_us(SPI_PROP_CMD_DELAY_US);
		SPI_Prop_Send_Cmd( GPU_GFX_TWO_GAUGE, (param1 & 0xFF), ((param1 >> 8) & 0xFF));
	}
	else
	{
		SPI_Prop_Send_Cmd( GPU_GFX_TWO_GAUGE, param1, param2);
	}

	// return success
	return(1);

} // end GFX_Two_Gauge

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_Four_Gauge(int param1, int param2)
{
	if (param2 != -99)
	{
		SPI_Prop_Send_Cmd( GPU_GFX_FOUR_GAUGE, -99, param2);
		_delay_us(SPI_PROP_CMD_DELAY_US);
		//SPI_Prop_Send_Cmd(GPU_GFX_FOUR_GAUGE, gauge_1_2&0xFF, gauge_3_4&0xFF);
		SPI_Prop_Send_Cmd( GPU_GFX_FOUR_GAUGE, (param1 & 0xFF), ((param1 >> 8) & 0xFF));
	}
	else
	{
		SPI_Prop_Send_Cmd( GPU_GFX_FOUR_GAUGE, ((param1 >> 8) & 0xFF), param2);
		_delay_us(SPI_PROP_CMD_DELAY_US);
		SPI_Prop_Send_Cmd( GPU_GFX_FOUR_GAUGE, (param1 & 0xFF), param2+1);
	}
	
	// return success
	return(1);

} // end GFX_Four_Gauge

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
int GFX_Six_Gauge1(int gauge_1_2, int gauge_3_4)
{
	SPI_Prop_Send_Cmd(GPU_GFX_SIX_GAUGE_1, gauge_1_2&0xFF, gauge_3_4&0xFF);

	// return success
	return(1);

} // end GFX_Six_Gauge

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_Six_Gauge2(int gauge_5_6)
{
	SPI_Prop_Send_Cmd(GPU_GFX_SIX_GAUGE_2, gauge_5_6&0xFF, 0x00);

	// return success
	return(1);

} // end GFX_Six_Gauge

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_Update_Gauge(int gauge_position, int param_val)
{
	
	SPI_Prop_Send_Cmd(GPU_GFX_REFRESH_ACTIVE, param_val, gauge_position );

	// return success
	return(1);

} // end GFX_Six_Gauge

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_Send_Active_Screen(int active_screen)
{

	SPI_Prop_Send_Cmd(GPU_GFX_ACTIVE_SCREEN, active_screen, 0x00);

	// return success
	return(1);

} // end GFX_Refresh_Active
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_Param_List(int param_list[], int max_params)
{
	int status = 0;
	int i, tempint;
	char byte1,byte2 = 0;
	char buff[20];

	if (max_params != -99)
	{
		for (i=0;i<max_params;i++)
		{
			status = SPI_Prop_Send_Cmd(GPU_GFX_PARAM_LIST, -99, i);
			_delay_us(SPI_PROP_CMD_DELAY_US);
			tempint = param_list[i];
			byte1 = tempint & 0xFF;
			byte2 = (tempint >> 8) & 0xFF;
			status = SPI_Prop_Send_Cmd(GPU_GFX_PARAM_LIST, byte1, byte2);
			_delay_us(SPI_PROP_CMD_DELAY_US);
		}
	}
	else
	{
	    status = SPI_Prop_Send_Cmd(GPU_GFX_PARAM_LIST, 0x00, max_params);
	}

	// return last status value
	return ( status );

} // end GFX_Param_List

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
int GFX_Can_Sniffer(char *string)
{
		
	SPI_Prop_Print_CAN_String(string);
	
	// return success
	return(1);

} // end GFX_Can_Sniffer
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_SetTimerXY(unsigned short start_speed, unsigned short stop_speed)
{
	SPI_Prop_Send_Cmd( GPU_GFX_TIMER_SETXY, start_speed, stop_speed);
	
	// return success
	return(1);

} // end GFX_SetTimerXY

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_TimerXY(int param1, int param2)
{
	if (param2 != -99)
	{
		SPI_Prop_Send_Cmd( GPU_GFX_TIMERXY, -99, param2);
		_delay_us(SPI_PROP_CMD_DELAY_US);
		SPI_Prop_Send_Cmd( GPU_GFX_TIMERXY, (param1 & 0xFF), ((param1 >> 8) & 0xFF));
	}
	else
	{
		SPI_Prop_Send_Cmd( GPU_GFX_TIMERXY, param1, param2);
	}	
	
	// return success
	return(1);

} // end GFX_TimerXY

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_SetTimerFATS(unsigned short start_rpm, unsigned short stop_rpm)
{
	SPI_Prop_Send_Cmd( GPU_GFX_TIMER_SETFATS, start_rpm, stop_rpm);
	
	// return success
	return(1);

} // end GFX_SetTimerFATS

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_TimerFATS(int param1, int param2)
{
	if (param2 != -99)
	{
		SPI_Prop_Send_Cmd( GPU_GFX_TIMERFATS, -99, param2);
		_delay_us(SPI_PROP_CMD_DELAY_US);
		SPI_Prop_Send_Cmd( GPU_GFX_TIMERFATS, (param1 & 0xFF), ((param1 >> 8) & 0xFF));
	}
	else
	{
		SPI_Prop_Send_Cmd( GPU_GFX_TIMERFATS, param1, param2);
	}	
	
	// return success
	return(1);

} // end GFX_TimerFATS

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_Bottom_Menu(int param1, int param2)
{
	
	//Map # update
	if (param1 == -1)
	{
		SPI_Prop_Send_Cmd( GPU_GFX_BOTTOM_MENU, -1, param2);
	}
	//AT boost update
	else if (param1 == -99)
	{
		SPI_Prop_Send_Cmd( GPU_GFX_BOTTOM_MENU, (param2 & 0xFF), ((param2 >> 8) & 0xFF));
	}
	//Tab highlight, param2 == -99
	else
	{
		SPI_Prop_Send_Cmd( GPU_GFX_BOTTOM_MENU, param1, param2);
	}

	// return success
	return(1);

} // end GFX_Bottom_Menu

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_Highlight(int screen, int highlight_item, int extra_param)
{
		
	if (extra_param == -99)
	{
		//Bottom bar update
		GFX_Bottom_Menu(screen,-99);
		SPI_Prop_Send_Cmd( GPU_GFX_HIGHLIGHT, screen, -99);
	}
	else
	{
		SPI_Prop_Send_Cmd( GPU_GFX_HIGHLIGHT, screen, highlight_item);
	}
	
	// return success
	return(1);

} // end GFX_Highlight

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
int GFX_Splash_In()
{
		
	SPI_Prop_Send_Cmd(GPU_GFX_SPLASH_IN,0,0);
	
	// return success
	return(1);

} // end GFX_Splash_In

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX_Splash_Out()
{
		
	SPI_Prop_Send_Cmd(GPU_GFX_SPLASH_OUT,0,0);
	
	// return success
	return(1);

} // end GFX_Splash_Out
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Square button with text label centered
// x,y are lower left corner of button (pixel location)
// width,height are button size in pixels
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int GFX_Box(int x, int y, int width, int height)
{

	//Draw box
	SPI_Prop_Send_Cmd(GPU_GFX_DRAW_BOX,x,y);
	_delay_us(SPI_PROP_CMD_DELAY_US);
	SPI_Prop_Send_Cmd(GPU_GFX_DRAW_BOX,width,height);
	
	return(1);

} // end GFX_Button

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sets X,Y location to print text
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int GFX_Print_SetXY(int x, int y)
{
	SPI_Prop_Send_Cmd(GPU_GFX_PRINT_SETXY,x,y);
	
	return (1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prints text at last X,Y location specified by SetXY
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int GFX_Print_XY(char *string, char color)
{
	// print the string to Prop over SPI channel

	static int   data   = 0, 
				 status = 0;

	// print the string until NULL reached
	while ((data = *string++) != NULL)
	{
		status = SPI_Prop_Send_Cmd( GPU_GFX_PRINT_XY, data, color);
		_delay_us( SPI_PROP_STRING_DELAY_US );
	} 

	// return last status value
	return ( status );
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prints char at last X,Y location specified by SetXY
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int GFX_Print_Hex_XY(char ch, char color)
{
	SPI_Prop_Send_Cmd( GPU_GFX_PRINT_XY, ch, color);

	return (1);
}