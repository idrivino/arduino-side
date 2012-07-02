/**
 * 
 *
 * Copyright (c) 2008-2009  All rights reserved.
 */
#include "WConstants.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pins_arduino.h"
#include <inttypes.h>
#include "global.h"
#include "mcp2515.h"
#include "defaults.h"
#include "Canbus.h"




/* C++ wrapper */
CanbusClass::CanbusClass() {

 
}
char CanbusClass::message_rx(unsigned char *buffer) {
		tCAN message;
	
	//char buff[20];
	char max_rx;
	
	if (mcp2515_check_message()) 
	{
	    // Lese die Nachricht aus dem Puffern des MCP2515
		if (mcp2515_get_message(&message)) 
		{
		//	print_can_message(&message);
		//	PRINT("\n");
/*				buffer[0] = message.data[0];
			buffer[1] = message.data[1];
			buffer[2] = message.data[2];
			buffer[3] = message.data[3];
			buffer[4] = message.data[4];
			buffer[5] = message.data[5];
			buffer[6] = message.data[6];
			buffer[7] = message.data[7];								
*/
//				buffer[] = message[];
//				buffer[] = message[];
//				buffer[] = message[];
//				buffer[] = message[];
			buffer[0] = (unsigned char)message.id;
			//sprintf(buff,"%x",buffer[0]);
			buffer[1] = (unsigned char)(message.id >> 8);
			//sprintf(buff," %x",buffer[1]);
			buffer[2] = (unsigned char)message.header.length;
			//sprintf(buff," %x",buffer[2]);
			if (buffer[2] > 16)
			{
				max_rx = 16;
			}
			else
			{
				max_rx = buffer[2];
			}
			for (int i=0;i<max_rx;i++)
			//for (int i=0;i<8;i++)
			{
				buffer[3+i] = message.data[i];
				//sprintf(buff," %x",buffer[3+i]);
			}
			//NTSC_Term_Print("./");
		}
		else 
		{
		//	PRINT("Kann die Nachricht nicht auslesen\n\n");
		  return 0;
		}
	}
  return 1;
}

char CanbusClass::message_tx(unsigned char *buffer) {
	tCAN message;


	// einige Testwerte
	/*message.id = 0x7DF;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = 0x02;
	message.data[1] = 0x01;
	message.data[2] = 0x05;
	message.data[3] = 0x00;
	message.data[4] = 0x00;
	message.data[5] = 0x00;
	message.data[6] = 0x00;
	message.data[7] = 0x00;						
	*/
	
	/* Buffer (8-bits per word)
	0,1 = 15..11 (unused), 10..0 11-bit ID
	  = |15  14  13  12  11  10  9  8| 7  6  5  4  3  2  1  0|
	    | (unused)         |    11-bit identifier            |
		|   buffer[1]                |      buffer[0]        |
	2 = 7..0  data length n
	3 = 7..0  1st data word
	n = 7..0  nth data word
	*/
	
	message.id = (buffer[1] << 8) + buffer[0];
	message.header.rtr = 0;  //Remote Transmission Request (RTR), not used so set to 0
	message.header.length = buffer[2];
	for (int i=0;i<message.header.length;i++)
	//for (int i=0;i<8;i++)
	{
		message.data[i] = buffer[3+i];
	}
	
//	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), (1<<REQOP1));	
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
		
	if (mcp2515_send_message(&message)) {
		//	SET(LED2_HIGH);
		return 1;	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}

char CanbusClass::ecu_req(unsigned char pid, char *buffer) 
{
	tCAN message;
	float engine_data;
	int timeout = 0;
	char message_ok = 0;
	
	// Prepare message
	message.id = PID_REQUEST;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = 0x02;
	message.data[1] = 0x01;
	message.data[2] = pid;
	message.data[3] = 0x00;
	message.data[4] = 0x00;
	message.data[5] = 0x00;
	message.data[6] = 0x00;
	message.data[7] = 0x00;						
	
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
//		SET(LED2_HIGH);	
	if (mcp2515_send_message(&message)) 
	{	
	}
	
	while(timeout < 4000)
	{
		timeout++;
		if (mcp2515_check_message()) 
		{
			if (mcp2515_get_message(&message)) 
			{
				if((message.id == PID_REPLY) && (message.data[2] == pid))	// Check message is the reply and its the right PID
				{
					switch (message.data[2])
					{   /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
						case ENGINE_RPM:  			//   ((A*256)+B)/4    [RPM]
						engine_data =  ((message.data[3]*256) + message.data[4])/4;
						sprintf(buffer,"%d",(int) engine_data);
						break;
				
						case ENGINE_COOLANT_TEMP: 	// 	A-40			  [degree C]
						engine_data =  message.data[3] - 40;
						sprintf(buffer,"%d",(int) engine_data);
						break;
				
						case VEHICLE_SPEED: 		// A				  [km]
						engine_data =  message.data[3];
						sprintf(buffer,"%d",(int) engine_data);
						break;

						case MAF_SENSOR:   			// ((256*A)+B) / 100  [g/s]
						engine_data =  ((message.data[3]*256) + message.data[4]);
						sprintf(buffer,"%d",(int) engine_data);
						break;

						case O2_VOLTAGE:    		// A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
						engine_data = message.data[3]*0.005*100;
						sprintf(buffer,"%d",(int) engine_data);
						break;
				
						case THROTTLE_POS:				// Throttle Position
						engine_data = (message.data[3]*100)/255;
						sprintf(buffer,"%d",(int) engine_data);
						break;
				
						case INTAKE_AIR_TEMP:			// Intake air temperature
						engine_data = message.data[3]-40;
						sprintf(buffer,"%d",(int) engine_data);
						break;
				
						case MANIFOLD_PRESS:			// Intake manifold pressure
						engine_data = message.data[3];
						sprintf(buffer,"%d",(int) engine_data);
						break;
						
						case TIMING_ADVANCE:			// Timing advance
						engine_data = ((message.data[3]/2)-64);
						sprintf(buffer,"%d",(int) engine_data);
						break;
						
						case CONTROL_VOLTAGE:			// Control module voltage
						engine_data = ((message.data[3]*256)+message.data[4])/10;
						sprintf(buffer,"%d",(int) engine_data);
						break;
						
						case EXHAUST_GAS_TEMP:          // EGT
						engine_data = message.data[3];
						sprintf(buffer,"%d",(int) engine_data);
						break;
						
						default:
						break;
					}
					message_ok = 1;
				}

			}
		}
		if(message_ok == 1) return 1;
	}

 	return 0;
}






char CanbusClass::init(unsigned char speed, unsigned char filter) {

  if (speed == 0)
	return mcp2515_initauto();
  else
	return mcp2515_init(speed,filter);
 
}

CanbusClass Canbus;
