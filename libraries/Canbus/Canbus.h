/**
 * CAN BUS
 *
 * Copyright (c) 2010 Sukkin Pang All rights reserved.
 */

#ifndef canbus__h
#define canbus__h

#define CANSPEED_100A   0x9     // CAN speed at 100 kbps
#define CANSPEED_100B   0x7     // CAN speed at 100 kbps
#define CANSPEED_100C   0x4     // CAN speed at 100 kbps
#define CANSPEED_100D   0x3     // CAN speed at 100 kbps
#define CANSPEED_125 	7		// CAN speed at 125 kbps
#define CANSPEED_250  	3		// CAN speed at 250 kbps
#define CANSPEED_500	1		// CAN speed at 500 kbps

#define FILTER_OFF      0
#define FILTER_ON       1

#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define O2_VOLTAGE          0x14
#define THROTTLE_POS		0x4C
#define INTAKE_AIR_TEMP     0x0F
#define MANIFOLD_PRESS      0x0B
#define CONTROL_VOLTAGE     0x42
#define TIMING_ADVANCE      0x0E
#define OIL_TEMP            0x5C
#define EXHAUST_GAS_TEMP    0x78

#define PID_REQUEST         0x7DF
#define PID_REPLY			0x7E8

class CanbusClass
{
  public:

	CanbusClass();
    char init(unsigned char,unsigned char);
	char message_tx(unsigned char *buffer);
	char message_rx(unsigned char *buffer);
	char ecu_req(unsigned char pid,  char *buffer);
private:
	
};

extern CanbusClass Canbus;
//extern tCAN message;
#endif
