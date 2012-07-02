#ifndef	DEFAULTS_H
#define	DEFAULTS_H

#define	P_MOSI	B,3
#define	P_MISO	B,4
#define	P_SCK	B,5

//#define	MCP2515_CS			D,3	// Rev A
//#define	MCP2515_CS			B,2 // Rev B
#define MCP2515_CS          C,3  //not used on iDrivino
#define	MCP2515_INT			D,2
#define LED2_HIGH			B,0
#define LED2_LOW			B,0

#define SPI_CS0     PORTC2
#define SPI_CS1     PORTC3

#endif	// DEFAULTS_H
