///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// Copyright Nurve Networks LLC 2009
// 
// Filename: CHAM_AVR_TWI_SPI_DRV_V010.H
//
// Original Author: Andre' LaMothe
// 
// Last Modified: 4.28.09
//
// Description: Header file for CHAM_AVR_TWI_SPI_DRV_V010.c
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

// watch for multiple inclusions
#ifndef CHAM_AVR_TWI_SPI_DRV_IDRIVINO
#define CHAM_AVR_TWI_SPI_DRV_IDRIVINO

 // support C++ compilers
 #ifdef __cplusplus
 extern "C" {
 #endif 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MACROS /////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEFINES/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

// IMPORTANT - the 328p header file changes the port names from Px0...Px7 to PORTx0...PORTx7
// so these two conditionals are now needed to support compilation switching between the 168 and 328p

#ifdef __AVR_ATmega168__

// TWI/I2C defines
// I2C clock in Hz
#define TWI_CLOCK  100000L    // initially set to 100Khz, this is the default when you call TWI_Init(...)
#define SCL_CLOCK  TWI_CLOCK

#define TWI_PORT   PORTC
#define TWI_DDR    PORTC
#define TWI_SCLK   PC0
#define TWI_SDA    PC1


// SPI defines
#define SPI_DDR     DDRB
#define SPI_PORT    PORTB
#define SPI_CS      PB2 // not used for spi addressing
#define SPI_MOSI    PB3
#define SPI_MISO    PB4
#define SPI_SCLK    PB5

#define SPI_CS_DDR  DDRC
#define SPI_CS_PORT PORTC

#define SPI_CS0     PC2
#define SPI_CS1     PC3

#endif

#ifdef __AVR_ATmega328P__

// TWI/I2C defines
// I2C clock in Hz
#define TWI_CLOCK  100000L    // initially set to 100Khz, this is the default when you call TWI_Init(...)
#define SCL_CLOCK  TWI_CLOCK


#define TWI_PORT   PORTC
#define TWI_DDR    PORTC
#define TWI_SCLK   PORTC0
#define TWI_SDA    PORTC1


// SPI defines
#define SPI_DDR     DDRB
#define SPI_PORT    PORTB
#define SPI_CS      PORTB2 // not used for spi addressing
#define SPI_MOSI    PORTB3
#define SPI_MISO    PORTB4
#define SPI_SCLK    PORTB5

#define SPI_CS_DDR  DDRC
#define SPI_CS_PORT PORTC

#define SPI_CS0     PORTC2
#define SPI_CS1     PORTC3

#endif

// SPI rate bit settings 
#define SPI_FOSC_2   0b100
#define SPI_FOSC_4   0b000
#define SPI_FOSC_8   0b101
#define SPI_FOSC_16  0b001
#define SPI_FOSC_32  0b110
#define SPI_FOSC_64  0b010
#define SPI_FOSC_128 0b011

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TIMING CONSTANTS FOR SPI COMMS - PLAY WITH THESE IF THINGS GET FUNKY
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// what rate to run the SPI at? this effects the following constants, faster rates, longer delays
#define SPI_DEFAULT_RATE    (SPI_FOSC_32)   // SPI_FOSC_32 for optimization settings -O1, -O2, -O3, -Os
                                            // SPI_FOSC_16 for no optimizations -O0


// these defines "throttle" the SPI interaction with the virtual SPI software interface on the Prop which is slow
#define SPI_PROP_SLOW_FACTOR       4      // this number scales the delays, so if you turn on heavy optimization, then make this number larger to offset the code speed
                                          // the optimizer seems to cause issues with the final code, interrupts, etc. if you do try turning on 
                                          // the optimizer, you will have to play with delays and get things to work properly 
                                          // Optimizer settings:  -O0 | use slow factor of 1 or 2
                                          //                      -O1, -O2, -O3, -Os | use slow factor of 4 or 5

#define SPI_PROP_STRING_DELAY_US  (50*SPI_PROP_SLOW_FACTOR)    // the prop needs time to react to each spi terminal print command

#define SPI_PROP_CMD_DELAY_US     ((2*SPI_PROP_SLOW_FACTOR)/2) // the prop needs time to react to each spi transition, this delay insures it can keep up
                                                               // if you optimize the Prop driver or convert to ASM then you can reduce this of course

#define SPI_PROP_DELAY_SHORT_US   (100*SPI_PROP_SLOW_FACTOR)   // this delay is used where the critial path thru the SPI/Prop is relatively short
#define SPI_PROP_DELAY_LONG_US    (250*SPI_PROP_SLOW_FACTOR)   // this delay is used where the critial path thru the SPI/Prop is relatively long

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// display devices
#define DEVICE_NTSC            0
#define DEVICE_VGA             1

// PROP SPI driver media and IO command set

// SPI commands
#define CMD_NULL               0

// NTSC commands
#define GFX_CMD_NTSC_PRINTCHAR 1 
#define GFX_CMD_NTSC_GETX      2
#define GFX_CMD_NTSC_GETY      3
#define GFX_CMD_NTSC_CLS       4

// vga commands
#define GFX_CMD_VGA_PRINTCHAR 8 
#define GFX_CMD_VGA_GETX      9
#define GFX_CMD_VGA_GETY      10
#define GFX_CMD_VGA_CLS       11

// keyboard commands
#define KEY_CMD_RESET         16
#define KEY_CMD_GOTKEY        17
#define KEY_CMD_KEY           18
#define KEY_CMD_KEYSTATE      19
#define KEY_CMD_START         20
#define KEY_CMD_STOP          21
#define KEY_CMD_PRESENT       22
  
// mouse commands
#define MOUSE_CMD_RESET       24 // resets the mouse and initializes it
#define MOUSE_CMD_ABS_X       25 // returns the absolute X-position of mouse
#define MOUSE_CMD_ABS_Y       26 // returns the absolute Y-position of mouse
#define MOUSE_CMD_ABS_Z       27 // returns the absolute Z-position of mouse
#define MOUSE_CMD_DELTA_X     28 // returns the delta X since the last mouse call
#define MOUSE_CMD_DELTA_Y     29 // returns the delta Y since the last mouse call
#define MOUSE_CMD_DELTA_Z     30 // returns the delta Z since the last mouse call
#define MOUSE_CMD_RESET_DELTA 31 // resets the mouse deltas
#define MOUSE_CMD_BUTTONS     32 // returns the mouse buttons encoded as a bit vector
#define MOUSE_CMD_START       33 // starts the mouse driver, loads a COG with it, etc.
#define MOUSE_CMD_STOP        34 // stops the mouse driver, unloads the COG its running on
#define MOUSE_CMD_PRESENT     35 // determines if mouse is present and returns type of mouse    

// general data readback commands
#define READ_CMD              36

// sound commands
#define SND_CMD_PLAYSOUNDFM   40 // plays a sound on a channel with the sent frequency at 90% volume
#define SND_CMD_STOPSOUND     41 // stops the sound of the sent channel 
#define SND_CMD_STOPALLSOUNDS 42 // stops all channels
#define SND_CMD_SETFREQ       43 // sets the frequency of a playing sound channel        
#define SND_CMD_SETVOLUME     44 // sets the volume of the playing sound channel
#define SND_CMD_RELEASESOUND  45 // for sounds with infinite duration, releases the sound and it enters the "release" portion of ADSR envelope

// propeller local 8-bit port I/O commands
#define PORT_CMD_SETDIR       48 // sets the 8-bit I/O pin directions for the port 1=output, 0=input
#define PORT_CMD_READ         49 // reads the 8-bit port pins, outputs are don't cares
#define PORT_CMD_WRITE        50 // writes the 8-bit port pins, port pins set to input ignore data 

// general register access commands, Propeller registers for the SPI driver cog can be accessed ONLY
// but, the user can leverage the counters, and even the video hardware if he wishes, most users will only
// play with the counters and route outputs/inputs to/from the Propeller local port, but these generic access
// commands model how you would access a general register based system remotely, so good example
// these commands are DANGEROUS since you can break the COG with them and require a reset, so if you are going to
// write directly to the registers, be careful.

#define REG_CMD_WRITE         56 // performs a 32-bit write to the addressed register [0..F] from the output register buffer
#define REG_CMD_READ          57 // performs a 32-bit read from the addressed register [0..F] and stores in the input register buffer 
#define REG_CMD_WRITE_BYTE    58 // write byte 0..3 of output register g_reg_out_buffer.byte[  0..3  ]
#define REG_CMD_READ_BYTE     59 // read byte 0..3 of input register g_reg_in_buffer.byte[  0..3 ]

// system commands
#define SYS_RESET             64 // resets the prop

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Commands in this range for future expansion...
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// advanced GFX commands for GFX tile engine
#define GPU_GFX_BASE_ID         192
#define GPU_GFX_NUM_COMMANDS    26

#define GPU_GFX_ONE_GAUGE       (0+GPU_GFX_BASE_ID)
#define GPU_GFX_TWO_GAUGE       (1+GPU_GFX_BASE_ID)
#define GPU_GFX_FOUR_GAUGE      (2+GPU_GFX_BASE_ID)
#define GPU_GFX_SIX_GAUGE_1     (3+GPU_GFX_BASE_ID)
#define GPU_GFX_SIX_GAUGE_2     (4+GPU_GFX_BASE_ID)
#define GPU_GFX_PARAM_LIST      (5+GPU_GFX_BASE_ID)
#define GPU_GFX_TIMERXY         (6+GPU_GFX_BASE_ID)
#define GPU_GFX_TIMER_SETXY     (7+GPU_GFX_BASE_ID)
#define GPU_GFX_CONFIG_SET      (8+GPU_GFX_BASE_ID)
#define GPU_GFX_PROCEDE_PGM     (9+GPU_GFX_BASE_ID)
#define GPU_GFX_FIRMWARE_PGM    (10+GPU_GFX_BASE_ID)
#define GPU_GFX_DATA_LOG        (11+GPU_GFX_BASE_ID)
#define GPU_GFX_CAN_SNIFFER     (12+GPU_GFX_BASE_ID)
#define GPU_GFX_V1              (13+GPU_GFX_BASE_ID)
#define GPU_GFX_REFRESH_ACTIVE  (14+GPU_GFX_BASE_ID)
#define GPU_GFX_TERMINAL_PRINT  (15+GPU_GFX_BASE_ID)
#define GPU_GFX_BOTTOM_MENU     (16+GPU_GFX_BASE_ID)
#define GPU_GFX_HIGHLIGHT       (17+GPU_GFX_BASE_ID)
#define GPU_GFX_CLS_MAIN        (18+GPU_GFX_BASE_ID)
#define GPU_GFX_SPLASH_IN       (19+GPU_GFX_BASE_ID)
#define GPU_GFX_SPLASH_OUT      (20+GPU_GFX_BASE_ID)
#define GPU_GFX_DRAW_BOX        (21+GPU_GFX_BASE_ID)
#define GPU_GFX_PRINT_XY        (22+GPU_GFX_BASE_ID)
#define GPU_GFX_PRINT_SETXY     (23+GPU_GFX_BASE_ID)
#define GPU_GFX_TIMERFATS       (24+GPU_GFX_BASE_ID)
#define GPU_GFX_TIMER_SETFATS   (25+GPU_GFX_BASE_ID)
   
/*#define  GPU_GFX_SUBFUNC_STATUS_R  (0+GPU_GFX_BASE_ID) // Reads the status of the GPU, Writes the GPU Sub-Function register and issues a high level command like copy, fill, etc.                                                      
#define  GPU_GFX_SUBFUNC_STATUS_W  (1+GPU_GFX_BASE_ID) // Writes status of the GPU, Writes the GPU Sub-Function register and issues a high level command like copy, fill, etc.
   
    // sub-function constants that are executed when the GPU_GFX_SUBFUNC_STATUS_W command is issued 
    #define GPU_GFX_SUBFUNC_COPYMEM16 0 // Copies numbytes from src -> dest in wordsize chunks
    #define GPU_GFX_SUBFUNC_FILLMEM16 1 // Fills memory with data16, 2 bytes at a time

    #define GPU_GFX_SUBFUNC_COPYMEM8  2 // Copies numbytes from src -> dest in byte size chunks
    #define GPU_GFX_SUBFUNC_FILLMEM8  3 // Fills memory with low byte of data16, 1 bytes at a time 

// normal commands
#define  GPU_GFX_TILE_MAP_R        (2+GPU_GFX_BASE_ID) // Reads 16-bit tile map ptr which points to the current tile map displayed.
#define  GPU_GFX_TILE_MAP_W        (3+GPU_GFX_BASE_ID) // Writes 16-bit tile map ptr which points to the current tile map displayed.                                                                         
   
#define  GPU_GFX_DISPLAY_PTR_R     (4+GPU_GFX_BASE_ID) // Reads 16-bit tile map ptr which points to the current display ptr into the tile map, low level print functions use this pointer.
#define  GPU_GFX_DISPLAY_PTR_W     (5+GPU_GFX_BASE_ID) // Writes 16-bit tile map ptr which points to the current display ptr into the tile map, low level print functions use this pointer.                                                                
   
#define  GPU_GFX_TERM_PTR_R        (6+GPU_GFX_BASE_ID) // Reads 16-bit tile map ptr which points to the current terminal ptr into the tile map, terminal level print functions use this pointer.
#define  GPU_GFX_TERM_PTR_W        (7+GPU_GFX_BASE_ID) // Writes 16-bit tile map ptr which points to the current terminal ptr into the tile map, terminal level print functions use this pointer.                                                                
   
#define  GPU_GFX_BITMAP_R          (8+GPU_GFX_BASE_ID) // Reads 16-bit tile bitmaps ptr which points to the bitmaps indexed by the tilemap. 
#define  GPU_GFX_BITMAP_W          (9+GPU_GFX_BASE_ID) // Reads 16-bit tile bitmaps ptr which points to the bitmaps indexed by the tilemap.                                                                         
   
#define  GPU_GFX_PALETTE_R         (10+GPU_GFX_BASE_ID) // Reads 16-bit palette array ptr which points to the palettes in use for the tilemap.
#define  GPU_GFX_PALETTE_W         (11+GPU_GFX_BASE_ID) // Writes 16-bit palette array ptr which points to the palettes in use for the tilemap.                                                                         
   
#define  GPU_GFX_TOP_OVERSCAN_COL_R (12+GPU_GFX_BASE_ID) // Reads top overscan color drawn on the screen.
#define  GPU_GFX_TOP_OVERSCAN_COL_W (13+GPU_GFX_BASE_ID) // Writes top overscan color drawn on the screen.
   
#define  GPU_GFX_BOT_OVERSCAN_COL_R (14+GPU_GFX_BASE_ID) // Reads top overscan color drawn on the screen.
#define  GPU_GFX_BOT_OVERSCAN_COL_W (15+GPU_GFX_BASE_ID) // Writes top overscan color drawn on the screen.
   
#define  GPU_GFX_HSCROLL_FINE_R  (16+GPU_GFX_BASE_ID)  // Reads current fine horizontal scroll register (0..7) NOTE: (functionality NOT implemented yet) 
#define  GPU_GFX_HSCROLL_FINE_W  (17+GPU_GFX_BASE_ID)  // Writes current fine horizontal scroll register (0..7) NOTE: (functionality NOT implemented yet)
   
#define  GPU_GFX_VSCROLL_FINE_R  (18+GPU_GFX_BASE_ID) // Reads current fine vertical scroll register (0..7)
#define  GPU_GFX_VSCROLL_FINE_W  (19+GPU_GFX_BASE_ID) // Writes current fine vertical scroll register (0..7)
   
#define  GPU_GFX_SCREEN_WIDTH_R  (20+GPU_GFX_BASE_ID) // Reads screen width value, 0=16 tiles, 1=32 tiles, 2=64 tiles, etc.
#define  GPU_GFX_SCREEN_WIDTH_W  (21+GPU_GFX_BASE_ID) // Writes screen width value, 0=16 tiles, 1=32 tiles, 2=64 tiles, etc.
   
#define  GPU_GFX_SRC_ADDR_R      (22+GPU_GFX_BASE_ID) // Reads 16-bit source address for GPU operations.
#define  GPU_GFX_SRC_ADDR_W      (23+GPU_GFX_BASE_ID) // Writes 16-bit source address for GPU operations.
                       
#define  GPU_GFX_DEST_ADDR_R     (24+GPU_GFX_BASE_ID) // Reads 16-bit destination address for GPU operations.
#define  GPU_GFX_DEST_ADDR_W     (25+GPU_GFX_BASE_ID) // Writes 16-bit destination address for GPU operations.
   
#define  GPU_GFX_NUM_BYTES_R     (26+GPU_GFX_BASE_ID) // Reads 16-bit number representing the number of bytes for a GPU operation Sub-Function.
#define  GPU_GFX_NUM_BYTES_W     (27+GPU_GFX_BASE_ID) // Writes 16-bit number representing the number of bytes for a GPU operation Sub-Function.
   
#define  GPU_GFX_DATA_R          (28+GPU_GFX_BASE_ID) // Reads 16-bit data word uses for GPU operations or memory access operations.
#define  GPU_GFX_DATA_W          (29+GPU_GFX_BASE_ID) // Writes 16-bit data word uses for GPU operations or memory access operations.
   
#define  GPU_GFX_RAM_PORT8_R     (30+GPU_GFX_BASE_ID) // Reads 8-bit data pointed to by the currently addressed memory location in the GPU pointed to by the src_addr_low|hi.
                                                        // After the operation, the src_addr_ptr is incremented as per the GPU configuration.
   
#define  GPU_GFX_RAM_PORT8_W     (31+GPU_GFX_BASE_ID) // Writes 8-bit data pointed to by the currently addressed memory location in the GPU pointed to by the src_addr_low|hi.
                                                        // After the operation, the src_add_rptr are incremented as per the GPU configuration.
   
#define  GPU_GFX_RAM_PORT16_R    (32+GPU_GFX_BASE_ID) // Reads 16-bit data pointed to by the currently addressed memory location in the GPU pointed to by the src_addr_low|hi.
                                                        // After the operation, the src_addr_ptr is incremented as per the GPU configuration.
   
#define  GPU_GFX_RAM_PORT16_W    (33+GPU_GFX_BASE_ID) // Writes 16-bit data pointed to by the currently addressed memory location in the GPU pointed to by the src_addr_low|hi.
                                                        // After the operation, the src_add_rptr are incremented as per the GPU configuration.
   
#define  GPU_GFX_RASTER_LINE_R   (34+GPU_GFX_BASE_ID) // Reads the current raster line being drawn from 0..191, or whatever the GPU's line resolution is set to.                             
   
// gpu configuration registers
#define  GPU_GFX_SET_AUTO_INC_R  (35+GPU_GFX_BASE_ID) // Reads current memory auto increment setting for read/write operations lower 4-bits (0..15), default 0
#define  GPU_GFX_SET_AUTO_INC_W  (36+GPU_GFX_BASE_ID) // Writes the current memory auto increment setting for read/write operations lower 4-bits (0..15), default 0
*/

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

//TWI prototypes
int TWI_Init(long twi_clock);
void TWI_Error();
void TWI_Status();
void TWI_Success(int code);
char TWI_Action(unsigned char command);

// SPI prototypes
int SPI_Init(unsigned char spi_rate);
int SPI_Set_Speed(unsigned char spi_rate);
unsigned char SPI_Write(unsigned char data8);
unsigned char SPI_Read(unsigned char data8);
unsigned char SPI_WriteRead(unsigned char data8);
unsigned char SPI_Send_Recv_Byte(unsigned char data8);
 
long SPI_Prop_Send_Cmd(int cmd, int data, int status);
int SPI_Prop_Print_String(char *string);
int SPI_Prop_Print_CAN_String(char *string);
int SPI_Prop_Send_Gauge(int cmd, int gauge_first, int gauge_last);
//int SPI_Prop_Send_Param_List(int cmd, int *param_list[], int max_params);

// end support for C++ compilers
 #ifdef __cplusplus
 }
 #endif 

// end multiple inclusions 
#endif
