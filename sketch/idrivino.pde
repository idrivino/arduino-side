///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// Filename: idrivino.pde
//
// Original Author: Constant Yu
// 
// Last Modified: 1.4.2013
//
// Description: iDrivino System
//  
// Overview: Complete iDrivino system software
//
// Revision: A
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDES ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// __AVR_ATmega328P__ , __AVR_ATmega168__
#define __AVR_ATmega328P__ 

// include everything 
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
#include <avr/wdt.h>
#include <Canbus.h>
#include <byteordering.h>
/*#include <fat.h>
#include <FAT16.h>
#include <fat_config.h>
#include <partition.h>
#include <partition_config.h>
#include <sd-reader_config.h>
#include <sd_raw.h>
#include <sd_raw_config.h>*/


// include headers required for demo, notice the use of <> brackets for the arduino version since it searches for headers
// differently than AVRStudio
#include <CHAM_AVR_SYSTEM_IDRIVINO_V0.h>          // you need this one always
//#include <CHAM_AVR_UART_DRV_IDRIVINO_V0.h>        // UART driver
#include <CHAM_AVR_TWI_SPI_DRV_IDRIVINO_V0.h>     // Custom iDrivino SPI driver
#include <CHAM_AVR_FLASH_DRV_IDRIVINO_V0.h>       // Custom iDrivino Flash memory driver
#include <CHAM_AVR_NTSC_DRV_IDRIVINO_V0.h>        // Custom iDrivino NTSC driver
//#include <CHAM_AVR_SOUND_DRV_IDRIVINO_V0.h>       // Custom iDrivino sound driver

//Must include NewSoftSerial after CHAM_AVR drivers
//#include <NewSoftSerial.h>
//#include <SD.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEFINES AND MACROS ////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ARMED    2
#define COMPLETE 3

//Analog IO defines
#define BOOST_ADC_PIN   A0

//Digital IO defines (note: all DIO lines default to input)
#define CAN_INT_PIN     2  //in
#define METH_FLOAT_PIN  3  //in
#define BT_RESET_PIN    4  //out
#define METH_OUT_PIN    5  //out
//#define METH_LEVELO_PIN 6  //out
//#define RS232_TX        8  //out
//#define RS232_RX        9  //in
#define RS232_BT_SEL    8  //out
#define AV_SENSE_PIN    9  //in

//Active screen
#define ONE_GAUGE       10
#define TWO_GAUGE       11
#define FOUR_GAUGE      12
#define SIX_GAUGE       13
#define PARAM_LIST      14
#define TIMER_XY        15
#define TIMER_FATS      16
#define CONFIG_SET      17
#define PROCEDE_FW_PGM  18
#define PROCEDE_MAP_PGM 19
#define CAN_SNIFFER     20
#define IDRIVINO_RESET  21
#define DATA_LOG        22
#define VALENTINE1      23
#define MAX_SCREEN_TYPE 14

//Flash memory
#define FLASH_OFFSET      256
//#define MAX_FILE_SIZE 96
#define MAX_FLASH_BUFF 32
#define MAX_FILE_BUFF  64
#define FLASH_LOG_OFFSET 1600

//Procede Parameter Code Request/Receives
#define PROCEDE_RPM           0x01
#define PROCEDE_THROTTLE      0x02
#define PROCEDE_BOOST         0x03
#define PROCEDE_LOAD_INDEX    0x05
#define PROCEDE_ENG_LOAD_IN   0x0B
#define PROCEDE_ENG_LOAD_OUT  0x0C
#define PROCEDE_MAP2          0x0D
#define PROCEDE_SPEED         0x0E
#define PROCEDE_MAP_SELECT    0x11
#define PROCEDE_IAT           0x12
#define PROCEDE_AFR           0x13
#define PROCEDE_FUEL_PRESS    0x15
#define PROCEDE_GEAR_CHANGE   0x16
#define PROCEDE_METH_INJ_FLOW 0x18
#define PROCEDE_AFR_BANK1     0x20
#define PROCEDE_OIL_TEMP      0x21
#define PROCEDE_AFR_BANK2     0x22
#define PROCEDE_DBW_THROTTLE  0x23
#define PROCEDE_DME_IGN_ADV   0x24
#define PROCEDE_DME_CODES     0x25
#define PROCEDE_DME_BOOST_TGT 0x26
#define PROCEDE_ACT_IGN_ADV   0x27
#define PROCEDE_DME_BOOST_ACT 0x28
#define PROCEDE_AT_STATUS     0x30
#define PROCEDE_KNOCK_RETARD  0x31
#define PROCEDE_CURR_AGGR_LVL 0x32
#define PROCEDE_AT_IGN_CORR   0x34
#define PROCEDE_AT_BOOST_LVL  0x35
#define PROCEDE_FUEL_CORR     0x42
#define PROCEDE_IGN_CORR      0x44
#define PROCEDE_COOLANT_TEMP  0x81  //not implemented?
#define PROCEDE_IGN_ADV       0x82
#define PROCEDE_UPSHIFT       0x8D
#define PROCEDE_STORED_VIN    0x8E
#define PROCEDE_CURRENT_VIN   0x8F
#define PROCEDE_METH_INJ_PCT  0x91
#define PROCEDE_PSET          0x05
#define PROCEDE_REQ           0x06
#define PROCEDE_MAX_PARAMS    16
#define EXPECT_RESP_15        35 //35 bytes expected from 15 parameters retrieval
#define EXTERNAL_BOOST        0xFF

//Gauge Types
#define RpmG         0
#define BoostG       1
#define ActIgnAdvG   2
#define IgnCorrG     3
#define DbwThrottleG 4
#define SpeedG       5
#define MapSelectG   6
#define IatG         7
#define MethFlowPctG 8
#define OilTempG     9
#define DmeIgnAdvG   10
#define AtBoostLvlG  11
#define DmeCodesG    12
#define AfrBank1G    13
#define AfrBank2G    14
#define BoostExtG    15
#define TimerG       16
#define MaxTypeG     16

//Debug defines
#define CAN_ACTIVE true
#define DEBUG_LOOP false

//iDrive knob defines
#define KNOB_ROTATE_RIGHT 20
#define KNOB_ROTATE_LEFT  21
#define KNOB_UP           22
#define KNOB_DOWN         23
#define KNOB_RIGHT        24
#define KNOB_LEFT         25
#define KNOB_PRESS        26
#define MENU_BUTTON       27

//iDrive Menu defines
#define IDRIVE_KNOB 0x1B8
#define MAIN_WINDOW 10
#define MENU_BAR    30
#define FOCUS1_MAX  10
#define FOCUS2_MAX  11
#define FOCUS4_MAX  13
#define FOCUSP_MAX  10
#define FOCUST_MAX  10

// define CPU frequency in Mhz here if not defined in Makefile or other includes, compiler will throw a warning, ignore
#ifndef F_CPU
#define F_CPU 16000000UL // 28636360UL, 14318180UL, 21477270 UL
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBALS ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Timer vars
byte timer_active = false;
unsigned long start_time = 0;
unsigned long end_time = 0;
unsigned long sniff_start = 0;
unsigned long idrive_timer = 0;
unsigned long log_timer = 0;
unsigned long log_start = 0;

//File vars
//FAT sdFile;      //This will be the file we manipulate in the sketch
char flash_buffer[MAX_FLASH_BUFF];
//char file_buffer[MAX_FILE_BUFF];  //Data will be temporarily stored to this buffer before being written to the file
boolean sd_present = false;
boolean flash_open = false;
short startcount = FLASH_LOG_OFFSET;

int tempint;
float tempfloat;
float boost_psi = 0.0;

//Gauge & menu related variables
byte active_screen = 1;
byte prev_active_screen = 99;
unsigned short start_vel, end_vel;
float peak_boost,timer1,timer2,meth_cal;
int one_g;
int two_g[2];
int four_g[4]; 
//int six_g[6];

//Volatile variables (anything used by the ISR should go here)
volatile boolean fresh_rx = false;

//CANBUS variables
unsigned char read_buffer[19];  //2 cmd words + 1 length wd + 16 data wds = 19, truncate if > 16 data wds
//unsigned char read_buffer_temp[10][19];
unsigned char read_buffer_temp[19];
unsigned short can_cmd = 0;
byte can_count = 0;
 
//Global Constants
//const int find_id[5] = {0x1B8,0x228,0x304,0x1D2,0x1F6};
//static int p_list[Max_Gauge_Type] = {0};
//static int p_list_old[Max_Gauge_Type] = {0};

//Setup up Procede RS232 serial vars
byte procede_counter = 0;
byte procede_array[PROCEDE_MAX_PARAMS];
static int procede_raw[PROCEDE_MAX_PARAMS] = {0};
static int p_list[PROCEDE_MAX_PARAMS] = {0};
//static int p_list_old[PROCEDE_MAX_PARAMS] = {0};
//NewSoftSerial procedeRS232(RS232_RX,RS232_TX);
boolean procede_online = false;
short curr_map_sel = 0;
short prev_map_sel = 9;
int curr_at_boost_lvl = 0;
int prev_at_boost_lvl = 1;
boolean engine_start = false;
byte sim_dir[PROCEDE_MAX_PARAMS] = {1};

//iDrive menu globals
byte window_focus = 0;
short menu_command = 0;
boolean menu_changed = false;
unsigned int last_rotate = 0;
byte bt_activity = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS /////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Flash memory functions
void load_flash_config_file();
void save_flash_config_file();

//Arduino <-> Propeller functions
void draw_one();
void update_one();
void draw_two();
void update_two();
void draw_four();
void update_four();
//void draw_six();
//void update_six();
void draw_param_list();
void update_param_list();
void draw_timer();
void update_timer();
void draw_fats();
void update_fats();
void draw_tools();
void update_tools();
void draw_procede();
void update_procede_fw();
void update_procede_map();
void draw_firmware();
void update_firmware();
void draw_logging();
void update_logging();
void draw_sniffer();
void update_sniffer();
void draw_v1();
void update_v1();
void draw_bottom_menu();
void update_bottom_menu();
void draw_highlight(byte);
void update_main_highlight();
void check_for_highlight_timeout();
void download_btsketch();
void update_screen_from_iDrive();
void read_canbus_message();
void right_prompt();
void check_for_reinit();
void start_flash_logging();
void record_flash_logging();
void read_flash_logging();
void stop_flash_logging();
//void procede_fw_upload();
//void procede_map_upload();
//void run_sniffer();

//Procede functions
boolean procede_init();
boolean update_all_params();
int lookup_single_param(short);
void sim_all_params();

//Misc functions
//float read_std_obd2(unsigned short);
int get_update_param(int);
unsigned long start_stop_timer(unsigned short,unsigned short);
void find_can_id(unsigned short);
int check_for_shutdown();
void verify_config();
void command_memorycheck();
//void canbus_to_serial(unsigned char *);

//Interrupt Service Routine (ISR)
void Canbus_ISR();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP CALL (ARDUINO SPECIFIC: initialization call) ////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()                    // run once, when the sketch starts
{
  
} // end setup

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP CALL (ARDUINO SPECIFIC: main entry point) ////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()                     // run over and over again
{
  unsigned long timerloop = millis();
  unsigned long meth_on_time;
  //int autobaud = -1;
  int cnt = 0;
  byte loop_time = 30;
  byte update_fails = 0;
  
  // initialize and set SPI rate
  SPI_Init( SPI_DEFAULT_RATE );
  // allow time for SPI & Propeller to initialize
  _delay_ms(2500);
  
  Serial.begin(57600);
  //command_memorycheck();
  
  NTSC_ClearScreen();
  NTSC_Term_Print("iDrivino v0.1/");
  NTSC_Term_Print("System loading.../");
  
  if (CAN_ACTIVE == true)
  {
    if (Canbus.init(CANSPEED_100A,FILTER_ON))  // Initialize MCP2515 CAN controller at the specified speed
    {
      NTSC_Term_Print("CAN Init OK/");
    } 
    else
    {
      NTSC_Term_Print("Cannot init CAN!/");
    }
  }
/*  if (CAN_ACTIVE == true)
  {
    autobaud = Canbus.init(0);
    sprintf(buff,"Autobaud setting of %d returned/",autobaud);
    NTSC_Term_Print(buff);
  }
*/
  else
  {
    NTSC_Term_Print("CAN-bus interface disabled/");
  }
  
  //Set DIO output pins (default is input)
  pinMode(CAN_INT_PIN,INPUT);
  pinMode(BT_RESET_PIN,OUTPUT);
  pinMode(METH_FLOAT_PIN,INPUT);
  pinMode(RS232_BT_SEL,OUTPUT);
  pinMode(AV_SENSE_PIN,INPUT);
  pinMode(METH_OUT_PIN,OUTPUT);
  //pinMode(10,OUTPUT); //must be output for SD library
  
  //Set analog pins
  pinMode(BOOST_ADC_PIN,INPUT);

  delay(50);
  
  //Set output behavior
  digitalWrite(RS232_BT_SEL,HIGH);    //set relay to RS232
  digitalWrite(METH_FLOAT_PIN,HIGH);  //set pullup on float switch
  digitalWrite(AV_SENSE_PIN,HIGH);    //set pullup on AV sense line
  digitalWrite(METH_OUT_PIN,LOW);    //default meth output off
  
  //Manually call out RX/TX pins: NewSoftSerial is called globally and initalizes RX/TX, but SPI_Init() resets pinMode
/*  pinMode(RS232_RX,INPUT);  
  digitalWrite(RS232_RX,HIGH);
  pinMode(RS232_TX,OUTPUT);
  digitalWrite(RS232_TX,HIGH);
  _delay_ms(100);
  procedeRS232.begin(57600);
*/
    
  if (procede_init())
  {
    NTSC_Term_Print("Procede init OK/");
  }
  else
  {
    NTSC_Term_Print("Cannot init Procede!/");
    NTSC_Term_Print("Entering simulation mode.../");
  }
  /*while (procede_init() == 0)
  {
    //procede_init() already has 1 sec delay leaving sub
    _delay_ms(2000);
  }*/
  
  //draw_intro();
  NTSC_ClearScreen();

  //Load config file from FLASH
  load_flash_config_file();
  //Verify config file params OK
  verify_config();
  
//###flash debug
/*  start_flash_logging();
  for (int t=0;t<800;t++)
  {
    record_flash_logging();
    _delay_ms(1);
  }
  read_flash_logging();
  stop_flash_logging();
  do {} while (1);*/
  //start_flash_logging();  
  
  //Insert delay so all screen prints/intros are done before interrupts start
  _delay_ms(1000);

  //Attach interrupt to CAN-bus interrupt line (DIO pin 2)
  attachInterrupt(0,Canbus_ISR,FALLING);
  
  // enter infinite loop...
  while(1)
  {
    //Check for Arduino firmware update via Bluetooth
    //if (Serial.read() == '@')
    //{ // check if command is '}'
    //  download_btsketch();
    //}

    //Only draw/update every 30ms, since ~30Hz refresh rate acceptable for motion
    //  Timer functions will update every 10ms for greater timing accuracy
    if ((active_screen == TIMER_XY) || (active_screen == TIMER_FATS))
      loop_time = 10;
    else if (active_screen == CAN_SNIFFER)
      loop_time = 0;
    else
      loop_time = 30;
    
    if ((millis()-timerloop) >= loop_time)
    {
      if (prev_active_screen == active_screen)
      {
        if (active_screen < CONFIG_SET)
        {
          if (procede_online == false)
          {
            sim_all_params();
          }
          else
          {
            if (update_all_params() == false)
            {
              update_fails++; 
            }
            else
            {
              update_fails = 0;
            }
          }
        }
        
        //Debug data logging
      
      /*record_flash_logging();
      cnt++;
      if (cnt > 1800)
      {
        read_flash_logging();
        do{} while (1);      
      }*/
        
        update_bottom_menu();
        
        switch (active_screen)
        {
          case ONE_GAUGE:
            update_one();
            break; 
          case TWO_GAUGE:
            update_two();
            break;
          case FOUR_GAUGE:
            update_four();
            break;
          case SIX_GAUGE:
            //update_six();
            break;
          case PARAM_LIST:
            update_param_list();
            break;
          case TIMER_XY:
            update_timer();
            break;
          case TIMER_FATS:
            update_fats();
            break;
          case CONFIG_SET:
            //update_tools();
            break;
/*          case PROCEDE_FW_PGM:
            update_procede_fw();
            break;
          case PROCEDE_MAP_PGM:
            update_procede_map();
            break;
*/
          case CAN_SNIFFER:
            update_sniffer();
            break;
/*          case IDRIVINO_RESET:
            //update_firmware();
            break;    
*/
          case DATA_LOG:
            update_logging();
            break;
          case VALENTINE1:
            update_v1();
            break;
          default:
            break;
        } //switch
      }
      else
      {
        draw_bottom_menu();
        switch (active_screen)
        {
          case ONE_GAUGE:
            draw_one();
            break; 
          case TWO_GAUGE:
            draw_two();
            break;
          case FOUR_GAUGE:
            draw_four();
            break;
          case SIX_GAUGE:
            //draw_six();
            break;
          case PARAM_LIST:
            draw_param_list();
            break;
          case TIMER_XY:
            draw_timer();
            break;
          case TIMER_FATS:
            draw_fats();
            break;
          case CONFIG_SET:
            draw_tools();
            break;
/*          case PROCEDE_FW_PGM:
          case PROCEDE_MAP_PGM:
*/
          case CAN_SNIFFER:
            draw_sniffer();
            break;
          case IDRIVINO_RESET:
            break;    
          case DATA_LOG:
            draw_logging();
            break;
          case VALENTINE1:
            draw_v1();
            break;
          default:
            break;
        } //switch
  
      }
      
      check_for_reinit();
      check_for_highlight_timeout();
      
      prev_active_screen = active_screen;

      if ((check_for_shutdown() > 30) || (update_fails > 30))
      {
        do
        {
          delay(1000);
        }
        while (procede_init() == false);
        
        GFX_ClearMainWindow();
        prev_active_screen = 99;
        update_fails = 0;
      }
      
      timerloop = millis();
    }  //end if millis()
    
    read_canbus_message();
    update_screen_from_iDrive();
    
    boost_psi = (analogRead(BOOST_ADC_PIN)*0.0049)*8.94-14.53;  //GM 3-bar sensor scaling: V*8.94 - 14.53
    
    if ((boost_psi >= 3.5) && (curr_map_sel > 0) && (digitalRead(METH_FLOAT_PIN) == LOW)) //###
    {
      //turn meth output pin on
      digitalWrite(METH_OUT_PIN,HIGH);
      meth_on_time = millis();
    }
    else
    {
      //turn meth output pin off if it's on for at least 1 sec
      //  this prevents rapid cycling of the output pin if directly driving a pump relay, control box, etc
      if ((millis()-meth_on_time) > 1000)
      {
        digitalWrite(METH_OUT_PIN,LOW);
      }
    }
    
  } // end while(1)    

} // end main

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Load Config File //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void load_flash_config_file()
{
  char temp_read[16];
  //char temp_str[32];

  // open the 1MB FLASH memory up and unprotect the sectors 
  NTSC_Term_Print(" Opening FLASH memory/");

  Flash_Open();

  // read and print the first MAX_FILE_SIZE bytes of the FLASH memory, non-printable character will print as dots '.'
  NTSC_Term_Print(" Reading data from FLASH/");

/*  Bytes  Purpose (of 1024K in FLASH)
  ----------------------------------
  0-1    active_screen
  2-4    start_vel
  5-7    end_vel
  8-15   timer1
  16-23  timer2
  24-25  first_run_flag
  26-31  
  32-39  methanol calibration
  40-41  One_Gauge active type
  42-43  Two_Gauge active type A
  44-45  Two_Gauge active type B
  46-47  Four_Gauge active type A
  48-49  Four_Gauge active type B
  50-51  Four_Gauge active type C
  52-53  Four_Gauge active type D
  54-55  
  56-57  
  58-59  
  60-61  
  62-63  
  64-65  
  ..
  100-108 peak 0
  109-116 peak 1
  117-124 peak 2
  125-132 peak 3
  133-140 peak 4
  141-148 peak 5
  149-156 peak 6
  157-164 peak 7
  165-172 peak 8
  173-180 peak 9
  181-188 peak 10
  189-196 peak 11
  197-204 peak 12
  205-212 peak 13
  213-220 peak 14
  221-228 peak 15  
*/

  Flash_Read(0, (unsigned char *)flash_buffer, MAX_FLASH_BUFF);
  
  strncpy(temp_read,&flash_buffer[0],2);
  temp_read[2] = '\0';
  active_screen = atoi(temp_read);
  //sprintf(temp_str,"Active screen = %s,%d",temp_read,active_screen);
  //NTSC_Term_Print(temp_str);

  strncpy(temp_read,&flash_buffer[2],3);
  temp_read[3] = '\0';
  start_vel = atoi(temp_read);
  //sprintf(temp_str,"|Start v = %s,%d/",temp_read,start_vel);
  //NTSC_Term_Print(temp_str);

  strncpy(temp_read,&flash_buffer[5],3);
  temp_read[3] = '\0';
  end_vel = atoi(temp_read);
  //sprintf(temp_str,"End v = %s,%d",temp_read,end_vel);
  //NTSC_Term_Print(temp_str);
  
  strncpy(temp_read,&flash_buffer[8],8);
  temp_read[8] = '\0';
  timer1 = atof(temp_read);
  //sprintf(temp_str,"|Timer1 = %s,%d/",temp_read,timer1);
  //NTSC_Term_Print(temp_str);
  
  strncpy(temp_read,&flash_buffer[16],8);
  temp_read[8] = '\0';
  timer2 = atof(temp_read);
  //sprintf(temp_str,"Timer2 = %s,%d",temp_read,timer2);
  //NTSC_Term_Print(temp_str);
  
  // Bytes 24-31
  //strncpy(temp_read,&flash_buffer[24],8);
  //temp_read[8] = '\0';
  //peak_boost = atof(temp_read);
  //sprintf(temp_str,"|Peak boost = %s/",temp_read);
  //NTSC_Term_Print(temp_str);
  
  Flash_Read(MAX_FLASH_BUFF*1, (unsigned char *)flash_buffer, MAX_FLASH_BUFF);
  
  strncpy(temp_read,&flash_buffer[0],8);
  temp_read[8] = '\0';
  meth_cal = atof(temp_read);
  //sprintf(temp_str,"Meth Cal = %s",temp_read);
  //NTSC_Term_Print(temp_str);
  
  strncpy(temp_read,&flash_buffer[8],2);
  temp_read[2] = '\0';
  one_g = atoi(temp_read);
  //sprintf(temp_str,"|Gauge type 1 = %s,%d/",temp_read,one_g);
  //NTSC_Term_Print(temp_str);

  strncpy(temp_read,&flash_buffer[10],2);
  temp_read[2] = '\0';
  two_g[0] = atoi(temp_read);
  //sprintf(temp_str,"Gauge type 2A = %s,%d",temp_read,two_g[0]);
  //NTSC_Term_Print(temp_str);
  
  strncpy(temp_read,&flash_buffer[12],2);
  temp_read[2] = '\0';
  two_g[1] = atoi(temp_read);
  //sprintf(temp_str,"|2B = %s,%d/",temp_read,two_g[1]);
  //NTSC_Term_Print(temp_str);
  
  strncpy(temp_read,&flash_buffer[14],2);
  temp_read[2] = '\0';
  four_g[0] = atoi(temp_read);
  //sprintf(temp_str,"Gauge type 4A = %s,%d",temp_read,four_g[0]);
  //NTSC_Term_Print(temp_str);
  
  strncpy(temp_read,&flash_buffer[16],2);
  temp_read[2] = '\0';
  four_g[1] = atoi(temp_read);
  //sprintf(temp_str,"|4B = %s,%d/",temp_read,four_g[1]);
  //NTSC_Term_Print(temp_str);
  
  strncpy(temp_read,&flash_buffer[18],2);
  temp_read[2] = '\0';
  four_g[2] = atoi(temp_read);
  //sprintf(temp_str,"Gauge type 4C = %s,%d",temp_read,four_g[2]);
  //NTSC_Term_Print(temp_str);
  
  strncpy(temp_read,&flash_buffer[20],2);
  temp_read[2] = '\0';
  four_g[3] = atoi(temp_read);
  //sprintf(temp_str,"|4D = %s,%d/",temp_read,four_g[3]);
  //NTSC_Term_Print(temp_str);
  
  // close the 1MB FLASH memory and protect the sectors 
  NTSC_Term_Print(" Closing FLASH memory.../");

  Flash_Close();

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Save Config File //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void save_flash_config_file()
{
  char temp_save[16];
  
  // open the 1MB FLASH memory up and unprotect the sectors 
  //NTSC_Term_Print("Opening FLASH memory.../");
  
  Flash_Open();

  // write file to FLASH...

  // step 1: erase the first 4K block (the function takes the 4K block number)
  //NTSC_Term_Print("Erasing first 4K of FLASH...");
  
  Flash_Erase( FLASH_ADDR_TO_BLOCK( 0 ) );

  // now write the bytes to the FLASH starting at address 0...
  //sprintf(sbuffer,"Writing %d bytes to FLASH.../",bytes_received);
  //NTSC_Term_Print( sbuffer );
  
  itoa((int)active_screen,temp_save,10);
  strncpy(&flash_buffer[0],temp_save,2);
  
  itoa(start_vel,temp_save,10);
  strncpy(&flash_buffer[2],temp_save,3);
  
  itoa(end_vel,temp_save,10);
  strncpy(&flash_buffer[5],temp_save,3);
  
  dtostrf(timer1,8,3,temp_save);
  strncpy(&flash_buffer[8],temp_save,8);

  dtostrf(timer2,8,3,temp_save);
  strncpy(&flash_buffer[16],temp_save,8);

  //dtostrf(peak_boost,8,2,temp_save);
  //itoa((int)first_run_flag,temp_save,10);
  //strncpy(&flash_buffer[24],temp_save,2);
  
  Flash_Write(0, (unsigned char *)flash_buffer, MAX_FLASH_BUFF);

  dtostrf(meth_cal,8,3,temp_save);
  strncpy(&flash_buffer[0],temp_save,8);
  
  itoa((int)one_g,temp_save,10);
  strncpy(&flash_buffer[8],temp_save,2);
  
  itoa((int)two_g[0],temp_save,10);
  strncpy(&flash_buffer[10],temp_save,2);
  itoa((int)two_g[1],temp_save,10);
  strncpy(&flash_buffer[12],temp_save,2);
  
  itoa((int)four_g[0],temp_save,10);
  strncpy(&flash_buffer[14],temp_save,2);
  itoa((int)four_g[1],temp_save,10);
  strncpy(&flash_buffer[16],temp_save,2);
  itoa((int)four_g[2],temp_save,10);
  strncpy(&flash_buffer[18],temp_save,2);
  itoa((int)four_g[3],temp_save,10);
  strncpy(&flash_buffer[20],temp_save,2);

  Flash_Write(MAX_FLASH_BUFF*1, (unsigned char *)flash_buffer, MAX_FLASH_BUFF);

  // close the 1MB FLASH memory and protect the sectors 
  //NTSC_Term_Print("Closing FLASH memory.../");

  Flash_Close();

}

/*Procede data logging file size estimate, based on 85 bytes written 30 times per second (2550Bps):

  B/s	KBps	        KB/min	        KB/hr	        MB/hr
  2550	2.490234375	149.4140625	8964.84375	8.754730224609375
  
  Given a flash size just under 1MB, you could datalog for approx 6 mins continuously.
  
  Exact file size will depend on the # of bytes written per second.  The # of bytes are based on the specific
  parameters chosen to log, as some params are decimal, some integer, etc. which change the byte count.
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_flash_logging()
{
  char temp_buffer[42];
  byte bytecount = 0;
  
  Flash_Open();
  
  //Serial.println("erasing flash");

  //BT files at 8kB-256kB, blocks 2 thru 63 of 0-255 (4kB)
  //size = 785kB, approx 10 mins of logging
  //for (int i=64;i<256;i++)
  /*for (int i=1;i<16;i++)
  {
    //Flash_Erase( FLASH_ADDR_TO_BLOCK( i ) );
    Flash_Erase64 (FLASH_ADDR_TO_BLOCK64(i));
    Serial.println(millis());
    Serial.println(i);
  }*/
  Serial.println(millis());
  Flash_ChipErase();
  Serial.print(millis());
  Serial.println(" chip erased");

/*  startcount = FLASH_LOG_OFFSET;
  do
  { 
   //buffer should be 1 character less than the Flash read count (63 vs 64) so extraneous null termination char is not printed! 
    Flash_Read(startcount,(unsigned char *)temp_buffer,64);
    Serial.print(temp_buffer);
    startcount += 64;
  }
  while (startcount < 1000);
*/

  startcount = FLASH_LOG_OFFSET;
  /* Write header info to flash
  Procede Data Log
  Data File 1
  5
  time,RPM,Load Index,Fuel Cor,Ign Cor,Boost
  */
  //sprintf(temp_buffer,"Procede Data Log\n");
  
  sprintf(temp_buffer,"Procede Data Log\n");
  bytecount = strlen(temp_buffer);
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount);
  
  //Serial.print("wrote ");
  //Serial.println(startcount);
  
  sprintf(temp_buffer,"Data File 1\n");
  startcount += bytecount;
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));

  //Serial.print("wrote ");
  //Serial.println(startcount);
  
  sprintf(temp_buffer,"%d\n",PROCEDE_MAX_PARAMS-1);
  startcount += bytecount;
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
  
  //Serial.print("wrote ");
  //Serial.println(startcount);
  
  sprintf(temp_buffer,"time,RPM,Boost,CAN Actual Ign Adv,");
  startcount += bytecount;
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
  
  //Serial.print("wrote ");
  //Serial.println(startcount);
  
  sprintf(temp_buffer,"Ign Cor,CAN DBW Throttle,Road Speed,");
  startcount += bytecount;
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
  
  //Serial.print("wrote ");
  //Serial.println(startcount);
  
  sprintf(temp_buffer,"Current Map Selection,Inlet Air Temp,");
  startcount += bytecount;
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
  
  //Serial.print("wrote ");
  //Serial.println(startcount);

  sprintf(temp_buffer,"Methanol Injection Flow,CAN Oil Temp,");
  startcount += bytecount;
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
  
  //Serial.print("wrote ");
  //Serial.println(startcount);
  
  sprintf(temp_buffer,"CAN DME Ign Adv,Autotune Boost Setting,");
  startcount += bytecount;
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
  
  //Serial.print("wrote ");
  //Serial.println(startcount);
  
  sprintf(temp_buffer,"CAN DME Codes,CAN Actual AFR Bank 1,");
  startcount += bytecount;
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
  
  //Serial.print("wrote ");
  //Serial.println(startcount);
  
  sprintf(temp_buffer,"CAN Actual AFR Bank 2\n");
  startcount += bytecount;
  Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));

  //Serial.print("wrote ");
  //Serial.println(startcount);

  flash_open = true;
  startcount += bytecount;
  
  log_start = 0;
  log_timer = 0;
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void record_flash_logging()
{
  char temp_buffer[16];
  char temp_num[12];
  static byte bytecount = 0;
  static byte param_count = 0;
  
  //Do not log if Procede is offline or Flash is not opened on purpose
//  if ((procede_online == false) || (flash_open == false))
//    return;
    
  //Check for max Flash memory size, do not go past 0xfffff - 100 = 0xfff9b
  if (startcount >= 0xfff9b)
  {
    return;
  }
  
  if (log_start == 0)
  {
    log_start = millis();
  }
  
  /*7000,22.9,64.0,-15.75,100,199,10,130.1,
       100,523.4,64.1,15.4,65535,29.4,29.4
    */
  
  //Do something if at least 1 ms has elapsed since last write
  if (millis() - log_timer >= 1)
  {
    switch (param_count)
    {  
      case 0:
        //Timestamp
        dtostrf((millis()-log_start)/1000.0,7,3,temp_num);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        //RPM
        itoa(lookup_single_param(PROCEDE_RPM),temp_num,10);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 1:
        //Boost
        //dtostrf(FLOAT,WIDTH,PRECISION,BUFFER);
        dtostrf(lookup_single_param(PROCEDE_BOOST)/100.0,6,2,temp_num);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 2:
        //Act Ign Adv
        //dtostrf(lookup_single_param(PROCEDE_ACT_IGN_ADV)/100.0,6,2,temp_num);
        dtostrf(5.211,6,2,temp_num);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 3:
        //Ign Cor
        dtostrf(lookup_single_param(PROCEDE_IGN_CORR)/100.0,6,2,temp_num);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 4:
        //Dbw Throttle
        itoa(lookup_single_param(PROCEDE_DBW_THROTTLE),temp_num,10);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 5:
        //Road Speed
        //dtostrf(lookup_single_param(PROCEDE_SPEED)/100.0,6,2,temp_num);
        dtostrf(12.34,6,2,temp_num);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 6:
        //Map Select
        itoa(lookup_single_param(PROCEDE_MAP_SELECT),temp_num,10);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 7:
        //IAT
        itoa(lookup_single_param(PROCEDE_IAT),temp_num,10);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 8:
        //Meth Flow
        itoa(lookup_single_param(PROCEDE_METH_INJ_FLOW),temp_num,10);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 9:
        //Oil Temp
        itoa(lookup_single_param(PROCEDE_OIL_TEMP),temp_num,10);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 10:
        //DME Ign Adv
        dtostrf(lookup_single_param(PROCEDE_DME_IGN_ADV)/100.0,6,2,temp_num);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 11:
        //Autotune Boost
        dtostrf(lookup_single_param(PROCEDE_AT_BOOST_LVL)/100.0,6,2,temp_num);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 12:
        //DME Codes
        itoa(lookup_single_param(PROCEDE_DME_CODES),temp_num,10);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 13:
        //AFR1
        dtostrf(lookup_single_param(PROCEDE_AFR_BANK1)/100.0,4,1,temp_num);
        sprintf(temp_buffer,"%s,",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      case 14:
        //AFR2
        dtostrf(lookup_single_param(PROCEDE_AFR_BANK2)/100.0,4,1,temp_num);
        sprintf(temp_buffer,"%s\n",temp_num);
        startcount += bytecount;
        Flash_Write(startcount, (unsigned char *)temp_buffer, bytecount = strlen(temp_buffer));
        /*Serial.print("wrote @ ");
        Serial.print(startcount);
        Serial.println(temp_buffer);*/
        break;
      default:
        return;
    }
    
    if (param_count == 14)
      param_count = 0;
    else
      param_count++;
    
    log_timer = millis();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_flash_logging()
{
  char temp_buffer[2];
  
  if (!flash_open)
  {
    Flash_Open();
    flash_open = true;
  }
  
  startcount = FLASH_LOG_OFFSET;
  
  do
{ 
 
 //buffer should be 1 character less than the Flash read count (63 vs 64) so extraneous null termination char is not printed! 
  /*Flash_Read(startcount,(unsigned char *)temp_buffer,64);
  Serial.print("read @ ");
  Serial.print(startcount);
  Serial.print(" = ");*/
  Flash_Read(startcount,(unsigned char *)temp_buffer,1);
  if (temp_buffer[0] == '\n')
  {
    Serial.println();
    Serial.print(startcount);
    Serial.print("= ");
  }
  else
  {
    Serial.print(temp_buffer[0]);
  }
  //Serial.println(temp_buffer);
  //startcount += 64;
  startcount++;
}
while (startcount < 6000);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stop_flash_logging()
{

  Flash_Close();
  
  startcount = FLASH_LOG_OFFSET;
  flash_open = false;
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Load Config File //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void load_sd_config_file()
{
  
/*  Bytes  Purpose (of 1024K in FLASH)
  ----------------------------------
  0-1    active_screen
  2-4    start_vel
  5-7    end_vel
  8-15   timer1
  16-23  timer2
  24-25  first_run_flag
  26-31  
  32-39  methanol calibration
  40-41  One_Gauge active type
  42-43  Two_Gauge active type A
  44-45  Two_Gauge active type B
  46-47  Four_Gauge active type A
  48-49  Four_Gauge active type B
  50-51  Four_Gauge active type C
  52-53  Four_Gauge active type D
  54-55  
  56-57  
  58-59  
  60-61  
  62-63  
  64-65  
  ..
  100-108 peak 0
  109-116 peak 1
  117-124 peak 2
  125-132 peak 3
  133-140 peak 4
  141-148 peak 5
  149-156 peak 6
  157-164 peak 7
  165-172 peak 8
  173-180 peak 9
  181-188 peak 10
  189-196 peak 11
  197-204 peak 12
  205-212 peak 13
  213-220 peak 14
  221-228 peak 15  
*/

/*
  int bytes_read=0; //Keeps track of how many bytes are read when accessing a file on the SD card.
  byte read_count = 0;
  
  if (sd_present == false)
    return;

  //sdFile.open_file("config.txt");  //Open the file. When the file is opened we will be looking at the beginning of the file.
  
  //should only need to loop twice?
  do
  {
    //bytes_read = sdFile.read(file_buffer);
    read_count++;
    Serial.println((const char *)file_buffer);
    char *token = strtok(file_buffer,";");
    while (token)
    {
      //do something with the token value
      int val = atoi(token);
      
      token = strtok(NULL,";");  // Use NULL to keep parsing until buffer is completey parsed
    }
    
    /*
    //tokenize & check for newline
    while ((str = strtok_r(file_buffer, ";", &p)) != "\n") // delimiter is the comma
    {
      Serial.println(str);
      file_buffer = NULL;
    }*/
    
    
//  }
//  while (bytes_read > 0);

  //sdFile.close();

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw Intro /////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_intro()
{

  //Draw splash screen
  //NTSC_Term_Print("Intro screen/");
  //GFX_Splash_In();
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw one gauge ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_one()
{
  GFX_ClearMainWindow();
  
  //Draw main display area
  GFX_One_Gauge(one_g,-99);
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update one gauge //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_one()
{

  GFX_One_Gauge(lookup_single_param(procede_array[one_g]),0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw two gauges ///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_two()
{
  int temp8bit = 0;
  
  GFX_ClearMainWindow();
  
  //Draw main display area
  //Pack gauge types into 4 bits each of 16 bit variable so only 1 SPI cmd needed
  temp8bit = temp8bit + ((two_g[0]&0xF) << 4);
  temp8bit += two_g[1]&0xF;

  GFX_Two_Gauge(temp8bit,-99);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_two()
{
  
  GFX_Two_Gauge(-99,0);
  GFX_Two_Gauge(lookup_single_param(procede_array[two_g[0]]),0);

  GFX_Two_Gauge(-99,1);
  GFX_Two_Gauge(lookup_single_param(procede_array[two_g[1]]),1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw four gauges //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_four()
{
  int temp16bit = 0;
  
  GFX_ClearMainWindow();
  
  //Draw main display area
  //Pack gauge types into 4 bits each of 16 bit variable so only 1 SPI cmd needed
  temp16bit = temp16bit + ((four_g[0]&0xF) << 12);
  temp16bit = temp16bit + ((four_g[1]&0xF) << 8);
  temp16bit = temp16bit + ((four_g[2]&0xF) << 4);
  temp16bit = temp16bit + (four_g[3]&0xF);

  GFX_Four_Gauge(temp16bit,-99);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_four()
{

  GFX_Four_Gauge(-99,0);
  GFX_Four_Gauge(lookup_single_param(procede_array[four_g[0]]),0);

  GFX_Four_Gauge(-99,1);
  GFX_Four_Gauge(lookup_single_param(procede_array[four_g[1]]),1);
  
  GFX_Four_Gauge(-99,2);
  GFX_Four_Gauge(lookup_single_param(procede_array[four_g[2]]),2);

  GFX_Four_Gauge(-99,3);
  GFX_Four_Gauge(lookup_single_param(procede_array[four_g[3]]),3);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw six gauge ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*void draw_six()
{
  int temp8bit1 = 0, temp8bit2 = 0;
  
  //NTSC_ClearScreen();
  
  //Draw main display area
  //Pack gauge types into 4 bits each of 16 bit variable so only 1 SPI cmd needed
  temp8bit1 = temp8bit1 + ((six_g[0]&0xF) << 4);
  temp8bit1 += six_g[1]&0xF;
  temp8bit2 = temp8bit2 + ((six_g[2]&0xF) << 4);
  temp8bit2 += six_g[3]&0xF;
  GFX_Six_Gauge1(temp8bit1,temp8bit2);
  
  temp8bit1 = 0;
  temp8bit1 = temp8bit1 + ((six_g[4]&0xF) << 4);
  temp8bit1 += six_g[5]&0xF;
  GFX_Six_Gauge2(temp8bit1);
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_six()
{

}*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw Parameter List ///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_param_list()
{
  int dummy_list[1] = {0};
  
  GFX_ClearMainWindow();

  //Send -99 to tell Propeller to draw paramlist for 1st time
  GFX_Param_List(dummy_list,-99);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Parameter List ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_param_list()
{
  
  for (byte i=0;i<PROCEDE_MAX_PARAMS;i++)
  {
    tempint = lookup_single_param(procede_array[i]);
    p_list[i] = tempint;
  }

  GFX_Param_List(p_list,PROCEDE_MAX_PARAMS-1);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw X-to-Y Timer ////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_timer()
{
  GFX_ClearMainWindow();
  
  timer_active = false;
  end_time = 0;
  
  //Mask 8 bits, 0-255 mph is more than enough
  start_vel &= 0xFF;
  end_vel &= 0xFF;
  
  GFX_SetTimerXY(start_vel,end_vel);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_timer()
{
  
  if ((timer_active == true) || (timer_active == ARMED))
  {
    GFX_TimerXY(-99,0);
    GFX_TimerXY(lookup_single_param(PROCEDE_SPEED),0);

    GFX_TimerXY(-99,1);
    GFX_TimerXY((int)(start_stop_timer(start_vel, end_vel)/10),1);
  }
  else
  {
    GFX_TimerXY(-99,0);
    GFX_TimerXY(lookup_single_param(PROCEDE_SPEED),0);
    
    start_stop_timer(start_vel,end_vel);
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw X-to-Y Timer ////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_fats()
{
  GFX_ClearMainWindow();
  
  timer_active = false;
  end_time = 0;
  
  //Mask 8 bits, 0-255 mph is more than enough
  start_vel &= 0xFF;
  end_vel &= 0xFF;
  
  GFX_SetTimerFATS(start_vel,end_vel);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_fats()
{
  
  if ((timer_active == true) || (timer_active == ARMED))
  {
    GFX_TimerFATS(-99,0);
    GFX_TimerFATS(lookup_single_param(PROCEDE_SPEED),0);

    GFX_TimerFATS(-99,1);
    GFX_TimerFATS((int)(start_stop_timer(start_vel, end_vel)/10),1);
  }
  else
  {
    GFX_TimerFATS(-99,0);
    GFX_TimerFATS(lookup_single_param(PROCEDE_SPEED),0);
    
    start_stop_timer(start_vel,end_vel);
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw Tools Screen ////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_tools()
{
  GFX_ClearMainWindow();
  
  GFX_Print_SetXY(15,190);
  GFX_Print_XY("Bluetooth Tools",3);
  //GFX_Box(15,170,120,20);
  
  GFX_Print_SetXY(25,170);
  GFX_Print_XY("Procede Firmware",1);
  GFX_Box(15,160,120,20);
  
  GFX_Print_SetXY(25,150);
  GFX_Print_XY("Procede Map",1);
  GFX_Box(15,140,120,20);
  
  GFX_Print_SetXY(25,130);
  GFX_Print_XY("CANBUS Sniffer",1);
  GFX_Box(15,120,120,20);
  
  GFX_Print_SetXY(15,110);
  GFX_Print_XY("iDrivino Options",3);
  //GFX_Box(15,100,120,15);
  
  GFX_Print_SetXY(25,90);
  GFX_Print_XY("Reboot iDrivino",1);
  GFX_Box(15,80,120,20);
  
  digitalWrite(RS232_BT_SEL,LOW);  //set relay to BT
  
//  GFX_Print_Hex_XY(0x7f,1);
//  GFX_Print_Hex_XY(0x80,1);  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_tools(byte prev_window_focus)
{
  if ((window_focus > 0) && (window_focus < MENU_BAR))
  {
    draw_tools();
    
    if ((window_focus == MAIN_WINDOW) && (prev_window_focus != MAIN_WINDOW))
    {
      GFX_Print_SetXY(25,170);
      GFX_Print_XY("Procede Firmware",2);
      
/*      GFX_Print_SetXY(25,150);
      GFX_Print_XY("Procede Map",1);
      GFX_Print_SetXY(25,130);
      GFX_Print_XY("CANBUS Sniffer",1);
      GFX_Print_SetXY(25,90);
      GFX_Print_XY("Reboot iDrivino",1);*/
    }
    else if ((window_focus == MAIN_WINDOW+1) && (prev_window_focus != MAIN_WINDOW+1))
    {
      GFX_Print_SetXY(25,150);
      GFX_Print_XY("Procede Map",2);
      
/*      GFX_Print_SetXY(25,170);
      GFX_Print_XY("Procede Firmware",1);
      GFX_Print_SetXY(25,130);
      GFX_Print_XY("CANBUS Sniffer",1);
      GFX_Print_SetXY(25,90);
      GFX_Print_XY("Reboot iDrivino",1);*/
    }
    else if ((window_focus == MAIN_WINDOW+2) && (prev_window_focus != MAIN_WINDOW+2))
    {
      GFX_Print_SetXY(25,130);
      GFX_Print_XY("CANBUS Sniffer",2);
      
/*      GFX_Print_SetXY(25,170);
      GFX_Print_XY("Procede Firmware",1);
      GFX_Print_SetXY(25,150);
      GFX_Print_XY("Procede Map",1);
      GFX_Print_SetXY(25,90);
      GFX_Print_XY("Reboot iDrivino",1);*/
    }
    else if ((window_focus == MAIN_WINDOW+3) && (prev_window_focus != MAIN_WINDOW+3))
    {
      GFX_Print_SetXY(25,90);
      GFX_Print_XY("Reboot iDrivino",2);
      
/*      GFX_Print_SetXY(25,170);
      GFX_Print_XY("Procede Firmware",1);
      GFX_Print_SetXY(25,150);
      GFX_Print_XY("Procede Map",1);
      GFX_Print_SetXY(25,130);
      GFX_Print_XY("CANBUS Sniffer",1);*/
    }
    
    right_prompt();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw Procede Wireless Upload Screen ///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_procede()
{
  GFX_ClearMainWindow();
  
  NTSC_Term_Print("Procede Wireless Upload/");
  
  //Switch relay to Bluetooth serial mode
  //digitalWrite(RS232_BT_SEL,LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_procede_fw()
{
//  unsigned char temp_buffer[MAX_FILE_BUFF] = {0};
  int timeout = 5000;
  byte mem_ptr = 128;
  
  if (bt_activity == false)
  {
    // open the 1MB FLASH memory up and unprotect the sectors 
/*    Flash_Open();
    //BT files at 8kB-256kB, blocks 2 thru 63 of 0-255 (4kB)
    //  This allows a max size of 248kB file to be loaded
    for (int i=2;i<64;i++)
    {
      Flash_Erase( FLASH_ADDR_TO_BLOCK( i ) );
    }
*/    
    bt_activity = ARMED;
  }
  else if (bt_activity == ARMED)
  {
    //BT is trying to send data
    if (Serial.available() > 0)
      bt_activity = true;
    
    while (timeout > 0)
    {
      while (Serial.available())
      {
//        Flash_Write(mem_ptr,(unsigned char *)Serial.read(),1);
        //Read 64 byte chunks
        /*if (Serial.available() >= 64)
        {
          for (int i=0;i<64;i++)
          {
            temp_buffer[i] = Serial.read();
          }
          Flash_Write(mem_ptr, (unsigned char *)temp_buffer, MAX_FILE_BUFF);
          mem_ptr++;
        }*/
        mem_ptr++;
      }
      //_delay_ms(1);
      timeout--;
    }
    
    //Clean up any remaining bytes < 64
    /*if (Serial.available() > 0)
    {
      for (int i=0;i<Serial.available();i++)
      {
        temp_buffer[i] = Serial.read();
      }
      Flash_Write(mem_ptr, (unsigned char *)temp_buffer,MAX_FILE_BUFF);
    }*/
    
    bt_activity = COMPLETE;
  }
  else if (bt_activity == COMPLETE)
  {
//    Flash_Close();
    bt_activity = false;
  }  
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_procede_map()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw Arduino Firmware Upload Screen ///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_firmware()
{
  GFX_ClearMainWindow();

  NTSC_Term_Print("Wireless iDrivino firmware Update/");
  
  //Switch relay to Bluetooth serial mode
  digitalWrite(RS232_BT_SEL,LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_firmware()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw Logging Screen ///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_logging()
{
  GFX_ClearMainWindow();
  
  NTSC_Term_Print("Drawing logging/");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_logging()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw CAN Sniffer //////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_sniffer()
{
  char buff_out[16] = {0};
  int buffer[36];  
  
  GFX_ClearMainWindow();

  detachInterrupt(0);
  if (Canbus.init(CANSPEED_100A,FILTER_OFF))  // Initialize MCP2515 CAN controller at the specified speed
  {
    /*Serial.print("$$$");
    delay(100);
    Serial.println("SU,115");
    delay(100);
    Serial.println("---<CR>");
    delay(100);*/
    Serial.end();
    delay(50);
    Serial.begin(115200);
    delay(50);
    Serial.flush();
    NTSC_Term_Print("CANBUS sniffer running on COM port.../");
    NTSC_Term_Print("Reboot iDrivino to exit/");
    sniff_start = millis();
  }
  else
  {
    NTSC_Term_Print("CANBUS problem, sniffer not running!/");
  }
  
  //while loop effectively locks up iDrivino from further use, requiring reboot to get
  //  out of CAN sniffer mode.  Just sit here and output CAN messages to the serial port...
  while (1)
  {
    //Check for new CAN messages
    if(Canbus.ecu_req(buffer) == 1)
    {
      sprintf(buff_out,"%dms, ",(millis()-sniff_start));
      Serial.print(buff_out);
      
      if(buffer[0] <=255)                     
      {
        Serial.print("0x0");
      } // write lead character
      else
      {
        Serial.print("0x");
      } // write lead character
        
      Serial.print(buffer[0],HEX); // ID
      Serial.print(" ");
      Serial.print(buffer[10]); //Length
      Serial.print(" ");
      for (int x=1; x<=buffer[10]; x++) // Only write the number of data bytes available
      {
        Serial.print(buffer[x]);
        Serial.print(" ");
      }
      Serial.println(" "); // New line
    }
    
    delay(1);
  } //end while(1)
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_sniffer()
{
  //Now unused
  /*if (Canbus.message_rx((unsigned char*)read_buffer) == 1)
  {
    canbus_to_serial(&read_buffer[0]);
  }*/
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw V1 Screen ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_v1()
{
  GFX_ClearMainWindow();

  NTSC_Term_Print("Drawing V1/");  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_v1()
{

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw Menu /////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_bottom_menu()
{
  //This function is only called when active screen changes, so tab highlight also changes accordingly
  if (active_screen >= CONFIG_SET)
    GFX_Bottom_Menu(CONFIG_SET,-99);
  else
    GFX_Bottom_Menu(active_screen,-99);
  
  prev_map_sel = 9;
  prev_at_boost_lvl = 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update  ////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_bottom_menu()
{
  int oil_temp;

  curr_map_sel = lookup_single_param(PROCEDE_MAP_SELECT);
  
  if (prev_map_sel != curr_map_sel)
  {
    //Update Map #
    GFX_Bottom_Menu(-1,curr_map_sel);
    prev_map_sel = curr_map_sel;
  }

  if (curr_map_sel == 0)
  {
    curr_at_boost_lvl = 880;
  }
  else
  {
    oil_temp = lookup_single_param(PROCEDE_OIL_TEMP);
    
    //Procede goes into boost protection at these temps
    if ((oil_temp < 170) || (oil_temp > 260))
    {
      //curr_at_boost_lvl = 880;
      //Instead of showing stock boost of ~8.8psi, show the % when full boost is activated
      curr_at_boost_lvl = oil_temp/170.0*10000;
    }
    else
    {
      curr_at_boost_lvl = lookup_single_param(PROCEDE_AT_BOOST_LVL);
    }    
  }
  
  if (prev_at_boost_lvl != curr_at_boost_lvl)
  {
    //Update AT boost setting
    GFX_Bottom_Menu(-99,curr_at_boost_lvl);
    prev_at_boost_lvl = curr_at_boost_lvl;
  }
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw Highlight /////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw_highlight(byte prev_window_focus)
{

  if ((prev_window_focus >= MENU_BAR) && (window_focus < MENU_BAR))
  {
    GFX_Highlight(0,window_focus,-99);
    GFX_Highlight(active_screen,window_focus,0);
  }
  else if ((prev_window_focus >= MENU_BAR) && (window_focus >= MENU_BAR))
  {
    GFX_Highlight(active_screen,window_focus,-99);
  }
  else if ((prev_window_focus < MENU_BAR) && (window_focus < MENU_BAR))
  {
    GFX_Highlight(active_screen,window_focus,0);
  }
  else if ((prev_window_focus < MENU_BAR) && (window_focus >= MENU_BAR))
  {
    GFX_Highlight(active_screen,0,0);
    GFX_Highlight(active_screen,window_focus,-99);
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Updated Parameter and Convert to Integer //////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean update_all_params()
{
  byte checksum,read_temp;
  byte i,j,k,c;
  byte retries = 1;
  byte timeout;
  byte send_array[5];
  int procede_raw_temp[PROCEDE_MAX_PARAMS] = {0};
  //char buff[20];
  
  //Send request for data from Procede
  //Example: 0x06 DF 01 00 E6
  // where
  //    0x06	get parameter request
  //    DF	counter
  //	01 00	send command
  //	E6	checksum/CRC, where sum of all previous wds = E6.  Checksum = E6 so subtraction results in 0 when 8 bits masked
  for (k=0;k<retries;k++)
  {
    Serial.flush();
    
    checksum = 0;
    send_array[0] = PROCEDE_REQ;
    send_array[1] = ++procede_counter;
    send_array[2] = 0x01;
    send_array[3] = 0x00;
    checksum = PROCEDE_REQ + procede_counter + 0x01;
    send_array[4] = checksum;
    Serial.write(send_array,5);

    i = 0;
    j = 0;
    timeout = 100;
    
    _delay_ms(10);
  
    while (timeout > 0)
    {
      checksum = 0;
      while (Serial.available())
      {
        if (Serial.available() > 0)
        { 
          read_temp = Serial.read();
          checksum += read_temp;

          // Wait until header info has been read: 0x06 0x#cntr 0x#wds 0x00
          if ((i > 3) && (j < (PROCEDE_MAX_PARAMS)))
          {
            if ((i%2) == 0)
            {
              procede_raw_temp[j] = read_temp << 8;
            }
            else
            {
              procede_raw_temp[j] += read_temp;
              j++;
            }
          }
          i++;
        }
      }        
      
      checksum -= read_temp;
      
      if (checksum != read_temp)
      {
        break;
      }
      else
      {
        //Copy temp values over to procede_raw since checksums match
        for (c=0;c<PROCEDE_MAX_PARAMS;c++)
        {
          procede_raw[c] = procede_raw_temp[c];
        }
        return true;
      }
      
      _delay_ms(1);
      timeout--;
    }
  }
  
  return false;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Updated Parameter and Convert to Integer //////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void sim_all_params()
{
  for (byte i=0;i<PROCEDE_MAX_PARAMS;i++)
  {
    switch (procede_array[i])
    {
      case PROCEDE_RPM:
        if (procede_raw[i] >= 8000)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 0)
          sim_dir[i] = 1;
        procede_raw[i] += 100*sim_dir[i];
        break;
      case PROCEDE_THROTTLE://     0x02
      case PROCEDE_LOAD_INDEX://    0x05
      case PROCEDE_DBW_THROTTLE:// 0x23
        //if (procede_raw[i] >= 1023)
        if (procede_raw[i] >= 1000)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 0)
          sim_dir[i] = 1;
        procede_raw[i] += 50*sim_dir[i];
        break;
      case PROCEDE_BOOST://        0x03
        if (procede_raw[i] >= 2737)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 0)
          sim_dir[i] = 1;
        procede_raw[i] += 50*sim_dir[i];
        break;
      case PROCEDE_ENG_LOAD_IN: //  0x0B
      case PROCEDE_ENG_LOAD_OUT: // 0x0C
      case PROCEDE_AFR: //          0x13
      case PROCEDE_FUEL_PRESS: //   0x15
      case PROCEDE_CURR_AGGR_LVL:
      case PROCEDE_METH_INJ_PCT:
      case PROCEDE_DME_BOOST_TGT:
      case PROCEDE_FUEL_CORR://    0x42
      case PROCEDE_AT_IGN_CORR://  0xF1 //debug byte 2
      case PROCEDE_AT_STATUS:
        procede_raw[i] = 0;
        break;
      case PROCEDE_DME_CODES://    0x25
        procede_raw[i] = 5555;
        break;
      case PROCEDE_DME_IGN_ADV://  0x24
      case PROCEDE_ACT_IGN_ADV://  0x27
      case PROCEDE_IGN_ADV://      0x82
      case PROCEDE_IGN_CORR://     0x44
      case PROCEDE_AT_BOOST_LVL:// 0xF4 //debug byte 5
      case PROCEDE_KNOCK_RETARD:
        if (procede_raw[i] > 256)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 0)
          sim_dir[i] = 1;
        procede_raw[i] += 5*sim_dir[i];
        break;
      case PROCEDE_SPEED://        0x0E
        if (procede_raw[i] > 1023)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 0)
          sim_dir[i] = 1;
        procede_raw[i] += 10*sim_dir[i];
        break;
      case PROCEDE_MAP_SELECT://   0x11
        if (procede_raw[i] >= 9)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 0) 
          sim_dir[i] = 1;
        procede_raw[i] += sim_dir[i];
        break;
      case PROCEDE_IAT: //          0x12
        if (procede_raw[i] > 403)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 253)
          sim_dir[i] = 1;
        procede_raw[i] += 10*sim_dir[i];
        break;
      case PROCEDE_AFR_BANK1://    0x20 //CAN KNOCK
      case PROCEDE_AFR_BANK2: //    0x22
        if (procede_raw[i] > 2048)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 0)
          sim_dir[i] = 1;
        procede_raw[i] += 10*sim_dir[i];
        break;
      case PROCEDE_OIL_TEMP://     0x21
        if (procede_raw[i] > 5460)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 2500)
          sim_dir[i] = 1;
        procede_raw[i] += 50*sim_dir[i];
        break;
      case PROCEDE_COOLANT_TEMP://      0x81
        if (procede_raw[i] > 300)
          sim_dir[i] = -1;
        else if (procede_raw[i] <= 0)
          sim_dir[i] = 1;
        procede_raw[i] += 10*sim_dir[i];
        break;
      default:
        break;
    } //end switch
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Updated Parameter and Convert to Integer //////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int lookup_single_param(short lookup)
{
  boolean found = false;
  
  for (byte i=0;i<PROCEDE_MAX_PARAMS;i++)
  {
    switch (procede_array[i])
    {
      case PROCEDE_RPM:
        if (lookup == PROCEDE_RPM)
        {
          tempint = procede_raw[i];
          found = true;
        }
        break;
      case PROCEDE_THROTTLE://     0x02
        if (lookup == PROCEDE_THROTTLE)
        {
          tempfloat = procede_raw[i]/1023.0 * 100.0;
          tempint = (int)(tempfloat+0.5);
          found = true;
        }
        break;
      case PROCEDE_BOOST://        0x03
        if (lookup == PROCEDE_BOOST)
        {
          tempfloat = (procede_raw[i]-1013.0)/1000.0*14.504;
          //boost_psi = tempfloat;
          //tempfloat = boost_psi;
          tempint = (int)(tempfloat*100.0);    //*100
          if ((tempint < 0) && (tempint > -10.))
            tempint = 0;
          found = true;
        }
        break;
      case PROCEDE_LOAD_INDEX://    0x05
        if (lookup == PROCEDE_LOAD_INDEX)
        {
          tempfloat = procede_raw[i]/1023.0 * 100.0;
          tempint = (int)(tempfloat+0.5);
          found = true;
        }
        break;
      case PROCEDE_ENG_LOAD_IN: //  0x0B
        if (lookup == PROCEDE_ENG_LOAD_IN)
        {
          tempint = 0;
          found = true;
        }
        break;
      case PROCEDE_ENG_LOAD_OUT: // 0x0C
        if (lookup == PROCEDE_ENG_LOAD_OUT)
        {
          tempint = 0;
          found = true;
        }
        break;
      case PROCEDE_SPEED://        0x0E
        if (lookup == PROCEDE_SPEED)
        {
          tempfloat = procede_raw[i]*634.0/1023.0;
          tempint = (int)(tempfloat*100.0);    //*100
          found = true;
        }
        break;
      case PROCEDE_MAP_SELECT://   0x11
        if (lookup == PROCEDE_MAP_SELECT)
        {
          tempint = procede_raw[i];
          found = true;
        }
        break;
      case PROCEDE_IAT: //          0x12
        if (lookup == PROCEDE_IAT)
        {
          tempint = (procede_raw[i]-273.0)*1.8+32.0;
          found = true;
        }
        break;
      case PROCEDE_AFR: //          0x13
        if (lookup == PROCEDE_AFR)
        {
          tempint = 0;
          found = true;
        }
        break;
      case PROCEDE_FUEL_PRESS: //   0x15
        if (lookup == PROCEDE_FUEL_PRESS)
        {
          tempfloat = procede_raw[i]*5.0/255.0;
          tempint = (int)(tempfloat*100.0);      //*100
          found = true;
        }
        break;
      case PROCEDE_GEAR_CHANGE://    0x16
        if (lookup == PROCEDE_GEAR_CHANGE)
        {
          tempint = procede_raw[i];
          found = true;
        }
        break;
      case PROCEDE_METH_INJ_FLOW://    0x18
        if (lookup == PROCEDE_METH_INJ_FLOW)
        {
          tempfloat = procede_raw[i]/255.0*100.0;
          tempint = (int)(tempfloat+0.5);
          found = true;
        }
        break;
      case PROCEDE_AFR_BANK1://    0x20 //CAN KNOCK
        if (lookup == PROCEDE_AFR_BANK1)
        {
          tempfloat = procede_raw[i]*29.4/2048.0;
          tempint = (int)(tempfloat*100.0);      //*100
          if (tempint > 2940)
            tempint = 2940;
          found = true;
        }
        break;
      case PROCEDE_OIL_TEMP://     0x21
        if (lookup == PROCEDE_OIL_TEMP)
        {
          tempfloat = procede_raw[i]*0.18 - 459.4;
          tempint = (int)(tempfloat+0.5);
          found = true;
        }
        break;
      case PROCEDE_AFR_BANK2: //    0x22
        if (lookup == PROCEDE_AFR_BANK2)
        {
          tempfloat = procede_raw[i]*29.4/2048.0;
          tempint = (int)(tempfloat*100.0);      //*100
          if (tempint > 2940)
            tempint = 2940;
          found = true;
        }
        break;
      case PROCEDE_DBW_THROTTLE:// 0x23
        if (lookup == PROCEDE_DBW_THROTTLE)
        {
          tempfloat = procede_raw[i]/1023.0 * 100.0;
          tempint = (int)(tempfloat+0.5);
          found = true;
        }
        break;
      case PROCEDE_DME_IGN_ADV://  0x24
        if (lookup == PROCEDE_DME_IGN_ADV)
        {
          tempfloat = procede_raw[i]/4.0;
          tempint = (int)(tempfloat*100.0);     //*100
          found = true;
        }
        break;
      case PROCEDE_DME_CODES://    0x25
        if (lookup == PROCEDE_DME_CODES)
        {
          tempint = procede_raw[i]; //0-65535
          //DME codes cycle thru at 2 sec intervals
          found = true;
        }
        break;
      case PROCEDE_DME_BOOST_TGT://  0x26
        if (lookup == PROCEDE_DME_BOOST_TGT)
        {
          tempfloat = (procede_raw[i]-1013.0)/1000.0*14.504;
          tempint = (int)(tempfloat*100.0);      //*100
          found = true;
        }
        break;
      case PROCEDE_ACT_IGN_ADV://  0x27
        if (lookup == PROCEDE_ACT_IGN_ADV)
        {
          tempfloat = procede_raw[i]*64.0/256.0;
          tempint = (int)(tempfloat*100.0);      //*100
          found = true;
        }
        break;
      case PROCEDE_AT_STATUS://  0x30
        if (lookup == PROCEDE_AT_STATUS)
        {
          tempint = procede_raw[i];  //either 1 (ON) or 0 (OFF)
          found = true;
        }
        break;
      case PROCEDE_KNOCK_RETARD://  0x31
        if (lookup == PROCEDE_KNOCK_RETARD)
        {
          tempint = procede_raw[i]*63.75/255.0;
          found = true;
        }
        break;
      case PROCEDE_CURR_AGGR_LVL://  0x32
        if (lookup == PROCEDE_CURR_AGGR_LVL)
        {
          tempfloat = procede_raw[i]*255.0/1023.0;
          tempint = (int)(tempfloat*100.0);
          found = true;
        }
        break;
      case PROCEDE_AT_IGN_CORR://   0x34
        if (lookup == PROCEDE_AT_IGN_CORR)
        {
          tempfloat = procede_raw[i]*100.0/128.0;
          tempint = (int)(tempfloat+0.5);
          found = true;
        }
        break;
      case PROCEDE_AT_BOOST_LVL://  0x35
        if (lookup == PROCEDE_AT_BOOST_LVL)
        {
          tempfloat = (procede_raw[i]/255.0*10.0) + 10.0;
          tempint = (int)(tempfloat*100.0);      //*100
          found = true;
        }
        break;
      case PROCEDE_FUEL_CORR://    0x42
        if (lookup == PROCEDE_FUEL_CORR)
        {
          tempfloat = procede_raw[i]*200.0/510.0;
          tempint = (int)(tempfloat*100.0);      //*100
          found = true;
        }
        break;
      case PROCEDE_IGN_CORR://     0x44
        if (lookup == PROCEDE_IGN_CORR)
        {
          tempfloat = procede_raw[i]*0.25;
          tempint = (int)(tempfloat*100.0);      //*100
          found = true;
        }
        break;
      case PROCEDE_COOLANT_TEMP://      0x81
        if (lookup == PROCEDE_COOLANT_TEMP)
        {
          tempfloat = procede_raw[i]*404.6/300.0 - 54;
          tempint = (int)(tempfloat+0.5);
          found = true;
        }
        break;
      case PROCEDE_IGN_ADV://      0x82
        if (lookup == PROCEDE_IGN_ADV)
        {
          tempfloat = procede_raw[i]*64.0/256.0;
          tempint = (int)(tempfloat+0.5);
          found = true;
        }
        break;
      case PROCEDE_UPSHIFT://  0x8d
        if (lookup == PROCEDE_UPSHIFT)
        {
          tempint = procede_raw[i];
          found = true;
        }
        break;
      case PROCEDE_STORED_VIN://  0x8e
        if (lookup == PROCEDE_STORED_VIN)
        {
          tempint = procede_raw[i];
          found = true;
        }
        break;
      case PROCEDE_CURRENT_VIN://  0x8f
        if (lookup == PROCEDE_CURRENT_VIN)
        {
          tempint = procede_raw[i];
          found = true;
        }
        break;
      case PROCEDE_METH_INJ_PCT://  0x91
        if (lookup == PROCEDE_METH_INJ_PCT)
        {
          tempint = procede_raw[i];
          found = true;
        }
        break;
      case EXTERNAL_BOOST:
        if (lookup == EXTERNAL_BOOST)
        {
          tempint = (int)(boost_psi*100.0);    //*100
          if ((tempint < 0) && (tempint > -10.))
            tempint = 0;
          found = true;
        }
        break;
      default:
        break;
    } //end switch
    
    //Exit for loop if parameter is found & calculated
    if (found == true)
      return tempint;
  }

  return -999;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start to Stop Timer ///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long start_stop_timer(unsigned short start_speed, unsigned short end_speed)
{  
  unsigned short temp_start = start_speed;
  
  //Check if timer is currently active
  if (timer_active == true)
  {
    //Update timer results
    end_time = millis() - start_time;

    if (lookup_single_param(PROCEDE_SPEED) >= (end_speed*100))
    {
      timer_active = false;
      GFX_Print_SetXY(114,112);
      GFX_Print_XY("TIMING",0);
    }
  }
  else if (timer_active == ARMED)
  {
    if (lookup_single_param(PROCEDE_SPEED) > (start_speed*100))
    {
      start_time = millis();
      end_time = 0;
      
      GFX_Print_SetXY(118,112);
      GFX_Print_XY("ARMED",0);
      GFX_Print_SetXY(114,112);
      GFX_Print_XY("TIMING",2);
      
      timer_active = true;
    }
  }
  else
  {
    //Special condition where if start = 0mph, bug that occasionally reports 0.4 or 0.7mph
    if (start_speed == 0)
      temp_start = 0.8;
    
    //Check if starting conditions met  
    if (lookup_single_param(PROCEDE_SPEED) <= (temp_start*100))
    {
      //Arm and alert user timer is ready
      timer_active = ARMED;
      
      //Display message "when ready, go WOT"
      GFX_Print_SetXY(118,112);
      GFX_Print_XY("ARMED",2);
      
      end_time = 0;
    }
    else
    {
      //Exit this function to be called again in main loop
      timer_active = false;
      
      //do not reset end_time so last timer reading will persist
    }
  }

  return end_time;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SD Mem Card Check /////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void sd_check()
{
/*  sd_present = false;
  
  Serial.println("in sd check");
  
  //Check if memory card is installed
  if (sdFile.initialize())
  {
    Serial.println("SD init ok");
    
    //Read last known configuration parameters
    if (sdFile.open_file("config.txt"))
    {
      //File found
      NTSC_Term_Print("config file found/");
      Serial.println("config file found");
      sd_present = true;
    }
    else
    {
      //No file found, create a new one
      if (sdFile.create_file("config.txt"))
      {
        NTSC_Term_Print("config file not found, new file created/");
        Serial.println("config file not found, new file created");
        sd_present = true;
      }
    }
  }
  
  if (sd_present == true)
  {
    sdFile.close();
  }
  else
  {
    NTSC_Term_Print("SD problem or card missing!/");
    Serial.println("SD problem or card missing!/");
  }
  
*/  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SD  /////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void sd_datalog()
{

  // make a string for assembling the data to log:
/*  String dataString = "";

  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    //int sensor = analogRead(analogPin);
    int sensor = lookup_single_param(PROCEDE_RPM);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ","; 
    }
  }

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
*/
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Standard OBD2 messages ///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*float read_std_obd2(unsigned short param)
{

  if (CAN_ACTIVE == false)
    return -999;

  if (Canbus.ecu_req(param,can_buffer) == 1)
  {
    return atof(can_buffer);
  }
  else
    return -999;

}*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Procede Parameters ///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean procede_init()
{
  int timeout;
  byte i,j,k;
  byte read_temp,checksum;
  boolean statusp = false;
  byte retries = 10;
  //char buff[20];
  byte send_array[20];
  
  //15 maximum parameters from Procede
  //  Anything over & including index 15 is not a Procede read param
  procede_array[0] = PROCEDE_RPM;
  procede_array[1] = PROCEDE_BOOST;
  procede_array[2] = PROCEDE_ACT_IGN_ADV;
  procede_array[3] = PROCEDE_IGN_CORR;
  procede_array[4] = PROCEDE_DBW_THROTTLE;
  procede_array[5] = PROCEDE_SPEED;
  procede_array[6] = PROCEDE_MAP_SELECT;
  procede_array[7] = PROCEDE_IAT;
  procede_array[8] = PROCEDE_METH_INJ_FLOW;
  procede_array[9] = PROCEDE_OIL_TEMP;
  procede_array[10] = PROCEDE_DME_IGN_ADV;
  procede_array[11] = PROCEDE_AT_BOOST_LVL;
  procede_array[12] = PROCEDE_DME_CODES;
  procede_array[13] = PROCEDE_AFR_BANK1;
  procede_array[14] = PROCEDE_AFR_BANK2;
  procede_array[15] = EXTERNAL_BOOST;  //ADC 0

/*#define PROCEDE_RPM          0x01
#define PROCEDE_THROTTLE     0x02
#define PROCEDE_LOAD_INDEX   0x03
#define PROCEDE_BOOST        0x05
#define PROCEDE_ENG_LOAD_IN  0x0B
#define PROCEDE_ENG_LOAD_OUT 0x0C
#define PROCEDE_SPEED        0x0E
#define PROCEDE_MAP_SELECT   0x11
#define PROCEDE_IAT          0x12
#define PROCEDE_AFR          0x13
#define PROCEDE_FUEL_PRESS   0x15
#define PROCEDE_AFR_BANK1    0x20 //CAN KNOCK
#define PROCEDE_OIL_TEMP     0x21
#define PROCEDE_AFR_BANK2    0x22
#define PROCEDE_DBW_THROTTLE 0x23
#define PROCEDE_DME_IGN_ADV  0x24
#define PROCEDE_DME_CODES    0x25
#define PROCEDE_KNOCK_COUNT  0x26
#define PROCEDE_ACT_IGN_ADV  0x27
#define PROCEDE_FUEL_CORR    0x42
#define PROCEDE_IGN_CORR     0x44
#define PROCEDE_COOLANT_TEMP 0x81
#define PROCEDE_IGN_ADV      0x82
#define PROCEDE_AT_IGN_CORR  0xF1 //debug byte 2
#define PROCEDE_AT_ACTIVE    0xF2 //debug byte 3
#define PROCEDE_AT_BOOST_LVL 0xF4 //debug byte 5
#define PROCEDE_AGGR_LVL     0xF5 //debug byte 6
#define PROCEDE_METH_INJECT  0xF6 //debug byte 7
#define PROCEDE_BOOST_TARGET 0xFC //debug word 5
#define PROCEDE_PSET         0x05
#define PROCEDE_REQ          0x06
#define PROCEDE_MAX_PARAMS   16
*/
  
  NTSC_Term_Print("Connecting to Procede ECU");
  Serial.flush();
  
  for (i=0;i<retries;i++)
  {
    NTSC_Term_Print("..");
    
    timeout = 60;
    read_temp = 0;
    
    Serial.write(0xff);
    
    while (timeout > 0)
    {
      //if (procedeRS232.available() > 0)
      if (Serial.available() > 0)
      {
//        NTSC_Term_Print("init=");
//        Serial.print("init=");
        
        //read_temp = procedeRS232.read();
        read_temp = Serial.read();
//        sprintf(buff," 0x%x/",read_temp);
//        NTSC_Term_Print(buff);
//        Serial.println(buff);
        //}
      }
      if (read_temp == 0xff) break;

      _delay_ms(1);
      timeout--;
    }  //end while
        
    //sprintf(buff,"calc chk = %x vs =%x/",checksum,read_temp);
    //NTSC_Term_Print(buff);
    if (read_temp == 0xff)
    {
      procede_counter = 0;
      NTSC_Term_Print("ok/");
//      Serial.println("Procede reset");
//      _delay_ms(1000);
      statusp = true;
      break;
    }    
    _delay_ms(200);
  }
  
  if (statusp == false) 
    NTSC_Term_Print("fail/");
  
  _delay_ms(10);
  NTSC_Term_Print("Querying Procede ID");
  
  //statusp = false;
  for (i=0;i<retries;i++)
  {
    NTSC_Term_Print("..");
    
    Serial.flush();
    j = 0;
    timeout = 100;
    read_temp = -1;
    
    send_array[0] = 0x01;
    send_array[1] = 0x00;
    send_array[2] = 0x01;
    Serial.write(send_array,3);
    
//    NTSC_Term_Print("Sending 0x01 0x00 0x01/");
//    Serial.println("Sending 0x01..");

    while (timeout > 0)
    {
      checksum = 0;
      //while (procedeRS232.available())
      while (Serial.available())
      {
//        NTSC_Term_Print("id=");
//        Serial.print("id=");
        
        //if (procedeRS232.available() > 0)
        if (Serial.available() > 0)
        {
          //read_temp = procedeRS232.read();
          read_temp = Serial.read();
//          sprintf(buff," 0x%x,%d",read_temp,read_temp);
//          NTSC_Term_Print(buff);
//          Serial.println(buff);

          checksum += read_temp;
          j++;
          
//          sprintf(buff,"chk %d",checksum);
//          NTSC_Term_Print(buff);
//          Serial.println(buff);
          
          //Expecting 0x01 00 03 65 68 1D EE(checksum)
        }
      }
      checksum -= read_temp;
//      checksum &= 0xff;
      if (j == 7) break;
      
      _delay_ms(1);
      timeout--;
    }  //end while
    
/*    if (j > 0)
    {
//      sprintf(buff,"chk = %x vs =%x/",checksum,read_temp);
//      NTSC_Term_Print(buff);
//      Serial.println(buff);
    }
    else
    {
      NTSC_Term_Print("No data/");
//      Serial.println("No data");
    }
*/

    if (checksum == read_temp)
    {
      procede_counter = 0;
      NTSC_Term_Print("ok/");
//      Serial.println("Procede ident");
//      _delay_ms(500);
      statusp = true;
      break;
    }
    //_delay_ms(200);
  }
  
  if (statusp == false) 
    NTSC_Term_Print("fail/");
  
  _delay_ms(10);
    /*Build parameter Setup request list
    
    Parameter Request Example:
      0x05 DE 06 00 01 05 42 44 03 78
        where:
  	0x05	parameter request code
  	DE	counter
  	06	number of params requested
  	00	unused, always zero
  	01	RPM
  	05	Boost
  	42	Fuel correction
  	44	Ignition correction
  	03	?	
  	78	checksum/CRC, where sum of all previous wds = 178.  Checksum = 78 so 178-78 results in 0 when 8 bits masked
  
  05 ct 10 00 01 05 27 44 23 0e 11 12 81 21 02 24 f4 25 20 chk
  05 01 10 00 01 05 27 44 23 0e 11 12 81 21 02 24 f4 25 20 dc
    */
  
  NTSC_Term_Print("Uploading Procede parameters");

  statusp = false;
  for (i=0;i<retries;i++)
  {
    NTSC_Term_Print("..");
    Serial.flush();
    
    checksum = 0;    
    send_array[0] = PROCEDE_PSET;
    checksum += PROCEDE_PSET;
    send_array[1] = ++procede_counter;
    checksum += procede_counter;
    send_array[2] = PROCEDE_MAX_PARAMS;  //0x00 is included in byte count, so max params is actually 15 + 1 = 16
    checksum += PROCEDE_MAX_PARAMS; 
    send_array[3] = 0x00;
    for (byte k=0;k<(PROCEDE_MAX_PARAMS-1);k++)
    {
      send_array[k+4] = procede_array[k];
      checksum += procede_array[k];
    }
    send_array[19] = checksum;
    
/*    for (k=0;k<20;k++)
    {
      //procedeRS232.print(send_array[k],BYTE);
      sprintf(buff,"s 0x%x",send_array[k]);
      NTSC_Term_Print(buff);
//      Serial.println(buff);
    }
*/
    Serial.write(send_array,20);
  
    j = 0;
    timeout = 100;
    read_temp = -1;
    
    _delay_ms(10);
    
    while (timeout > 0)
    {
      checksum = 0;
      //while (procedeRS232.available())
      while (Serial.available())
      {
//        NTSC_Term_Print("param=");
//        Serial.print("param=");

        //if (procedeRS232.available() > 0)
        if (Serial.available() > 0)
        {
          //read_temp = procedeRS232.read();
          read_temp = Serial.read();
          
//          sprintf(buff,"r%d 0x%x/",j,read_temp);
//          NTSC_Term_Print(buff);
//          Serial.println(buff);

          checksum += read_temp;
          j++;
          
          //sprintf(buff,"chk %d",checksum);
          //NTSC_Term_Print(buff);
//          Serial.println(buff);
        }
      }
      checksum -= read_temp;
//      checksum &= 0xff;
      
      if (j == (PROCEDE_MAX_PARAMS+4))
      {
        break;
      }
      _delay_ms(1);
      timeout--;
    }
    
/*    if (j > 0)
    {
      sprintf(buff,"chk = %x vs = %x/",checksum,read_temp);
      NTSC_Term_Print(buff);
//      Serial.println(buff);
    }
    else
    {
      NTSC_Term_Print("No data/");
//      Serial.println("No data");
    }
*/

    if (checksum == read_temp)
    {
      statusp = true;
      NTSC_Term_Print("ok/");
      NTSC_Term_Print("Procede ready!/");
      procede_online = true;
//      Serial.println("Procede ready!");
      break;
    }
    _delay_ms(200);
  }
  
  if (statusp == false)
  {
    NTSC_Term_Print("fail/");
    procede_online = false;
  }
  
  _delay_ms(1000);
  
  return statusp;
  
 }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read CAN bus for Specific Command ID //////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void find_can_id(unsigned short cmd_to_find)
{
  if (CAN_ACTIVE == false)
    return;

  //find_flag = true;
  //find_id = cmd_to_find;
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update screen focus depending on iDrive knob motion ///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_screen_from_iDrive()
{
  byte tempadd = 0;
  byte prev_window_focus = window_focus;
//  char buff[20];

  if (menu_changed == true)
  {
    //Mark timestamp to keep 
    idrive_timer = millis();

    //Main windows navigation
    if (window_focus == 0)
    {
      //Determine what the iDrive knob is trying to do:
      switch (menu_command)
      {
        case KNOB_UP:
          window_focus = MAIN_WINDOW;
          break;
        case KNOB_DOWN:
        case KNOB_RIGHT:
        case KNOB_LEFT:
          if (active_screen >= CONFIG_SET)
            tempadd = CONFIG_SET;
          else
            tempadd = active_screen;
          window_focus = MENU_BAR+tempadd-10;
          break;
        case KNOB_PRESS:
        case KNOB_ROTATE_RIGHT:
        case KNOB_ROTATE_LEFT:
        case MENU_BUTTON:
          break;
        default:
          break;
      }
    }
    else if (window_focus >= MENU_BAR) //Menu bar navigation
    {
      switch (menu_command)
      {
        case KNOB_UP:
          window_focus = MAIN_WINDOW;
          break;
        case KNOB_RIGHT:
          if (active_screen >= CONFIG_SET)
            tempadd = ONE_GAUGE;
          else if (active_screen == FOUR_GAUGE)
            tempadd = PARAM_LIST;
          else if (active_screen == TIMER_XY)
            tempadd = CONFIG_SET;
          else
            tempadd = active_screen+1;
          window_focus = MENU_BAR+tempadd-10;
          active_screen = tempadd;
          break;
        case KNOB_LEFT:
          if (active_screen == ONE_GAUGE)
            tempadd = CONFIG_SET;
          else if (active_screen == PARAM_LIST)
            tempadd = FOUR_GAUGE;
          else if (active_screen == CONFIG_SET)
            tempadd = TIMER_XY;
          else
            tempadd = active_screen-1;
          window_focus = MENU_BAR+tempadd-10;
          active_screen = tempadd;
          break;
        case KNOB_PRESS:
        case KNOB_ROTATE_RIGHT:
        case KNOB_ROTATE_LEFT:
        case MENU_BUTTON:
        case KNOB_DOWN:
          break;
        default:
          break;
      }
    }
    else if ((window_focus >= MAIN_WINDOW) && (window_focus < MENU_BAR))
    {
      update_main_highlight();
    }

    //Draw new highlight item
    draw_highlight(prev_window_focus);
    if (active_screen >= CONFIG_SET)
    {
      update_tools(prev_window_focus);
    }
    
    //sprintf(buff,"prev=%d, curr=%d",prev_window_focus,window_focus);
    //Serial.println(buff);
    
    //Reset menu_command flags
    menu_command = 0;
    menu_changed = false;
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check for highlight in upper main window //////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_main_highlight()
{
  short element = 0;
  
  switch (active_screen)
  {
    case ONE_GAUGE:
      switch (menu_command)
      {
        case KNOB_DOWN:
          window_focus = MENU_BAR+ONE_GAUGE-10;
          break;
        case KNOB_ROTATE_RIGHT:
          ++one_g;
          if (one_g >= MaxTypeG)
            one_g = 0;

          draw_one();
          break;
        case KNOB_ROTATE_LEFT:
          --one_g;
          if (one_g < 0)
            one_g = MaxTypeG-1;

          draw_one();
          break;
        case KNOB_UP:
        case KNOB_RIGHT:
        case KNOB_LEFT:
        default:
          break;
      }        
      break;
    case TWO_GAUGE:
      switch (menu_command)
      {
        case KNOB_DOWN:
          window_focus = MENU_BAR+TWO_GAUGE-10;
          break;
        case KNOB_ROTATE_RIGHT:
          if (window_focus == MAIN_WINDOW)
            element = 0;
          else 
            element = 1;
          
          ++two_g[element];
          if (two_g[element] >= MaxTypeG)
            two_g[element] = 0;

          draw_two();
          break;
        case KNOB_ROTATE_LEFT:
          if (window_focus == MAIN_WINDOW)
            element = 0;
          else 
            element = 1;
          
          --two_g[element];
          if (two_g[element] < 0)
            two_g[element] = MaxTypeG-1;

          draw_two();
          break;
        case KNOB_RIGHT:
          if (window_focus == MAIN_WINDOW)
            window_focus++;
          break;
        case KNOB_LEFT:
          if (window_focus == MAIN_WINDOW+1)
            window_focus--;
          break;
        case KNOB_UP:
        default:
          break;
      }
      break;
    case FOUR_GAUGE:
      switch (menu_command)
      {
        case KNOB_DOWN:
          if (window_focus == MAIN_WINDOW)
            window_focus = MAIN_WINDOW+2;
          else if (window_focus == MAIN_WINDOW+1)
            window_focus = MAIN_WINDOW+3;
          else //if (window_focus == MAIN_WINDOW+2) or if (window_focus == MAIN_WINDOW+3)
            window_focus = MENU_BAR+TWO_GAUGE-10;
          break;
        case KNOB_UP:
          if (window_focus == MAIN_WINDOW+2)
            window_focus = MAIN_WINDOW;
          else if (window_focus == MAIN_WINDOW+3)
            window_focus = MAIN_WINDOW+1;
          break;
        case KNOB_RIGHT:
          if (window_focus == MAIN_WINDOW)
            window_focus = MAIN_WINDOW+1;
          else if (window_focus == MAIN_WINDOW+2)
            window_focus = MAIN_WINDOW+3;
          break;
        case KNOB_LEFT:
          if (window_focus == MAIN_WINDOW+1)
            window_focus = MAIN_WINDOW;
          else if (window_focus == MAIN_WINDOW+3)
            window_focus = MAIN_WINDOW+2;
          break;
        case KNOB_ROTATE_RIGHT:
          if (window_focus == MAIN_WINDOW)
            element = 0;
          else if (window_focus == MAIN_WINDOW+1)
            element = 1;
          else if (window_focus == MAIN_WINDOW+2)
            element = 2;
          else if (window_focus == MAIN_WINDOW+3)
            element = 3;
          
          ++four_g[element];
          if (four_g[element] >= MaxTypeG)
            four_g[element] = 0;

          draw_four();
          break;
        case KNOB_ROTATE_LEFT:
          if (window_focus == MAIN_WINDOW)
            element = 0;
          else if (window_focus == MAIN_WINDOW+1)
            element = 1;
          else if (window_focus == MAIN_WINDOW+2)
            element = 2;
          else if (window_focus == MAIN_WINDOW+3)
            element = 3;
          
          --four_g[element];
          if (four_g[element] < 0)
            four_g[element] = MaxTypeG-1;

          draw_four();
          break;
        default:
          break;
      }
      break;
    case PARAM_LIST:
      switch (menu_command)
      {
        case KNOB_DOWN:
          window_focus = MENU_BAR+TIMER_XY-10;
          break;
        default:
          break;
      }
      break;
    case TIMER_XY:
      switch (menu_command)
      {
        case KNOB_DOWN:
          window_focus = MENU_BAR+TIMER_XY-10;
          break;
        case KNOB_RIGHT:
          if (window_focus == MAIN_WINDOW)
            window_focus = MAIN_WINDOW+1;
          break;
        case KNOB_LEFT:
          if (window_focus == MAIN_WINDOW+1)
            window_focus = MAIN_WINDOW;
          break;
        case KNOB_ROTATE_RIGHT:
          if (window_focus == MAIN_WINDOW)
            ++start_vel;
          else if (window_focus == MAIN_WINDOW+1)
            ++end_vel;
          
          draw_timer();
          break;
        case KNOB_ROTATE_LEFT:
          if (window_focus == MAIN_WINDOW)
            --start_vel;
          else if (window_focus == MAIN_WINDOW+1)
            --end_vel;
          
          draw_timer();
          break;
        case KNOB_UP:
        default:
          break;
      }
      break;
    case CONFIG_SET:
      switch (menu_command)
      {
        case KNOB_ROTATE_LEFT:
          if (window_focus == MAIN_WINDOW+1)
            window_focus = MAIN_WINDOW;
          else if (window_focus == MAIN_WINDOW+2)
            window_focus = MAIN_WINDOW+1;
          else if (window_focus == MAIN_WINDOW+3)
            window_focus = MAIN_WINDOW+2;
          else if (window_focus == MAIN_WINDOW)
            window_focus = MAIN_WINDOW+3;
          break;
        case KNOB_ROTATE_RIGHT:
          if (window_focus == MAIN_WINDOW+1)
            window_focus = MAIN_WINDOW+2;
          else if (window_focus == MAIN_WINDOW+2)
            window_focus = MAIN_WINDOW+3;
          else if (window_focus == MAIN_WINDOW+3)
            window_focus = MAIN_WINDOW;
          else if (window_focus == MAIN_WINDOW)
            window_focus = MAIN_WINDOW+1;
          break;
        case KNOB_DOWN:
          window_focus = MENU_BAR+CONFIG_SET-10;
          break;
        case KNOB_RIGHT:
          if (window_focus == MAIN_WINDOW+1)
            update_procede_map();
          else if (window_focus == MAIN_WINDOW+2)
            active_screen = CAN_SNIFFER;
          else if (window_focus == MAIN_WINDOW+3)
            digitalWrite(BT_RESET_PIN, HIGH);
          else if (window_focus == MAIN_WINDOW)
            update_procede_fw();
          break;
        default:
          break;
      }
      break;
    case DATA_LOG:
    case VALENTINE1:
      break;
    default:
      break;
  }
  
  //Serial.println("exit updt highlt");
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  //////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void right_prompt()
{
  GFX_Print_SetXY(155,112);
  GFX_Print_XY("-> to confirm",1);
  GFX_Box(145,102,100,20);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  //////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*void canbus_to_serial(unsigned char *buff_in)
{
  //char buff_temp[7] = {0};
  char buff_out[16] = {0};
  
  sprintf(buff_out,"%dms, 0x",(int)(millis()-sniff_start));
  Serial.print(buff_out);
  //strcpy(buff_out,buff_temp);

  for (byte i=1;i<buff_in[2]+3;i+=2)
  {
    if (i == 3)
    {
      sprintf(buff_out,"%02x ",buff_in[2]);
      //strcat(buff_out,buff_temp);
      Serial.print(buff_out);
      i = 2;
    }
    else if (i > 18)
    {
      //truncate message if greater than 16 data words (19 total words)
      //Serial.println("");
      break;
    }
    else
    {
      sprintf(buff_out,"%02x %02x ",buff_in[i],buff_in[i-1]);
      Serial.print(buff_out);
      //strcat(buff_out,buff_temp);
    }
    
    //sprintf(buff_out,"%x ",buff_in[i]);
    
    if (can_count > 0)
    {
      can_count = 0;
      break;
    }
  }
  
  Serial.println("");

}*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check for highlighted window //////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_for_highlight_timeout()
{
  
  if ((millis() - idrive_timer) > 3000) //&& (window_focus != 0))
  {
    //Unhighlight all selections because user timed out
    draw_highlight(0);
    idrive_timer = 0;
    if (window_focus > 0)
    {
      draw_bottom_menu();
      window_focus = 0;
    }
    else
    {
      window_focus = 0;
    }
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read CANBUS message if interrupt fired ////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_canbus_message()
{
  unsigned int rotate_num = 0;
  
  if (fresh_rx == true)
  {
    //Read CAN message
    if (Canbus.message_rx((unsigned char*)read_buffer) == 0)
    {
      //NTSC_Term_Print("?");
      fresh_rx = false;
      return;
    }
    
    //Route CAN message to serial (BT)
    /*if (active_screen == CAN_SNIFFER)
    {
      for (byte i=0;i<19;i++)
      {
        //read_buffer_temp[can_count][i] = read_buffer[i];
        read_buffer_temp[i] = read_buffer[i];
      }
      canbus_to_serial(&read_buffer_temp[0]);
      //Serial.print(can_count);
      if (can_count < 9) 
      {
        //can_count++;
      }
    }*/
    
    can_cmd = (read_buffer[1] << 8) + read_buffer[0];
    //sprintf(buff,"buf1 = 0x%x, buf0 = 0x%x, sum=0x%x",read_buffer[1],read_buffer[0],can_cmd);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"2 0x%x/",read_buffer[2]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"3 0x%x/",read_buffer[3]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"4 0x%x/",read_buffer[4]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"5 0x%x/",read_buffer[5]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"6 0x%x/",read_buffer[6]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"7 0x%x/",read_buffer[7]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"8 0x%x/",read_buffer[8]);
    //NTSC_Term_Print(buff);
  
    //http://translate.google.com/translate?u=http%3A%2F%2Fwww.xolmatic.com%2Fxprojects%2FXE65%2FCAN.htm&sl=es&tl=en&hl=&ie=UTF-8  
    //Look for specific commands, anything else exit ISR
    switch (can_cmd)
    {
      //iDrive knob commands
      //I-drive Menu button: 1B8 6 0FC5nnnn206F (or 0FC4nnnn206F, sometimes C4 sometimes C5)
      //I-drive knob press: 1B8 6 0FC1nnnn206F
      //I-drive knob up: 1B8 6 00C0nnnn206F
      //I-drive knob down: 1B8 6 04C0nnnn206F
      //I-drive knob left: 1B8 6 06C0nnnn206F
      //I-drive knob right: 1B8 6 02C0nnnn206F
      //I-drive knob rotate right: 1B8 6 0FC0nnnn206F (nn is increasing)
      //I-drive knob rotate left: 1B8 6 0FC0nnnn206F (nn is decreasing)
      //  nnnn min = 0000
      //  nnnn max = FF|FF (where buffer 5 is lower and 6 is the higher byte)
       
      case IDRIVE_KNOB:  //Command word 0x1B8
        //Knob rotation or return to center
        if ((read_buffer[3] == 0x0F) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          rotate_num = (read_buffer[6] << 8) + read_buffer[5];

          //I-drive knob rotate right: 1B8 6 0FC0nnnn206F (nn is increasing)
          //if (read_buffer[5] > last_rotate)
          if (rotate_num > last_rotate)          
          {
            menu_command = KNOB_ROTATE_RIGHT;
            if (window_focus != 0)
              menu_changed = true;
            //NTSC_Term_Print("Knob r right/");
            //Serial.println("Knob r rt");
          }
          //I-drive knob rotate left: 1B8 6 0FC0nnnn206F (nn is decreasing)
          //else if (read_buffer[5] < last_rotate)
          else if (rotate_num < last_rotate)
          {
            menu_command = KNOB_ROTATE_LEFT;
            if (window_focus != 0)
              menu_changed = true;
            //NTSC_Term_Print("Knob r left/");
            //Serial.println("Knob r lf");
          }
          //I-drive knob return to center since no rotation increase
          else
          {
            menu_changed = true;
            //NTSC_Term_Print("debnc");
            //Serial.println("Return");
          }
          //last_rotate = read_buffer[5];
          last_rotate = rotate_num;
        }
        //I-drive knob up: 1B8 6 00C0nnnn206F
        else if ((read_buffer[3] == 0x00) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_UP;
          //menu_changed = true;
          //NTSC_Term_Print("Knob up/");
          //Serial.println("Knob up");
        }
        //I-drive knob down: 1B8 6 04C0nnnn206F
        else if ((read_buffer[3] == 0x04) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_DOWN;
          //menu_changed = true;
          //NTSC_Term_Print("Knob down/");
          //Serial.println("Knob down");
        }
        //I-drive knob right: 1B8 6 02C0nnnn206F
        else if ((read_buffer[3] == 0x02) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_RIGHT;
          //menu_changed = true;
          //NTSC_Term_Print("Knob right/");
          //Serial.println("Knob right");
        }
        //I-drive knob left: 1B8 6 06C0nnnn206F
        else if ((read_buffer[3] == 0x06) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_LEFT;
          //menu_changed = true;
          //NTSC_Term_Print("Knob left/");
          //Serial.println("Knob left");
        }
        //I-drive knob press: 1B8 6 0FC1nnnn206F
        else if ((read_buffer[3] == 0x0F) && (read_buffer[4] == 0xC1)  //0xc0 is press up
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_PRESS;
          //menu_changed = true;
          //NTSC_Term_Print("Knob press/");
          //Serial.println("Knob press");
        }
        //I-drive Menu button: 1B8 6 0FC5nnnn206F (or 0FC4nnnn206F, sometimes C4 sometimes C5)
        else if ((read_buffer[3] == 0x0F) && ((read_buffer[4] == 0xC5) || (read_buffer[4] == 0xC4))
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = MENU_BUTTON;
          //menu_changed = true;
          //NTSC_Term_Print("Menu btn/");
          //Serial.println("Menu btn");
        }
        break;
      //Other custom BMW commands, put here
      //  ....
      
      default:
        menu_command = 0;
        menu_changed = false;
        break;
    }
    
    fresh_rx = false;
  }
  
}  

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check for CAN Shield interrupts knob changes //////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Canbus_ISR()
{

  if (CAN_ACTIVE == false)
  {
    fresh_rx = false;
    return;
  }
  
  fresh_rx = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check for car shutdown  ///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int check_for_shutdown()
{
  static int counter = 0;
  
  if ((lookup_single_param(PROCEDE_RPM) == 0) && (engine_start == true))
  {
    //Save config settings here
    save_flash_config_file();
    engine_start = false;
    counter++;
  }
  else if ((lookup_single_param(PROCEDE_RPM) > 100) && (engine_start == false))
  {
    engine_start = true;
    counter = 0;
  }
  else if ((engine_start == false) && (boost_psi < -2.0))
  {
    //counter++;
  }

  return counter;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check for Reinitialization after running CAN Sniffer //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_for_reinit()
{
  if ((prev_active_screen == CAN_SNIFFER) && (active_screen != CAN_SNIFFER))
  {
    Serial.end();
    delay(100);
    Serial.begin(57600);
    Canbus.init(CANSPEED_100A,FILTER_ON);
    attachInterrupt(0,Canbus_ISR,FALLING);
  }
  
  if (((prev_active_screen >= CONFIG_SET) && (prev_active_screen <= CAN_SNIFFER)) && (active_screen < CONFIG_SET))
  {
    //Serial.begin(57600);
    digitalWrite(RS232_BT_SEL,HIGH);  //set relay to RS232
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reset iDrivino for BT firmware upload /////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void download_btsketch() 
{
  char buff[24];
  byte i;
  
  Serial.println("Click upload 5 secs from now, iDrivino will then reboot!");
  for (i=5;i>0;i--)
  {
    sprintf(buff,"Hit upload in %d secs",i);
    Serial.println(buff);
    delay(1000);
  }
  Serial.println("UPLOAD NOW!!!");
  for (i=16;i>0;i--)
  {
    sprintf(buff,"Rebooting in %d secs",i);
    Serial.println(buff);
    delay(1000);
  }
  digitalWrite(BT_RESET_PIN, HIGH);
}


void verify_config()
{

  //Check if the active screen was valid  
  if ((active_screen < ONE_GAUGE) || (active_screen > CONFIG_SET))
  {
    active_screen = ONE_GAUGE;
    one_g = BoostG;
  }

  if ((start_vel < 0) || (start_vel > 255))
  {  
    start_vel = 0;
  }
  
  if ((end_vel < 0) || (end_vel > 255))
  {  
    end_vel = 60;
  }
  
  //For simulation
  if (procede_online == false)
  {
    procede_array[0] = PROCEDE_RPM;
    procede_array[1] = PROCEDE_BOOST;
    procede_array[2] = PROCEDE_ACT_IGN_ADV;
    procede_array[3] = PROCEDE_IGN_CORR;
    procede_array[4] = PROCEDE_DBW_THROTTLE;
    procede_array[5] = PROCEDE_SPEED;
    procede_array[6] = PROCEDE_MAP_SELECT;
    procede_array[7] = PROCEDE_IAT;
    procede_array[8] = PROCEDE_METH_INJ_FLOW;
    procede_array[9] = PROCEDE_OIL_TEMP;
    procede_array[10] = PROCEDE_DME_IGN_ADV;
    procede_array[11] = PROCEDE_AT_BOOST_LVL;
    procede_array[12] = PROCEDE_DME_CODES;
    procede_array[13] = PROCEDE_AFR_BANK1;
    procede_array[14] = PROCEDE_AFR_BANK2;
    procede_array[15] = EXTERNAL_BOOST;
  }
  
}

void procede_fw_upload()
{
}
void procede_map_upload()
{
}
void run_sniffer()
{
}

void command_memorycheck()
{
  void* HP = malloc(4);
  if (HP)
    free (HP);
    
  unsigned long free = (unsigned long)SP - (unsigned long)HP;
  
  Serial.print("Heap=");
  Serial.println((unsigned long)HP,HEX);
  Serial.print("Stack=");
  Serial.println((unsigned long)SP,HEX);
  Serial.print("Free=");
  Serial.println((unsigned long)free,HEX);
}
