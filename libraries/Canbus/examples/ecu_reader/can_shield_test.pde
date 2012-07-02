/* Welcome to the ECU Reader project. This sketch uses the Canbus library.
It requires the CAN-bus shield for the Arduino. This shield contains the MCP2515 CAN controller and the MCP2551 CAN-bus driver.
A connector for an EM406 GPS receiver and an uSDcard holder with 3v level convertor for use in data logging applications.
The output data can be displayed on a serial LCD.

SK Pang Electronics www.skpang.co.uk

v1.0 28-03-10

*/

#include <NewSoftSerial.h>
#include <Canbus.h>

NewSoftSerial sLCD =  NewSoftSerial(3, 14); /* Serial LCD is connected on pin 14 (Analog input 0) */
#define COMMAND 0xFE
#define CLEAR   0x01
#define LINE0   0x80
#define LINE1   0xC0

#define UP     15
#define DOWN   17
#define LEFT   16

#include <byteordering.h>
#include <fat.h>
#include <FAT16.h>
#include <fat_config.h>
#include <partition.h>
#include <partition_config.h>
#include <sd-reader_config.h>
#include <sd_raw.h>
#include <sd_raw_config.h>


FAT TestFile;      //This will be the file we manipulate in the sketch
char buffer[512];  //Data will be temporarily stored to this buffer before being written to the file
int read_size=0;   //Used as an indicator for how many characters are read from the file
int count=0;       //Miscellaneous variable

int D10 = 10;

int LED2 = 8;
int LED3 = 7;

//char buffer[16];

NewSoftSerial mySerial =  NewSoftSerial(4, 5);

#define COMMAND 0xFE
//#define powerpin 4

#define GPSRATE 4800
//#define GPSRATE 38400


// GPS parser for 406a
#define BUFFSIZ 90 // plenty big
//char buffer[BUFFSIZ];
char *parseptr;
char buffidx;
uint8_t hour, minute, second, year, month, date;
uint32_t latitude, longitude;
uint8_t groundspeed, trackangle;
char latdir, longdir;
char status;
 
void setup() {
    Serial.begin(GPSRATE);
  mySerial.begin(GPSRATE);
  pinMode(LED2, OUTPUT); 
  pinMode(LED3, OUTPUT); 
   digitalWrite(LED2, HIGH);
  pinMode(UP,INPUT);
  pinMode(DOWN,INPUT);
    pinMode(LEFT,INPUT);
  Serial.begin(9600);
  Serial.println("ECU Reader");  /* For debug use */
  
  sLCD.begin(9600);              /* Setup serial LCD and clear the screen */
  sLCD.print(COMMAND,BYTE);
  sLCD.print(CLEAR,BYTE);
 
  sLCD.print("Select CAN or GPS");
  
  while(1)
  {
    
    if (digitalRead(UP) == 0){
      Serial.println("gps");
      sLCD.print("GPS");
      gps_test();
    }
    
    if (digitalRead(DOWN) == 0) {
      sLCD.print("CAN");
      Serial.println("CAN");
      break;
    }
    
   if (digitalRead(LEFT) == 0) {
      sLCD.print("SD");
      Serial.println("SD");
      sd_test();
    }
    
    
    
    
    
  }
   sLCD.print(COMMAND,BYTE);
  sLCD.print(CLEAR,BYTE);
  
  if(Canbus.init(CANSPEED_500))  /* Initialise MCP2515 CAN controller at the specified speed */
  {
    sLCD.print("CAN Init ok");
  } else
  {
    sLCD.print("Can't init CAN");
  } 
   
  delay(1000); 
  
 //   pinMode(13, OUTPUT);

 
  }
 

void loop() {
    
  if(Canbus.ecu_req(ENGINE_RPM,buffer) == 1)          /* Request for engine RPM */
  {
    sLCD.print(COMMAND,BYTE);                   /* Move LCD cursor to line 0 */
    sLCD.print(LINE0,BYTE);
    sLCD.print(buffer);                         /* Display data on LCD */ 
  } 
  digitalWrite(LED3, HIGH);
   
  if(Canbus.ecu_req(VEHICLE_SPEED,buffer) == 1)
  {
    sLCD.print(COMMAND,BYTE);
    sLCD.print(LINE0 + 9,BYTE);
    sLCD.print(buffer);
  }
  
  if(Canbus.ecu_req(ENGINE_COOLANT_TEMP,buffer) == 1)
  {
    sLCD.print(COMMAND,BYTE);
    sLCD.print(LINE1,BYTE);                     /* Move LCD cursor to line 1 */
    sLCD.print(buffer);
  }
  
  if(Canbus.ecu_req(THROTTLE,buffer) == 1)
  {
    sLCD.print(COMMAND,BYTE);
    sLCD.print(LINE1 + 9,BYTE);
    sLCD.print(buffer);
  }  
//  Canbus.ecu_req(O2_VOLTAGE,buffer);
      
   digitalWrite(LED3, LOW); 
   delay(50); 

}
void sd_test(void)
{
pinMode(D10, OUTPUT);    /* Make sure D10 is an output otherwise SPI will not work */
  digitalWrite(D10, HIGH);
  pinMode(LED2, OUTPUT);    
  
  Serial.begin(9600);  //Initiate serial communication at 9600 bps
  Serial.println("ready...");
  
  TestFile.initialize();  //Initialize the SD card and the FAT file system.
  Serial.println("Starting...");
  TestFile.create_file("skpang2.txt");  //Create a file on the SD card named "Read_File_Test.txt"
  
  //NOTE: This function will return a 0 value if it was unable to create the file.
  TestFile.open();  //Now that the file has been created, open it so we can write to it.

while(1)
{

 TestFile.write("This is test data.");  //using the write function will always write to the beginning of the file. Here we add some text to the file.
  TestFile.close();  //We are done writing to the file for now. Close it for later use.
  
  while(1){
    digitalWrite(LED2, HIGH);
    TestFile.open();  //Open the file. When the file is opened we will be looking at the beginning of the file.
    read_size=TestFile.read(buffer); //Read the contents of the file. This will only read the amount of data specified by the size of 'buffer.'

    Serial.println(read_size, DEC);  //Print the number of characters read by the read function.
    for(int i=0; i<read_size; i++)   
    {
      Serial.print(buffer[i], BYTE);  //Print out the contents of the buffer.
    }
    Serial.println();

    sprintf(buffer, "%d", count++); //Now we'll use the buffer to write data back to the file. Here's we'll only add one value to buffer, the 'count' variable. 
    TestFile.write(buffer);         //Write the new buffer to the end of the file
    TestFile.close();               //Close the file for later use.

    delay(500);  //Wait one second before repeating the loop.
    digitalWrite(LED2, LOW);
    delay(500);  //Wait one second before repeating the loop.
  }




}


}

void gps_test(void){
  uint32_t tmp;
  while(1){
  
  Serial.print("\n\rread: ");
  readline();
  
  // check if $GPRMC (global positioning fixed data)
  if (strncmp(buffer, "$GPRMC",6) == 0) {
    
    // hhmmss time data
    parseptr = buffer+7;
    tmp = parsedecimal(parseptr); 
    hour = tmp / 10000;
    minute = (tmp / 100) % 100;
    second = tmp % 100;
    
    parseptr = strchr(parseptr, ',') + 1;
    status = parseptr[0];
    parseptr += 2;
    
    // grab latitude & long data
    // latitude
    latitude = parsedecimal(parseptr);
    if (latitude != 0) {
      latitude *= 10000;
      parseptr = strchr(parseptr, '.')+1;
      latitude += parsedecimal(parseptr);
    }
    parseptr = strchr(parseptr, ',') + 1;
    // read latitude N/S data
    if (parseptr[0] != ',') {
      latdir = parseptr[0];
    }
    
    //Serial.println(latdir);
    
    // longitude
    parseptr = strchr(parseptr, ',')+1;
    longitude = parsedecimal(parseptr);
    if (longitude != 0) {
      longitude *= 10000;
      parseptr = strchr(parseptr, '.')+1;
      longitude += parsedecimal(parseptr);
    }
    parseptr = strchr(parseptr, ',')+1;
    // read longitude E/W data
    if (parseptr[0] != ',') {
      longdir = parseptr[0];
    }
    

    // groundspeed
    parseptr = strchr(parseptr, ',')+1;
    groundspeed = parsedecimal(parseptr);

    // track angle
    parseptr = strchr(parseptr, ',')+1;
    trackangle = parsedecimal(parseptr);


    // date
    parseptr = strchr(parseptr, ',')+1;
    tmp = parsedecimal(parseptr); 
    date = tmp / 10000;
    month = (tmp / 100) % 100;
    year = tmp % 100;
    
    Serial.print("\nTime: ");
    Serial.print(hour, DEC); Serial.print(':');
    Serial.print(minute, DEC); Serial.print(':');
    Serial.println(second, DEC);
    Serial.print("Date: ");
    Serial.print(month, DEC); Serial.print('/');
    Serial.print(date, DEC); Serial.print('/');
    Serial.println(year, DEC);
    
      sLCD.print(COMMAND,BYTE);
     sLCD.print(0x80,BYTE);
   sLCD.print("La");
   
    Serial.print("Lat"); 
    if (latdir == 'N')
    {
       Serial.print('+');
          sLCD.print("+");
    }
    else if (latdir == 'S')
     {  Serial.print('-');
          sLCD.print("-");
     }
    Serial.print(latitude/1000000, DEC); Serial.print('\°', BYTE); Serial.print(' ');
    
    sLCD.print(latitude/1000000, DEC); sLCD.print(0xDF, BYTE); sLCD.print(' ');
    
    Serial.print((latitude/10000)%100, DEC); Serial.print('\''); Serial.print(' ');
     sLCD.print((latitude/10000)%100, DEC); sLCD.print('\''); //sLCD.print(' ');
     
    Serial.print((latitude%10000)*6/1000, DEC); Serial.print('.');
    sLCD.print((latitude%10000)*6/1000, DEC); sLCD.print('.');
    
    Serial.print(((latitude%10000)*6/10)%100, DEC); Serial.println('"');
    sLCD.print(((latitude%10000)*6/10)%100, DEC); sLCD.print('"');
    
     sLCD.print(COMMAND,BYTE);
     sLCD.print(0xC0,BYTE);
   sLCD.print("Ln");
   
    
    
    Serial.print("Long: ");
    if (longdir == 'E')
    {
       Serial.print('+');
       sLCD.print('+');
    }
    else if (longdir == 'W')
    { 
       Serial.print('-');
       sLCD.print('-');
    }
    Serial.print(longitude/1000000, DEC); Serial.print('\°', BYTE); Serial.print(' ');
     sLCD.print(longitude/1000000, DEC); sLCD.print(0xDF, BYTE); sLCD.print(' ');
     
    Serial.print((longitude/10000)%100, DEC); Serial.print('\''); Serial.print(' ');
  sLCD.print((longitude/10000)%100, DEC); sLCD.print('\''); //sLCD.print(' ');
  
    Serial.print((longitude%10000)*6/1000, DEC); Serial.print('.');
    sLCD.print((longitude%10000)*6/1000, DEC); sLCD.print('.');
    
    Serial.print(((longitude%10000)*6/10)%100, DEC); Serial.println('"');
   sLCD.print(((longitude%10000)*6/10)%100, DEC); sLCD.print('"');
   
   
  }
  //Serial.println(buffer);


  }



}

void readline(void) {
  char c;
  
  buffidx = 0; // start at begninning
  while (1) {
      c=mySerial.read();
      if (c == -1)
        continue;
      Serial.print(c);
      if (c == '\n')
        continue;
      if ((buffidx == BUFFSIZ-1) || (c == '\r')) {
        buffer[buffidx] = 0;
        return;
      }
      buffer[buffidx++]= c;
  }
}
uint32_t parsedecimal(char *str) {
  uint32_t d = 0;
  
  while (str[0] != 0) {
   if ((str[0] > '9') || (str[0] < '0'))
     return d;
   d *= 10;
   d += str[0] - '0';
   str++;
  }
  return d;
}
