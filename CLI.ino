/*
*   This file provides the command line interface bewteen the METRO M4 processor and teh NB-IoT module 
*   as well as a debug USB Arduinio port
*/

#include <stdio.h>
#include <arduino.h> //<string.h>
extern "C" {
  #include "gps.h"
}
 
#define UNHANDLED_CMD " Unhandled Command"
#define NOT_IMPLEMENTED_CMD " Not Implemented "

#define MAX_CLI_BUFFER_SIZE 2500 //512
#define MAX_GPS_BUFFER_SIZE 512

boolean disconnect_nbiot();  // its in the main file.
boolean connect_nbiot();  // its in the main file.


char DEBUG_inputStrCLI[MAX_CLI_BUFFER_SIZE];
char MODEM_inputStrCLI [MAX_CLI_BUFFER_SIZE];
char GPS_inputStrCLI[MAX_GPS_BUFFER_SIZE];  

boolean DEBUG_newCLICommand = false;
boolean MODEM_newCLICommand = false;
boolean GPS_newCLICommand = false;

/*
 * DEBUG_  is the USB serial port that can accept control commands as well as AT style commands that are routed straight
 * through to the MODEM port.
 * MODEM_ is the serial port attached to the NB_IoT module through which noise and vibration data will pass through
 * It also carries or relays AT commands that are issued from a serial terminal on the DEBUG port
 * Finally, telemetry commands from the NBIoT module are accepted and parsed.
 */


int getSystemStatus()
{
  return (int)0; // dummy
}

void get_serial_no(){
}

void get_bserial_no(char *bsn) {  // B1 board serial number
unsigned long crc;
int address;
int i = 0;
/*
if (check_crc()) {
        for (address = EPROM_OFFSET_BSN; address < (EPROM_BSN_LEN+EPROM_OFFSET_BSN) ;address++) {
             *(bsn+i) = EEPROM.read(address);
            i++; 
           }
           *(bsn+EPROM_BSN_LEN)='/0'; // ensure last character in string is actually NULL
                   
        }
        else
Serial.print("Invalid CRC");
*/
}

// function to program strings up to 16 character long into EEPROM from an offset
void program_eeprom(int offset, char data[16] ) {
  int i=0;
  int address;
  unsigned long crc;
  int strlength;
/*
  strlength = strlen(data); 
  for (address = offset; address < offset+strlength+1; address++) {
    EEPROM.put(address, data[i]);
    i++;
 */ 
  }


void DEBUG_resetCLI(){
  DEBUG_inputStrCLI[0]='\0';
  DEBUG_newCLICommand=false; 
}

void MODEM_resetCLI(){
  MODEM_inputStrCLI[0]='\0';
  MODEM_newCLICommand=false; 
}

void GPS_resetCLI(){
  GPS_inputStrCLI[0]='\0';
  GPS_newCLICommand=false;
}

void setupCLI()
{  
  //bReportMODEM=false;
  DEBUG_resetCLI();
  MODEM_resetCLI();
  GPS_resetCLI();
}





void DEBUG_serialEvent(){   // Debug USB port serial event.
   int count=0;
   while (DEBUG_STREAM.available() && (count < MAX_CLI_BUFFER_SIZE)) {
    // get the new byte:
    char inChar = (char)DEBUG_STREAM.read();
     // add it to the inputString:
    DEBUG_inputStrCLI[count]= inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    count++;
    if (inChar == '\r') {
      DEBUG_newCLICommand = true;
      DEBUG_parseCLI();
    } 
  }
}

void MODEM_serialEvent(){  // MODEM UART serial portto NB-IoT module.
  if (digitalRead(powerPin) == 1) {
    int count=0;
   while ( (MODEM_STREAM.available()) && (count < MAX_CLI_BUFFER_SIZE) ) {
    char inChar = (char)MODEM_STREAM.read();
    MODEM_inputStrCLI[count] = inChar;
    //Serial.print(inChar);  //echo chars
     //newCLICommand = true;
     count++;
    if (inChar == '\r') {
      MODEM_newCLICommand = true;      
      MODEM_parseCLI(); 
    } 
  }
}
}

#ifdef GPS_VERSION
void GPS_serialEvent(){  // MODEM UART serial portto NB-IoT module.
  int count=0;
   while ((GPS_STREAM.available()) && (count < MAX_GPS_BUFFER_SIZE-1)) {
    char inChar = (char)GPS_STREAM.read();
    GPS_inputStrCLI[count] = inChar;
   // DEBUG_STREAM.print(inChar);  //echo chars
    count++ ;
    if (inChar == '\r') {
      GPS_newCLICommand = true;
    } 
  }
  GPS_inputStrCLI[count] = '\0';
  //DEBUG_STREAM.print("GPS event char count=");DEBUG_STREAM.println(count);
}
#endif

void DEBUG_parseCLI(){
  char *ptr, *ptr1;
  char buf[10];
  float fmic_cal;
  uint16_t mic_cal_int, mic_cal_fract;
  int tmp;
  DEBUG_STREAM.flush();

  char command=0;
  int batval=0;
  int chargerval=0;
 
  command = DEBUG_inputStrCLI[0];
 
  switch(command){

    case '?':
            DEBUG_STREAM.println(F("s - Start Noise & Vibration Measuring"));
            DEBUG_STREAM.println(F("p - Stop Noise & Vibration Measuring"));
    	      DEBUG_STREAM.println(F("c  - Enable Calibration mode for Noise")); 
            DEBUG_STREAM.println(F("c0  - Disable Calibration mode for Noise")); 
            DEBUG_STREAM.println(F("cal <>  set mic_cal between 5 and 40dB, Eg cal 27 ")); 
            DEBUG_STREAM.println(F("a - Diagnostics"));
            DEBUG_STREAM.println(F("b - Batt Lvl"));
            DEBUG_STREAM.println(F("v - SW Ver"));
            DEBUG_STREAM.println(F("vh - HW Ver"));
            DEBUG_STREAM.println(F("gs - Product SN#"));
            DEBUG_STREAM.println(F("i - Status"));
            DEBUG_STREAM.println(F("f -  Factory Rst"));
            DEBUG_STREAM.println(F("e <m> <0 or 1> where <m> = -n Enable Noise, -v Enable Vibration, -a Enable Noise&Vibration"));    
            DEBUG_STREAM.println(F("AT <>  AT commands are directed to the mode port   "));
            DEBUG_STREAM.println(F("dd -  <>  Deregister from NB-IoT network and turn off radio  "));
            DEBUG_STREAM.println(F("de -  <>  Power on radio, Connect and register on NB-IoT Network."));
            DEBUG_STREAM.println(F("dt -  <>  Get time and date fromtheNB-IoT Network"));
            DEBUG_STREAM.println(F("t -  <>  Print out the time and date of the internal clock"));
                 
            break;

case 'c':           					
          DEBUG_STREAM.println(DEBUG_inputStrCLI);
          /* Calibration code section*/

          if (DEBUG_inputStrCLI[1]=='a' && DEBUG_inputStrCLI[2] =='l') {
             mic_cal_fract=0;
             mic_cal_int = (int)strtol(&DEBUG_inputStrCLI[4],&ptr ,10); // gets integer part
             if (*ptr++ == '.') // there is a decimal point
                mic_cal_fract = (int)strtol(ptr, &ptr1 ,10); // gets fractional part

             if (mic_cal_fract < 10)   
                mic_cal = (float)mic_cal_int + (float)(mic_cal_fract)/10;
             else if (mic_cal_fract < 100)   
                mic_cal = (float)mic_cal_int + (float)(mic_cal_fract)/100;
             else if (mic_cal_fract >= 100) {
                DEBUG_STREAM.println(F("**ERROR mic cal value only supports 2 places of decimal"));
                //mic_cal = (float)mic_cal_int;
             }
             
             
             DEBUG_STREAM.print("mic_cal changed to (dB):"); DEBUG_STREAM.println(mic_cal);
             if((mic_cal < 50) && (mic_cal > 5)){ // check for reasonable range
               Wire.begin(); // enable i2c
               Wire.beginTransmission(0x57);
               Wire.write(byte(0x04)); // sub address
            //   tmp = (conv2bcd(mic_cal) >> 8);
             //  DEBUG_STREAM.println(tmp);
             //  tmp =(conv2bcd(mic_cal) & 0x00ff);
             //  DEBUG_STREAM.println(tmp);
               
               Wire.write((byte)(conv2bcd(mic_cal) >> 8)); // default Mic Cal value in dB upper byte
               Wire.write((byte)(conv2bcd(mic_cal) & 0x00ff)); // default Mic Cal value in dB lower byte       
               Wire.write(byte(0x01)); // 0x00 = uncalibrated, 0x01 = Calibrated
               Wire.endTransmission();
               delay(170);
               Wire.end();
               DEBUG_STREAM.println(F("EEPROM updated with new mic_cal"));
              }
             else
              DEBUG_STREAM.println(F("mic_cal must be < 50 and > 5 so not updating EEPROM"));

             break;
          }
          
          if (DEBUG_inputStrCLI[1]=='0'){ // turn off calibration
                   calibration_enabled = 0;
                   DEBUG_STREAM.println(F("Calibration off"));
                   break;
          }
          else{
             //disconnect_nbiot();
             connected_flag = false;
             calibration_enabled = 1;  // turn on calibration.  can only be turned off by resetting or powering off
             DEBUG_STREAM.println(F("Calibration Mode Enabled"));
             }
         break;
          
    case 'p':
      enable_measurements = 0;
      DEBUG_STREAM.println(F("Enable measurements stopped"));
          break;
          
    case 'A':
     
    if ((DEBUG_inputStrCLI[1] == 'T') &&  (digitalRead(powerPin) == 1)){
       MODEM_STREAM.print(DEBUG_inputStrCLI);
      
    }
	  //    if(doDiagnostics())
		//	doDiagmosticsReadSensors();
		//	DEBUG_STREAM.println(getDeadzone());
          break;
          
    case 's':
     enable_measurements = 1;
      DEBUG_STREAM.println(F("Enable measurements started"));
          break;
          
    case 'b': 
    
      get_charger_stat();
      get_charger_mode();
      DEBUG_STREAM.print("Charge cycles % = ");DEBUG_STREAM.println(get_batt_mon_cycles());
      DEBUG_STREAM.print(F("Battery %="));DEBUG_STREAM.println(get_batt_percent());
      DEBUG_STREAM.print(F("Battery Voltage="));DEBUG_STREAM.println(get_batt_voltage());
      DEBUG_STREAM.print(F("Battery Current mA="));DEBUG_STREAM.println(get_batt_current());
          break;
          
     case 'o':
      /*   // Cant use ADCs unless we stop vibration, reuse its ADC to measure voltage and then reinitialise.
     *    THis will require a lot of code and testing.
     *    
       chargerval = analogRead(CHARGING);
       chargerval = analogRead(CHARGING); // needs a 2nd read as the analog mux needs to settle down
       DEBUG_STREAM.println(chargerval);
       */
       DEBUG_STREAM.print(F("This feature not currently supported"));
       break;
     
    case 'v':
         if (DEBUG_inputStrCLI[1]=='h') {
          DEBUG_STREAM.println(F("HW Version not currently supported"));
      /*
          get_hw_ver();
           DEBUG_STREAM.println(hw_version);          
         }
         else {
            #ifdef FACTORY_COMPILE
               DEBUG_STREAM.print(FACTORY_VERSION_NUM); DEBUG_STREAM.print(" "); DEBUG_STREAM.println(PROTOCOL_VERSION_NUM);
            #endif
            #ifndef FACTORY_COMPILE     
               DEBUG_STREAM.print(VERSION_NUM); DEBUG_STREAM.print(" "); DEBUG_STREAM.println(PROTOCOL_VERSION_NUM);
            #endif
        */
         }
         else
         DEBUG_STREAM.println(SW_VERSION);
         
          break;
                      
    case'f':
          DEBUG_STREAM.println("Resetting battery params in EEPROM");
          wipe_batt_mon_params(); // wipes eeprom battery params flag to make it invalid.
          DEBUG_STREAM.println("Switch off unit and Remove battery cable to clear battery monitor IC and then reconnect battery");
             
          break;
          
    case 'i': 
        getSystemStatus();  
        DEBUG_STREAM.print(F("i "));   DEBUG_STREAM.println(getSystemStatus(),HEX);
          break;
            
   
    case 'e':
	   {
	  char lcommand[10];
    char cparam[10];
		int index=-1;
		int data=-1;
		int parmNo=0;
		int pos;
        char strCLI[20];  
        
		//DEBUG_inputStrCLI.toCharArray(strCLI, sizeof(strCLI));
    strncpy(strCLI, DEBUG_inputStrCLI,sizeof(strCLI));
        
//		parmNo= sscanf( strCLI, "%s %d  %d", lcommand,  &index, &data );
    parmNo= sscanf( strCLI, "%s %s %d ", lcommand, cparam, &data);
		DEBUG_STREAM.print("NoParms=");DEBUG_STREAM.print(parmNo);
		DEBUG_STREAM.print(" : Command=");DEBUG_STREAM.print(lcommand);
//		DEBUG_STREAM.print(" : Index=");DEBUG_STREAM.print(index);
		DEBUG_STREAM.print(" : Data=");DEBUG_STREAM.println(data);
		if(parmNo<1){
		   DEBUG_STREAM.println(F("Insufficient params"));
		   break;
		}
		
//		switch(index){
    switch (cparam[0]){
		    case 'n':
             if (data == 1){
              enable_noise_flag = 1;
             }
              else{
                enable_noise_flag = 0; 
              }
              DEBUG_STREAM.print(F(" Noise monitoring flag ="));DEBUG_STREAM.println(enable_noise_flag);
            break;
            
            case 'v':
			           if (data == 1){
              enable_vibration_flag = 1;
             }
              else{
                enable_vibration_flag = 0; 
              }
               DEBUG_STREAM.print(F(" Vibration monitoring flag ="));DEBUG_STREAM.println(enable_vibration_flag);
            break;
            
            case 'a':
                 if (data == 1){
              enable_vibration_flag = 1;
              enable_noise_flag = 1;
             }
              else{
                enable_vibration_flag = 0; 
                enable_noise_flag = 0;
              }
               DEBUG_STREAM.print(F(" Vibration & Noise monitoring flags ="));DEBUG_STREAM.println(data);
            break;
             case 'e':
                 if (data == 1){
                   bEngineeringMode = 1;
                 }
                 else{
                   bEngineeringMode = 0; 
                 }
                 DEBUG_STREAM.print(F(" Engineering mode ="));DEBUG_STREAM.println(data);
            break;            
                    
			     default:
			        DEBUG_STREAM.println(F(" Invalid params"));
			     break; 
           } 
               
		}
		
   
		        
       case 'g':
       
                char StrCLI[SERIAL_NO_LEN+2];  			
	        //DEBUG_inputStrCLI.toCharArray(StrCLI, sizeof(StrCLI));
          strncpy(StrCLI, DEBUG_inputStrCLI,sizeof(StrCLI));
               if (StrCLI[1]=='s' ){// get product serial number
                 //get_serial_no();
                 DEBUG_STREAM.println(serial_no);
               }
               else
               {
                 if(StrCLI[1]=='b' ){ //get board serial number
                   get_bserial_no(&bserial_no[0]);  //pass by reference
                   
                   DEBUG_STREAM.println(bserial_no);
                 }  
             }

               break;	 
          
      case 'W':  //EEPROM Factory Write -only when in engineering mode (e 1)  KOS 27/1/2016 command is "WR [address] [string]"   No spaces allowed in string    
         if (bEngineeringMode==true && DEBUG_inputStrCLI[1]=='R' ){
          
           char lcommand[10];
			 int offset=-1;
			 int parmNo=0;
			char strCLI[20];
                        char data[16];			
			//DEBUG_inputStrCLI.toCharArray(strCLI, sizeof(strCLI));
      strncpy(strCLI, DEBUG_inputStrCLI,sizeof(strCLI));
			parmNo= sscanf( strCLI, "%s %d %s", lcommand, &offset, &data );

			
			if(parmNo==3){   								// we have 2 parameters e.g. c 1234			
			  if(offset > -1 && offset < 1008 && strlen(data) <16){  
			       program_eeprom(offset, data);
                                DEBUG_STREAM.print(F("Written eeprom from addr: "));
                                DEBUG_STREAM.print(offset);
                                DEBUG_STREAM.print(" with - ");
                                DEBUG_STREAM.println(data);
				                                               } 
                                else {
                                  DEBUG_STREAM.print(F("Str data > 15 chars or addr out of range"));
                                     }
                     }	  
                     else {
                          DEBUG_STREAM.print(F("Wrong no of params, enclose strings dquotes")); 
                          }   
         }
         else
         DEBUG_STREAM.println(F("Disallowed!"));
      
      break;


      case 'd':  // NB-IoT conectivity control
          if (DEBUG_inputStrCLI[1]=='d') {
               disconnect_nbiot();
               connected_flag = 0;
          }
          if (DEBUG_inputStrCLI[1]=='e') {
               connected_flag=1;
               if (connect_nbiot() == false){
                   DEBUG_STREAM.println(F("ERROR: NB-IoT Connection Failure"));
                   connect_state = retry1;
               }
               else
                   connect_state = live;  
          }
          

         if (DEBUG_inputStrCLI[1]=='t') {
           get_time();
           
         }

      break;

      case 't':  // NB-IoT conectivity control
      DEBUG_STREAM.print(day()); DEBUG_STREAM.print("/");
   DEBUG_STREAM.print(month()); DEBUG_STREAM.print("/");
   DEBUG_STREAM.print(year()); DEBUG_STREAM.print("    ");
   DEBUG_STREAM.print(hour()); DEBUG_STREAM.print(":");
   DEBUG_STREAM.print(minute()); DEBUG_STREAM.print(":");
   DEBUG_STREAM.println(second());
      break;
      
         
      case 10:
           DEBUG_STREAM.println(F(">"));
          break; 
              
      case 13:
           DEBUG_STREAM.println(F(">"));
          break;     
             
    default:
      DEBUG_STREAM.print(F(" ["));DEBUG_STREAM.print(command);DEBUG_STREAM.print(F("]"));
      DEBUG_STREAM.println(F("  UnSupported command"));       
  }

 
    DEBUG_resetCLI();
  }


void MODEM_parseCLI(){
    DEBUG_STREAM.print( MODEM_inputStrCLI);  // Just echo out anything from the modem over the debug port

    if (strstr(MODEM_inputStrCLI,"+NSONMI:0,8") != 0){
      MODEM_STREAM.println("AT+NSORF=0,8");  // readback the data and discard in order to keep the received data notifications coming
      if (!validate_OK_resp())
          DEBUG_STREAM.println("*** Readback received data request failed");
    }
     
     MODEM_resetCLI();

  }



  void DEBUG_CLI_Check(){
   if(DEBUG_newCLICommand ==1){
    DEBUG_parseCLI();
  }
}



void GPS_CLI_Check(){
   if(GPS_newCLICommand ==1){
    gps_process(&hgps, GPS_inputStrCLI, strlen((char *) GPS_inputStrCLI));
  //  DEBUG_STREAM.print("GPS parse command: ");DEBUG_STREAM.println(GPS_inputStrCLI);
    GPS_resetCLI();
  }
}

/* converts a floating point number to a BCD encoded 2.2 nibble numner */
uint16_t conv2bcd(float val)
 {
 
  uint8_t nibble[4]; // 4 nibbles expressing xx.xx where x is BCD encoded nibble
  float fract;
  uint16_t result;
  fract = 100* (val - roundf(val-.5));
  nibble[3] = (uint8_t)(val/10);
  nibble[2] = (uint8_t)(val - (10*(int)nibble[3]));
  nibble[1] = (uint8_t)(fract/10);
  nibble[0] = (uint8_t)(fract - (10*(int)nibble[1]));
  result = (nibble[3] << 12) | (nibble[2] << 8)| (nibble[1] << 4) | nibble[0];

  //DEBUG_STREAM.println(nibble[3]);
  //DEBUG_STREAM.println(nibble[2]);
  //DEBUG_STREAM.println(nibble[1]);
  //DEBUG_STREAM.println(nibble[0]);
  
  return result;
 }
