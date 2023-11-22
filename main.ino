 



/* 
 *  Adafruit Metro SAMD51 processor with 2ADCs
 *  DMA, Event Trigger and dual ADC interface
 *  FreeRTOS Version with multiple threads and inter task queue communications.
 *  Timer TC generates sampling trigger that goes through event system and connects
 *  to ADC that is triggered by each TV trigger.
 *  ADC RESRDY writes new result to DMA which places in a buffer. When buffer is full, DMA
 *  should trigger an interrupt that sets the dmadone flag.
*/

/***************READ THIS ***********************************
 * To compile a legacy version based on Sixfab:
 * comment out #define GPS_VERSION
 * Set the board definition file to Adafruit METRO M4 (SAMD51) 
 * 
 * To compile the newer Glenside GPS enabled NB-IoT module:
 * #define GPS_VERSION
 * *******************
 * DON'T FORGET TO Set the board definition file to Adafruit METRO M4g (SAMD51) 
 * **********************************************************
 */

#define GPS_VERSION

#ifdef  GPS_VERSION
 #define SW_VERSION "4V03g"
#else
 #define SW_VERSION "4V03"
#endif

/* ************************************************
 * The Thingsboard access token is made of of the NB-IoT IMEI number
 * preceded by "IMEI:" so that the overall string is 20 Chars
 * ************************************************
 */
 


/* ************************************************
 * Custmize alarm settings here.   
 * Will eventually be remotely set from the cloud.
 * ************************************************
 */
#define AUDIO_ALARM_LEQ_LEVEL 99          // LEQ level in dB that will eventually trigger the audio alarm relay after AUDIO_ALARM_TRIGGER_TIME_MS milliseconds
#define AUDIO_ALARM_TRIGGER_TIME_MS 5000  // The time in ms that the LEQ is over level before alarm actually triggers
#define AUDIO_ALARM_LEQ_TIMEOUT_MS  10000 // The minimum time in ms that the audio alarm will stay active once triggered


#define VIB_ALARM_PPV_LEVEL_MM 5         // PPV value in mm/sec that will eventually trigger the vibration alarm relay after VIB_ALARM_TRIGGER_TIME_MS milliseconds
#define VIB_AMPLITUDE_THRESHOLD 0.5    // Threshold below which no vibration data is uploaded to Thingsboard
#define VIB_ALARM_TRIGGER_TIME_MS 5000   // The time in ms that the PPV is over level before alarm actually triggers
#define VIB_ALARM_PPV_TIMEOUT_MS 30000   // // The minimum time in ms that the vibration alarm will stay active once triggered

#define FHT_N 128 // set to 128 point fht  (FFT with real numbers only)

// #define STATUS_ATTRIBUTE_INTERVAL  300 // status update interval in seconds 
#define DATA_UPLOAD_INTERVAL 300 // Data upload interval between network connectst and data upload. Ideally 15 mins
#define BACKOFF_INTERVAL 180 // backoff for which NB-IoT will be powered down before reattempting to reconnect to the network
#define NETWORK_KEEPCONNECT_INTERVAL 100  // The time that the we stay connected to the network after transmit data to ensure buffer is emptied onto the network
#define NETWORK_STATUS_TIMER_INTERVAL 10 // This the update interval of the Network connected status Blue LED indicator
#define ATTRIBUTE_ITERATION_CNT 3 // This defines the frequency of attribute update including RTC sync. The inetrval in seconds is ATTRIBUTE_ITERATION_CNT * DATA_UPLOAD_INTERVAL
#define WATCHDOG_RESET_INTERVAL  6  // Defines the interval between resets. The WDT has been set to the maximum of 8000ms so a reset every 6000ms is a safe margin

#define CHARGER_STATUS_STRLEN 32  // length of string used to report the charger and battery status

#define MIN_BATT_VOLTAGE 6.45
/*
extern "C"{
  #include "Filter_H1.h"
  #include "Filter_H3.h"
}
  #include "Filter_H2.h"

*/
#include "A_Weight_filt.h"


extern "C"{
  #include "gps.h"
}



/* *************************
 *  COMPILER SWITCHES
 * ************************* */
 #define NBIOT_ENABLED    // compiler switch for NBIOT based Arduino Shield
//#define ARM_MATH_CM4 // KOS for preprocessor to use CM4 maths library for cmsssis. Changed from CM3 12/11/2018

#define MODEM_STREAM Serial1
#ifdef GPS_VERSION
  #define GPS_STREAM Serial2
#endif
#define DEBUG_STREAM Serial
#define SERIAL_NO_LEN 22
#define IMSI_NO_LEN 18 // its actually 15 but allowing for CR/LF and NULL
#define CELL_ID_LEN 18 // Its actually 8 but allowing for CELL ID : preamble

#define LED_PIN 13 // currently used in audio sampling IRQ to toggle at audiosampling rate. (Toggle freq = 22.05KHz)
#define TEST_PIN12 12 

//#define powerPin 4
const int  powerPin=4;


#ifdef GPS_VERSION 
  #define RELAY_PIN 8
  #define BLUE_LED 9
#else
  #define RELAY_PIN 7
  #define BLUE_LED 5
#endif

#define TX_PIN 1 // UART TX pin nneds to be turned off when powering down NB-IoT board

#include <arduino.h>
#include "AnalogCapture.h"
#include <math.h>
#include "Queue_nOS.h"
#include "AAlarm.h"
#include "Egg_Timer.h"
#include "Percentile_Table.h"
//#include <stdlib.h> // For malloc/free
//#include <string.h> // For memset
#include <arm_math.h>
//#include <TimeLib.h> 
#include <arduinoFFT.h>
#include "WatchdogSAMD.h"

#include <SD.h>
#include <SPI.h>

#include <FreeRTOS_SAMD51.h>

#include<Wire.h>
#include "i2c_periphs.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
double vReal[128];
double vImag[128]; // has to be a power of 2

//#define HWORDS 24  // HWORDS must be an integer multiple of 3 as vibration samples 3 channels.
#define ADC_BLOCKSIZE 48  // size of data transfer for each getADC command. All filtering operates on blocks of this size. max val 64 but must be divisible by 3
#define VIB_ADC_BLOCKSIZE 480 // Will dma 480 samples for processing. over 3 channels thats 160 samples per channel acquired and processed.



volatile uint32_t  dmadone[2];

unsigned long baud = 9600;  //start at 9600 allow the USB port to change the Baudrate

int blueLed=BLUE_LED;

#define LEQ_VIB_QUEUESIZE 100 // Size of Leq queue / buffer  
#define SAMPLERATE_AUDIO 44100
#define SAMPLERATE_VIB 200  // This is the actual sampling rate of each individual ADC vibration channel. The actual ADC sampling rate is 3*SAMPLERATE_VIB
#define LEQ_PAYLOAD_SIZE 3  // number of Leq values encapsulated within each data packet. 
#define VIB_PAYLOAD_SIZE 3
#define PWR_PAYLOAD_SIZE 1

#define PI2 PI*2 
#define ADC_VOLT_REF 3300   // reference voltage in mV
#define ADC_RANGE 4096   // 12 bit ADC range of 4096 
#define GEOPHONE_CONV_FACTOR 28  // 28mV per mm
#define ADC_CONV_RATIO  (float)((float)ADC_VOLT_REF/(float)ADC_RANGE)/GEOPHONE_CONV_FACTOR

float leq_meas_interval = 60;// 0.002;  // Time interval in seconds on which each Leq measurement is calculated
float vib_meas_interval = 60;  // Time interval in seconds on which each xyz vibration measurement is calculated

 

// Remote IP Address and port  of destination Server
char remote_port[8] = "5683";
char remote_ip[16] = "40.113.71.23";

char socket[4];

// lut to efficently convert a byte to a 2 character decimal string
static uint16_t const str100p[100] = {
    0x3030, 0x3130, 0x3230, 0x3330, 0x3430, 0x3530, 0x3630, 0x3730, 0x3830, 0x3930,
    0x3031, 0x3131, 0x3231, 0x3331, 0x3431, 0x3531, 0x3631, 0x3731, 0x3831, 0x3931,
    0x3032, 0x3132, 0x3232, 0x3332, 0x3432, 0x3532, 0x3632, 0x3732, 0x3832, 0x3932,
    0x3033, 0x3133, 0x3233, 0x3333, 0x3433, 0x3533, 0x3633, 0x3733, 0x3833, 0x3933,
    0x3034, 0x3134, 0x3234, 0x3334, 0x3434, 0x3534, 0x3634, 0x3734, 0x3834, 0x3934,
    0x3035, 0x3135, 0x3235, 0x3335, 0x3435, 0x3535, 0x3635, 0x3735, 0x3835, 0x3935,
    0x3036, 0x3136, 0x3236, 0x3336, 0x3436, 0x3536, 0x3636, 0x3736, 0x3836, 0x3936,
    0x3037, 0x3137, 0x3237, 0x3337, 0x3437, 0x3537, 0x3637, 0x3737, 0x3837, 0x3937,
    0x3038, 0x3138, 0x3238, 0x3338, 0x3438, 0x3538, 0x3638, 0x3738, 0x3838, 0x3938,
    0x3039, 0x3139, 0x3239, 0x3339, 0x3439, 0x3539, 0x3639, 0x3739, 0x3839, 0x3939, };



 char coap_packet[200] = "";
 uint16_t message_id = 0x1000;
 uint32_t token = 0x10000000;

char serial_no[SERIAL_NO_LEN];   // product serial number
 char token_id[22];
 char access_token[64]="";  
 
typedef enum transmit_types{ noise, vibrate, status_attr, info_attr, vib_alarm, power }transmit_types ;

typedef struct time_leq_struct_t {
     transmit_types tt;
     long ts;
     float leq;
     float leq10;
     float leq90;
     float leqmx;
     float spare;  // makes structure same size as vibration for the RTos queue
} time_leq_struct_t;

typedef struct time_vib_struct_t {
     transmit_types tt;
     long ts;
     float vibx;
     float viby;
     float vibz;
     float vibxyz;
     float pk_freq;
} time_vib_struct_t;



/* 
 *  *THis has not been implemented yet as of 4v0 
* The attribute task is not yet active and instead attributes are posted
* in the Hub task as a legacy of before FreeRTOS was implemented
*/
typedef struct attr_stat_struct_t {
     transmit_types tt;
     long s_t;
     float longitude;
     float latitude;
     float gps_fix;
     float gps_siu;
     float stats; // encode 3 params in one float :csq +100*battery+1000*rst_cnt
} attr_stat_struct_t;

// This structure is compatible with all the telemetry structures so can receive any telemetry into its queue
// Reason is that we are passing different telemetry structures into the one queue which has identical size in
// order that the NB-IoT task has only one queue to accept telemetry from noise vibration, etc.
typedef struct generic_struct_t {
  transmit_types tt;
  long time_val;
  float param1;
  float param2;
  float param3;
  float param4;
  float param5;
} generic_struct_t;

/*
 * For passing messages to the LED blinker task the following is encoded
 * charger val=0, battery discharging; val=1, solar power input with reduced battery discharge, val = 2, charging
 * network val = 0, disconnected; val=1 connecting, val=2 online, val=3 transmitting data.
 */
typedef enum blink_types {charger, network};  

typedef struct led_blink_mode_t{
  blink_types bt;
  byte val;
}led_blink_mode_t;

uint32_t nb_iot_rst_cnt = 0;
char network_status[16];
time_t start_time;

AnalogCapture digitizer;


bool enable_vibration_flag, enable_noise_flag;
bool enable_measurements;
bool calibration_enabled = 0;
bool connected_flag;
bool bEngineeringMode = 0;

char bserial_no[SERIAL_NO_LEN];  // board serial number
char imsi_no[IMSI_NO_LEN+4]; // IMSI number linked to sim card ICCD
char cell_id[CELL_ID_LEN+4];



//String gps_buf;

uint32_t leq_iterationsA;  // leq_meas_interval converted to loop iterations with each iteration processing audio samples for alarm, LEQ10 and LEQ90
uint32_t leq_iterationsB;  // leq_meas_interval converted to loop iterations with each iteration processing audio samples between each Transmit() of data.
uint32_t leq_iterationsC;  // leq_meas_interval converted to loop iterations with each iteration processing audio samples for LEQ 1 second measurements used for calibration
uint32_t vib_iterationsA; 
uint32_t vib_iterationsB;  // vib_meas_interval converted to loop iterations with each iteration processing VIB_ADC_BLOCKSIZE vibration samples

uint32_t noise_block_countA = 0;  // keeps a count of the number of iterations of processing of blocks of ADC noise data between intervals of 1/8 seconds or 1/16 seconds for fine grain Alarm, LEQ10 and LEQ90 measurements.
uint32_t noise_block_countB = 0;  // keeps a count of the number of iterations of processing of blocks of ADC data noise between each  Leq Transmit over NB-IoT
uint32_t noise_block_countC = 0;  // keeps a count of the number of iterations of processing of blocks of ADC noise data between intervals of 1 seconds LEQ measurements. Used mainly for calibration
uint32_t vib_block_countA = 0; // vibration version of above. A is the frequent loop for alarms.
uint32_t vib_block_countB = 0; //  B is the less frequent loop for ppv reported to Thingsboard typically dictated bt vib_meas_interval.

short attribute_update = 0;

 
uint32_t csq; // signal quality/strength in the range 0  to 31 
float Leq; // The actual Leq loudness value in Decibels dBA
uint32_t LeqMx = 0; // captured peak Leq taken from multiple 1/8 second Leq measurements and reset every minute.
float mic_cal = 27.0;  // Decibel Gain correction to add to Raw Leq in Log domain. 
short * ptrSinelut;
short * ptrTestWfm;


float angle=0;
int  i,j,k, done=0;
int frequency = 20;
float ffreq = 10;




AAlarm audioAlarm;
AAlarm vibAlarm;
boolean vibration_alarm = 0;  // this is used for NB-IoT alarm status updates

// Egg_Timer Status_Attrib_Timer;
Egg_Timer Data_Upload_Timer;
Egg_Timer Backoff_Timer;
Egg_Timer Network_Keepconnect_Timer;
Egg_Timer Network_Status_Timer;  // updates the Blue LED indicator every 10 seconds
Egg_Timer Watchdog_Reset_Timer;  // resets the watch dog timer to prevent a system reset.  

typedef enum 
 {
  COLD_START,
  WARM_START,
  FIRST_START // does not issue AT+NRB because it has just been powered up so is already reset
 } nbiot_start_t;

 typedef enum 
 {
  KEEP_CELL_LIST,
  CLEAR_CELL_LIST
 } cell_list_t;


Percentile_Table Leq10;
Percentile_Table Leq90;


uint32_t accumulator;
uint32_t rms_out;

WatchdogSAMD WatchDogTimer;  // this is the actual watchdog timer class whose reset method is reset by the Watchdog_Reset_Timer


uint32_t t, u, v, w;





// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
File dummyFile;
File logFile;
File telemFile;

const int SD_chipSelect = 10;
boolean sd_card_enabled;

gps_t hgps;

/* **************************************
/* FreeRTOS specific Global Variables */

TaskHandle_t Handle_HubTask;
TaskHandle_t Handle_NoiseTask;
TaskHandle_t Handle_VibTask;
TaskHandle_t Handle_AttrTask;
TaskHandle_t Handle_CLITask;
TaskHandle_t Handle_PwrTask;
TaskHandle_t Handle_monitorTask;
TaskHandle_t Handle_LedTask;

charger_stat  charger_status;
char charger_status_str[CHARGER_STATUS_STRLEN];


/*
 *  ***********************
 *  FUNCTION PROTOTYPE DEFINITIONS
 *  ***********************
 */
void setupCLI();

boolean init_nbiot(void);
boolean initialise_nbiot(nbiot_start_t start,cell_list_t clearEARFCN);  // called from a cold restart
boolean connect_nbiot();
boolean disconnect_nbiot();
//void DEBUG_CLI_Check();
void GPS_CLI_check();
void DEBUG_serialEvent();
void MODEM_serialEvent();
void GPS_serialEvent();
void uint8_tostr(uint8_t in, char * out);
void uint32_tostr(uint32_t in, char * out);
void strtohex(unsigned char * in, size_t insz, char * out, size_t outsz);
void float_tostr(float in, int dp, char * out);
void double_tostr(double in, int dp, char * out);
int validate_OK_resp( void ) ;
void make_coap_header( bool telem_nattrib, char * coap_pkt, uint16_t message_id, uint32_t token, char * access_token);
void get_imei(char * answer);
void get_imsi(char *answer);
void get_cell_id(char *answer);
void get_network_status(char * answer);
boolean Wait_until_connected(void);
boolean Check_connected(void);
void get_time(void);
void get_csq(char * answer);

transmit_types transmit_what;
void transmit_pkt(transmit_types transmit_what, bool release_assist);


void debug_outln1(char * msg);
void debug_out1(char * msg);
void float_to_str( char* buf, float val);
void int_to_str( char* buf, int val);

enum connect_states { live, connecting, disconnected, retry1, retry2, retry3, backoff1, backoff2 };
connect_states connect_state;
void get_date_str( char * input_string);
void myDelayUs(int us); // FreeRTOS based delays. 
void myDelayMs(int ms);
void myDelayMsUntil(TickType_t *previousWakeTime, int ms);

static void prvTaskNB_IoT_Hub( void *pvParameters );
static void prvTaskNoise( void *pvParameters );
static void prvTaskVib( void *pvParameters );
static void prvTaskAttrib( void *pvParameters );
static void prvTaskCLI( void *pvParameters );
static void prvTaskPwr( void *pvParameters );
static void prvtaskMonitor( void *pvParameters );
static void prvTaskLed( void *pvParameters );
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName );
void get__nbiot_type( char * answer);


char date_time_str[32];

 QueueHandle_t x_iot_queue = NULL;
 QueueHandle_t x_led_queue = NULL;
// The QueuenOS objects are declared global eventhough they only reside in NB-IoT_Hub task because the transmit_pkt function needs access. 
Queue_nOS<time_vib_struct_t> vibration_q = Queue_nOS<time_vib_struct_t>(LEQ_VIB_QUEUESIZE); // Queue_nOS holds noise leq timestamped samples to be transmitted
Queue_nOS<time_leq_struct_t> noise_q = Queue_nOS<time_leq_struct_t>(LEQ_VIB_QUEUESIZE); // Queue_nOS holds the vibx,y,z,and xyz timestamped samples to be transmitted
Queue_nOS<generic_struct_t> power_q = Queue_nOS<generic_struct_t>(LEQ_VIB_QUEUESIZE);

/* ***************************************************
 *                     Setup() part of the code      *
 * ***************************************************
 */
void setup() {

pinMode(LED_PIN,OUTPUT); //this configures the LED pin, you can remove this it's just example code
pinMode(TEST_PIN12,OUTPUT);

pinMode(powerPin, OUTPUT);
digitalWrite(powerPin, HIGH); 

pinMode(RELAY_PIN, OUTPUT);
digitalWrite(RELAY_PIN, LOW);


//DEBUG_STREAM.begin(115200);
//delay(1000);
//while (!DEBUG_STREAM); // Wait for serial port to open before continuing

#ifdef GPS_VERSION
 GPS_STREAM.begin(baud);
 delay(1000);
 while (!GPS_STREAM); // Wait for serial port to open before continuing
#endif

MODEM_STREAM.begin(baud);
delay(1500);
while (!MODEM_STREAM); // Wait for serial port to open before continuing
delay(1000);
//MODEM_serialEvent();
MODEM_STREAM.println(F("AT"));
DEBUG_STREAM.println(F("******************************"));
DEBUG_STREAM.println(F("*    (c) 2021 Glenside Env.  *"));
DEBUG_STREAM.println(F("*    Author: Majentix Ltd    *"));
DEBUG_STREAM.println(F("* NB-IoT Noise & Vib Monitor *"));
DEBUG_STREAM.println(F("******************************"));

pinMode(blueLed,OUTPUT);
digitalWrite(blueLed, HIGH);

DEBUG_STREAM.println(F("Resetting NB_IoT: AT+NRB..."));
MODEM_STREAM.println(F("AT+NRB"));
delay(3000);
/*
char tststr[16];
float_tostr(-6.345,6,tststr);
DEBUG_STREAM.println(tststr);
float_tostr(53.12345,6,tststr);
DEBUG_STREAM.println(tststr);
*/


x_iot_queue = xQueueCreate( 400, sizeof( generic_struct_t  ) );  // main iot queue over which telemetry is sent to the iot hub task. tls, tvs, tas are all same size.
if (x_iot_queue == NULL){
  DEBUG_STREAM.println(F(" Unable to create Task Queue. System Halt!"));
  while (1){
    
    delay (1000);
    }
}

x_led_queue = xQueueCreate( 8, sizeof( led_blink_mode_t ) ); 
if (x_led_queue == NULL){
  DEBUG_STREAM.println(F(" Unable to create Task LED Queue. System Halt!"));
  while (1){   
    delay (1000);
    }
}

char tst[10];
strncpy(tst, "Test",4);
xTaskCreate(prvTaskNB_IoT_Hub,     "Task_Hub",       480, NULL, tskIDLE_PRIORITY , &Handle_HubTask); // NB-IoT connectivity task.
xTaskCreate(prvTaskNoise,     "Task_Noise",       256, NULL, tskIDLE_PRIORITY + 4, &Handle_NoiseTask); // 
xTaskCreate(prvTaskVib,     "Task_Vib",       256, NULL, tskIDLE_PRIORITY + 2, &Handle_VibTask); // 
//xTaskCreate(prvTaskAttrib,     "Task_Attr",       512, NULL, tskIDLE_PRIORITY + 1, &Handle_AttrTask); // 
//xTaskCreate(prvTaskCLI,     "Task_CLI",       512, NULL, tskIDLE_PRIORITY + 1, &Handle_CLITask); // 
//xTaskCreate(prvtaskMonitor, "Task_Monitor", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask); // RTOS Task Monitor
xTaskCreate(prvTaskPwr,     "Task_Pwr",       320, NULL, tskIDLE_PRIORITY + 2, &Handle_PwrTask); // 
xTaskCreate(prvTaskLed,     "Task_Led",       128, NULL, tskIDLE_PRIORITY + 3, &Handle_LedTask); 

DEBUG_STREAM.println(F("FreeRTOS Tasks Created"));

// Setup SD Card for logging

//delay(8000);// stall while we debug the sd card

  if (!card.init(SPI_HALF_SPEED, SD_chipSelect)) {
      DEBUG_STREAM.println(F("SD Memory Card not detected."));
      sd_card_enabled = false;
      } 
  else {
        DEBUG_STREAM.println(F("SD memory card detected"));
        sd_card_enabled = true;
  
        DEBUG_STREAM.print(F("Card type:         "));
        switch (card.type()) {
           case SD_CARD_TYPE_SD1:
              DEBUG_STREAM.println(F("SD1"));
           break;
           case SD_CARD_TYPE_SD2:
              DEBUG_STREAM.println(F("SD2"));
           break;
           case SD_CARD_TYPE_SDHC:
             DEBUG_STREAM.println(F("SDHC"));
           break;
           default:
             DEBUG_STREAM.println(F("Unknown"));          
          }

          if (!volume.init(card)) {
             DEBUG_STREAM.println(F("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card"));
             sd_card_enabled = false;
             }
          else {
              uint32_t volumesize;
              DEBUG_STREAM.print(F("Volume type is:    FAT"));
              DEBUG_STREAM.println(volume.fatType(), DEC);
              volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
              volumesize *= volume.clusterCount();       // we'll have a lot of clusters
              volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
              DEBUG_STREAM.print(F("Volume size (Mb):  "));
              volumesize /= 1024;
              DEBUG_STREAM.println(volumesize);
              DEBUG_STREAM.println(F("\nFiles found on the card (name, date and size in bytes): "));
              root.openRoot(volume);
          // list all files in the card with date and size
              root.ls(LS_R | LS_DATE | LS_SIZE);  

              if (!SD.begin(SD_chipSelect)) {
                Serial.println(F("Card failed"));
                }
              else {

                dummyFile = SD.open("dummy.txt", FILE_WRITE); // test  as 1st file write crashes
                if (dummyFile)  DEBUG_STREAM.print(F("Dummy File Opened"));
                   get_date_str( date_time_str);
                   DEBUG_STREAM.println(date_time_str);
                   DEBUG_STREAM.print(F("Opening tlmetry.txt for writing.."));
                   telemFile = SD.open("tlmetry.txt", FILE_WRITE);
                   DEBUG_STREAM.println(".");
                   if (telemFile) {               
                      telemFile.print("System start(clock not yet sync'd to network): "); telemFile.println(date_time_str);
                      DEBUG_STREAM.println("#");
                      telemFile.flush();
                   }
                   else            
                      DEBUG_STREAM.println(F("ERROR: Unable to write to telemFile"));
                   logFile = SD.open("log.txt", FILE_WRITE);
                   if (logFile) {
                      logFile.print("System start(clock not yet sync'd to network): "); logFile.println(date_time_str);
                      logFile.flush();
                   }
                   else
                      DEBUG_STREAM.println(F("ERROR: Unable to write to logFile"));
              }
           }
 
    }

DEBUG_STREAM.print(F("sd_card_enabled = ")); DEBUG_STREAM.println(sd_card_enabled);
DEBUG_STREAM.print(F("logFile = ")); DEBUG_STREAM.println(logFile);
delay(3000);






/* AMP Board detection  3V05 and later
 *  Amp board contains "AMP\0",  0x41, 0x4d, 0x50, 0x00 in 1st 4 bytes unless
 *  its a 1st gen amp board (4 pots) OR its a brand new one with uninitialised EEPROM
 *  We will initialise it if its a new one and try to read back to discern whether
 *  its an old V1 4 pot board or a brand new previously unpowered Amp V2.
 *  mic_cal is stored in eeprom as 2 bytes of BCD with decimal point between the uppwe and 
 *  lower byte. Eg. 27.53dB  is encoded as 0x2753 with upper byte in eeprom sub-address 4
 *  and lowe byte in eeprom address5.
 *  EEPROM Address Map
 *  0x00 - 0x03 = "AMP\0"
 *  0x04 - Mic Cal upper Byte integer in BCD format Eg. 0x27 for 27.??dBs
 *  0x05 - Mic Cal lower Byte fractional in BCD format. Eg. 0x53 for ??.53dBs
 *  0x06 - Mic Cal flag.  0x00 = uncalibrated. 0x01 = calibrated
 *  0x08 - 0x09 - battery monitor RCOMP0
 *  0x0a - 0x0b - battery monitor TempCo
 *  0x0c - 0x0d - battery monitor FullCapRep
 *  0x0e - 0x0f - battery monitor Cycles
 *  0x10 - 0x11 - battery monitor FullCapNom
 */
 
 const float amp_v2_def_mic_cal =  27.1; //dB
   Wire.begin(); // enable i2c 3v05
   int test_byte = 0xff;
   char eeprom_array[10];
   char * ptr_eeprom_array;
   ptr_eeprom_array = eeprom_array;
   Wire.beginTransmission(EEPROM_DEV_ADDR);
   Wire.write(byte(0x00)); // set sub-addr to 0
   Wire.endTransmission();
   Wire.requestFrom(EEPROM_DEV_ADDR,7);  // Should contain 0x41, 0x4d, 0x50, 0x00, CAL_VALUE_U, CAL_VALUE_L, CAL_FLAG.
   delay(200); // allow time for 7 bytes to be read into buffer
   int i2c_elapsed = 0;
   while (Wire.available() < 7 && i2c_elapsed <100){
     delay(10);
     i2c_elapsed++;
     }

     
   for (int i=0;i<7;i++)
     *ptr_eeprom_array++ = Wire.read();
   if ((eeprom_array[0] != 0x41) || (eeprom_array[1] != 0x4d) || (eeprom_array[2] != 0x50) || (eeprom_array[3] != 0x00) ){
   // Either AMP V1 Or EEPROM was never programmed
   // 1st try to program the eeprom with default values if it is present on an AMP V2 board. This saves the factory 
   // from having to preprogram the eeporm.
     Wire.beginTransmission(EEPROM_DEV_ADDR);
     Wire.write(byte(0x00)); // sub address
     Wire.write(byte(0x41)); // 'A'
     Wire.write(byte(0x4D)); // 'M'
     Wire.write(byte(0x50)); // 'P'
     Wire.write(byte(0x00)); // NULL string terminator
     Wire.write((byte)(conv2bcd(amp_v2_def_mic_cal) >> 8)); // default Mic Cal value in dB upper byte
     Wire.write((byte)(conv2bcd(amp_v2_def_mic_cal) & 0x00ff)); // default Mic Cal value in dB lower byte
     Wire.write(byte(0x00)); // 0x00 = uncalibrated, 0x01 = Calibrated
     Wire.endTransmission();
     delay(170);
     // Now read it back to see if the EEPROM is present
     Wire.beginTransmission(EEPROM_DEV_ADDR);
      Wire.write(byte(0x00)); // set sub-addr to 0
      Wire.endTransmission();
      Wire.requestFrom(EEPROM_DEV_ADDR,7);  // Should contain 0x41, 0x4d, 0x50, 0x00, CAL_VALUE, CAL_FLAG.
      delay(200); // allow time for 6 bytes to be read into buffer
      int i2c_elapsed = 0;
      while (Wire.available() < 7 && i2c_elapsed <100){
        delay(10);
        i2c_elapsed++;
      }
      ptr_eeprom_array = eeprom_array; 
      for (int i=0;i<7;i++)
       *ptr_eeprom_array++ = Wire.read();
       if (eeprom_array[0] != 0x41 ){
         debug_outln1("AMP V1 deduced by absence of EEPROM Mic Cal 27dB");
         mic_cal = 27.0;
       }
       else
       {
         debug_outln1("AMP V2 EEPROM did not contain AMP header so initialized to Factory Mic cal 27.5dB");
         mic_cal = amp_v2_def_mic_cal;
       }
       
    }
    else{ // Detected Amp V2 with eeprom containing "AMP" header string
       debug_out1("AMP V2 EEPROM detected with cal value of ");
       char temp[5];
       int_to_str( &temp[0], (int)(eeprom_array[4] >> 4)); // 1st BCD digit
       int_to_str( &temp[1], (int)(eeprom_array[4] & 0x0f)); // 2nd BCD digit
       temp[2] = '.';
       int_to_str( &temp[3], (int)(eeprom_array[5] >> 4)); // 3rd BCD digit
       int_to_str( &temp[4], (int)(eeprom_array[5] & 0x0f)); // 4th BCD digit
       debug_out1(temp); 
       debug_outln1("dB");
       if ((eeprom_array[4] < 100) && (eeprom_array[4] > 9)){
         mic_cal = 10*(eeprom_array[4] >> 4) + (eeprom_array[4] & 0x0f) + 0.1* (float)(eeprom_array[5] >> 4) + 0.01*(eeprom_array[5] & 0x0f );
         DEBUG_STREAM.print(F("mic_cal set to ")); DEBUG_STREAM.print(mic_cal);DEBUG_STREAM.println("dBs");
       }
       else{
        debug_out1("AMP V2 EEPROM Unexpected Cal Value outside range 10 to 100");
         mic_cal =amp_v2_def_mic_cal;
         
       }
    }
Wire.end();


int tste;



enable_vibration_flag = 1;  // set this flag along with enable_measurements to start vibration measurements
enable_noise_flag = 1;      // set this flag along with enable_measurements to start noise measurements
enable_measurements = 1;  // Set this and enable ne of the above flags for measurement.
connected_flag = true; // set this to connect to the NB-IoT network
setupCLI();
DEBUG_STREAM.println(F("CLI initialised"));

// Initialise Arduino NB-IoT Shield here!



digitizer.Set_samplerate(1, SAMPLERATE_AUDIO);
digitizer.Set_buffersize(1, 1024);
digitizer.Set_samplerate(0, SAMPLERATE_VIB * 3);
digitizer.Set_buffersize(0, 1024);
digitizer.Initialise();
DEBUG_STREAM.println(F("digitizer initialised"));
digitizer.AllocateDMAChannel(0);
digitizer.AllocateDMAChannel(1);


dmadone[0] = 1;  // First time setting to enable 1st transfer
dmadone[1] = 1;
DEBUG_STREAM.println(F("DMA enabled"));


leq_iterationsA = ((SAMPLERATE_AUDIO / 8)/ADC_BLOCKSIZE)+1; // The number of iterations on 1/8 second for evaluation of alarm, leq10, leq90.
leq_iterationsB = ((SAMPLERATE_AUDIO * leq_meas_interval)/ADC_BLOCKSIZE)+1;
leq_iterationsC = ((SAMPLERATE_AUDIO * 2)/ADC_BLOCKSIZE)+1; // number of iterations in 2 seconds for calibration
vib_iterationsA = ((SAMPLERATE_VIB * 3 ) /VIB_ADC_BLOCKSIZE)+1;  // The number of iterations every 4 seconds for evaluation of alarms
vib_iterationsB = ((SAMPLERATE_VIB * 3 * vib_meas_interval)/VIB_ADC_BLOCKSIZE)+1;



//*ptrWrite_counter = (uint32_t)0;
  

/*
 * Access token is derived form IMEI of IoT module so same sketch works on all systems. 
 * On THingsboard, device credintials Access Token, Insert "IMEI" before actual IMEI number
 */
 
DEBUG_STREAM.println(F("YOU HAVE 3 SECONDS TO ENTER A COMMAND BEFORE NB-IoT CONNECTS"));
delay(3000); // give user the chance to enter a CLI command before the NB-IoT connecting starts. (Mainly for calibration)
DEBUG_serialEvent();
//delay(100);
//DEBUG_CLI_Check();


//  DEBUG_STREAM.write(String(" datetime = ") + sodaq_gps.getDateTimeString());
  ptrTestWfm = (short*) malloc (ADC_BLOCKSIZE * sizeof(short)); // 64 audio sample buffer


audioAlarm.mag_threshold = AUDIO_ALARM_LEQ_LEVEL; //90;
audioAlarm.time_threshold = AUDIO_ALARM_TRIGGER_TIME_MS; //5000;  // alarm activates when condition is here this long
audioAlarm.alarm_timeout = AUDIO_ALARM_LEQ_TIMEOUT_MS; //30000;
audioAlarm.hysteresis = 3;
audioAlarm.relay_pin = 5;

vibAlarm.mag_threshold =  VIB_ALARM_PPV_LEVEL_MM; //5;
vibAlarm.time_threshold = VIB_ALARM_TRIGGER_TIME_MS; //5000;
vibAlarm.alarm_timeout =  VIB_ALARM_PPV_TIMEOUT_MS; //30000;
vibAlarm.hysteresis = 2;
vibAlarm.relay_pin = RELAY_PIN; 

// Status_Attrib_Timer.duration = STATUS_ATTRIBUTE_INTERVAL;
Data_Upload_Timer.duration = 100; // will get changed to DATA_UPLOAD_INTERVAL after the 1st upload;
Backoff_Timer.duration = BACKOFF_INTERVAL;
Network_Keepconnect_Timer.duration = NETWORK_KEEPCONNECT_INTERVAL;

Network_Status_Timer.duration = NETWORK_STATUS_TIMER_INTERVAL;
Network_Status_Timer.reset();

/*
DEBUG_STREAM.print(F("Enabling watchdog timer.......SET TO "));
//DEBUG_STREAM.print(WatchDogTimer.enable(15000, 0));
DEBUG_STREAM.println("ms");
WatchDogTimer.reset();

Watchdog_Reset_Timer.duration = WATCHDOG_RESET_INTERVAL;
Watchdog_Reset_Timer.reset();
*/

Leq10.percentile = 10;
Leq90.percentile = 90;

get_date_str( date_time_str);
DEBUG_STREAM.println(date_time_str);

telemFile.print("System start(sync'd to network): "); telemFile.println(date_time_str);

DEBUG_STREAM.println(F("Starting Scheduler"));
 // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  while(1)
  {
    DEBUG_STREAM.println(F("Scheduler Failed! \n"));
    delay(1000);
  }

  }





/* *********************************************************
 *                    The Main Loop                        *
 * *********************************************************
 */
void loop() {

       

//audioAlarm.event();    // Audio alarm relay disabled


 /*
 if ((hour() == 0) && (minute() >= 0)   && (minute() < 2)) {  // reset the LEQ10 and LEQ90 values at midnight every night.
  Leq10.reset();
  Leq90.reset();
 }
*/   

/*
// THis is a pass through routine between the 2 serial ports
//DEBUG_CLI_Check();
#ifdef GPS_VERSION
 GPS_CLI_Check();
#endif

DEBUG_serialEvent();
MODEM_serialEvent();

#ifdef GPS_VERSION
 GPS_serialEvent();
#endif
*/
}




// Ensure input string has been declared with at least 19 chars space
void get_date_str( char * input_string)
{ char temp[4];
  sprintf(input_string, "%d", day());
  strcat(input_string, "/");
  sprintf(temp, "%d", month());
  strcat(input_string, temp);
  strcat(input_string, "/");
  sprintf(temp, "%d", year());
  strcat(input_string, temp);
  strcat(input_string, " ");
  sprintf(temp, "%02d", hour());
  strcat(input_string, temp);
  strcat(input_string, ":");
  sprintf(temp, "%02d", minute());
  strcat(input_string, temp);
  strcat(input_string, ":");
  sprintf(temp, "%02d", second());
  strcat(input_string, temp); 
}

void transmit_pkt(transmit_types transmit_what, bool release_assist)
{
  //Assemble  packet here and transmit via NB-IoT
  // first dump the audio_pkt structure to a hexadecimal string
 // int leq_pkt_length = (LEQ_PAYLOAD_SIZE + 512) ; // bigger than it needs to be as 128 incorporates status
  #define VIB_PKT_LENGTH  (VIB_PAYLOAD_SIZE*4 + 512)  // bigger than it needs to be as 128 incorporates status
 
static char ascii_str[20* VIB_PKT_LENGTH];
static char hex_str[40*VIB_PKT_LENGTH];
static char string_length_str[24];
static char num_str[8];
 uint32_t string_length = 0;
 //long timestamp;
time_leq_struct_t time_leq_s ;
time_vib_struct_t time_vib_s;
generic_struct_t time_pwr_s;

 ascii_str[0] = '\0'; // initialise string to null
static char json_str[256];//TODO Needs to relate to PAYLOAD_SIZE
static char telemetry[64];//TODO Needs to relate to PAYLOAD_SIZE
static char temp[128]; //TODO Needs to relate to PAYLOAD_SIZE
static char csq_val[4];
 int csq ;
 int uptime ;
int result;
char rai_flag[strlen("0x200")+1];
char longitude_str[16], latitude_str[16];
static char vx_str[8], vy_str[8], vz_str[8], vxyz_str[8], freq_str[8];
static char leq_str[8], leq10_str[8], leq90_str[8], leqmx_str[8];
static char pwr_str[8];
static char chrg_str[8];
static char batt_v_str[8];
static char batt_i_str[8];
static led_blink_mode_t led_state;
if (release_assist == true)
{
  strcpy(rai_flag, "0x200"); // release assist flag to quicken PSM mode 
}
else
{
  strcpy(rai_flag, "0x000"); // not the last data to be imminently transmitted
}

#ifdef NBIOT_ENABLED
  
 // The JSON part of the packet
//   strcat(ascii_str, "{");


   switch (transmit_what)
      {
       case  vibrate : // vib 
           DEBUG_STREAM.println(F("TX Vib"));              
            strcpy(json_str, "["); //json_str.concat("[");
         for (i=0;i<VIB_PAYLOAD_SIZE;i++){
            time_vib_s = vibration_q.pop();
            float_tostr(time_vib_s.vibx, 1, vx_str);
            float_tostr(time_vib_s.viby, 1, vy_str);
            float_tostr(time_vib_s.vibz, 1, vz_str);
            float_tostr(time_vib_s.vibxyz, 1, vxyz_str);
            sprintf(temp, "{\"ts\":%d000, \"values\":{\"x\":%s,\"y\":%s,\"z\":%s,\"w\":%s", 
                time_vib_s.ts, vx_str, vy_str, vz_str, vxyz_str);
            strcat(json_str, temp);
            
            if (time_vib_s.pk_freq != 0) {  // only add frequency to payload if FFT has been performed
              float_tostr(time_vib_s.pk_freq, 1, freq_str);
              sprintf(temp,",\"f\":%s", freq_str);
              strcat(json_str, temp); 
            }
            strcat(json_str,"}}");
            if (i < VIB_PAYLOAD_SIZE-1)
                  strcat(json_str,",");
         }
           strcat(json_str,"]");

// dump to sd card
sprintf(telemetry, "%d,vib,%s,%s,%s,%s",time_vib_s.ts,vx_str, vy_str, vz_str, vxyz_str );
if (time_vib_s.pk_freq != 0){
strcat(telemetry,",");
strcat(telemetry,freq_str);
}

telemFile.println( telemetry);
telemFile.flush();
         
       break;
       case  noise : // audio 
         
         strcpy(json_str, "["); //json_str.concat("["); 
         for (i=0;i<LEQ_PAYLOAD_SIZE;i++){
            time_leq_s = noise_q.pop(); 
            float_tostr(time_leq_s.leq, 1, leq_str);
            float_tostr(time_leq_s.leq10, 1, leq10_str);
            float_tostr(time_leq_s.leq90, 1, leq90_str);
            float_tostr(time_leq_s.leqmx, 1, leqmx_str);
            
            
            sprintf(temp, "{\"ts\":%d000, \"values\":{\"l\":%s,\"l10\":%s,\"l90\":%s,\"lmx\":%s", 
                    time_leq_s.ts, leq_str, leq10_str, leq90_str, leqmx_str);
            strcat(json_str, temp);  
            
            strcat(json_str,"}}");
            if (i < LEQ_PAYLOAD_SIZE-1)
                strcat(json_str,",");           
         }
         
         strcat(json_str,"]");
DEBUG_STREAM.println(F("write to telemetry file"));
      
      sprintf(telemetry, "%d,leq,%s,%s,%s,%s",time_leq_s.ts, leq_str, leq10_str, leq90_str, leqmx_str);
 DEBUG_STREAM.println(telemetry);
 
       
       telemFile.println(telemetry);
       DEBUG_STREAM.println(".");
       telemFile.flush();
       DEBUG_STREAM.println(".");
DEBUG_STREAM.println("done!");
       
       break;

case power:
     strcpy(json_str, "[");
     for (i=0;i<PWR_PAYLOAD_SIZE;i++){
             time_pwr_s = power_q.pop(); 
             float_tostr(time_pwr_s.param1, 1, pwr_str);
             float_tostr(time_pwr_s.param2, 1, chrg_str);
             float_tostr(time_pwr_s.param3, 1, batt_v_str);
             float_tostr(time_pwr_s.param4, 1, batt_i_str);
             sprintf(temp, "{\"ts\":%d000, \"values\":{\"bp\":%s,\"cp\":%s,\"bv\":%s,\"bi\":%s",time_pwr_s.time_val, pwr_str, chrg_str, batt_v_str,batt_i_str );
             strcat(json_str, temp);  
             strcat(json_str,"}}");
             if (i < PWR_PAYLOAD_SIZE-1)
                strcat(json_str,",");
     }
            strcat(json_str,"]");
break;

      case status_attr : // status battery, signal strength csk, nb_iot_rst_cnt, up_time....
       get_csq(csq_val);
        csq = atoi(csq_val);
        uptime = now()-start_time;
        //charger_status = get_charger_stat();  moved to power task
       sprintf(json_str, "{\"q\":%d,\"r_cnt\":%d,\"u_t\":%d,\"cs\":%d", csq, nb_iot_rst_cnt, uptime, charger_status);
       
       // TODO: Need to qualify GPS coordinates before uploading. Maybe check the validity bit.
       double_tostr(hgps.longitude, 6, longitude_str);
       double_tostr(hgps.latitude, 6, latitude_str);
       sprintf(temp, ",\"latitude\":%s,\"longitude\":%s",latitude_str, longitude_str);
      // debug_outln1(temp); 
      // DEBUG_STREAM.print("double lat, long ");DEBUG_STREAM.print(hgps.latitude);DEBUG_STREAM.print(", ");DEBUG_STREAM.println(hgps.longitude);
       if ((hgps.fix != 0) || (1==hgps.is_valid))
       {  
        strcat(json_str,temp);
       /*
       json_str.concat(",\"latitude\":");
       json_str.concat(String(hgps.latitude,6));
       json_str.concat(",\"longitude\":");
       json_str.concat(String(hgps.longitude,6));
       */
       }
       sprintf(temp, ",\"gps_v\":%d,\"siu\":%d", hgps.fix+hgps.is_valid,hgps.sats_in_use); 
       /*
       json_str.concat(",\"gps_v\":");
       json_str.concat(String(hgps.fix+hgps.is_valid));
       json_str.concat(",\"siu\":");
       json_str.concat(String(hgps.sats_in_use));
       */
       strcat(json_str,temp);
       //json_str.concat("}");
        strcat(json_str,"}");
       
       break;

      case info_attr : // SW Version, start_time, etc
      sprintf(json_str, "{\"sv\":\"%s\", \"s_t\":%d, \"imsi\":%s,\"cell\":%s}",SW_VERSION,(int)start_time,imsi_no,cell_id );
      DEBUG_STREAM.println("Transmit Info Attr");
      DEBUG_STREAM.println(json_str);
  
       break;
       
       case vib_alarm :
       sprintf(json_str,"{\"v_a\":\"%d}",(int)vibration_alarm);
       /*
       json_str.concat("{\"v_a\":\"");
       json_str.concat(String(vibration_alarm));
       json_str.concat("}");
       */
       sprintf(temp, "%d,v_a,%d", (int)now(), (int)vibration_alarm);
       telemFile.println(temp);
       //telemFile.println( String(now()) + String(",v_a,") + String(vibration_alarm) );
       telemFile.flush();
       break;
    }
      
  string_length = strlen(json_str); // json_str.length();  // all the punctuation including colons commas, quotes etc

// json_str now has the entire json payload packet including leq data. just need to convert to ascii HEX and contatenate it with the Coap packet

//json_str.toCharArray(ascii_str,string_length+1); 
strcpy(ascii_str, json_str);
debug_out1("JSON PAYLOAD  (ascii_str) =");
debug_outln1(ascii_str);

/*
 * This ascii_str here can be used to send over a Lora link as the COAP stuff has not been added yet.
 */

   
  // The COAP header here with calculated length of message from string_length
  if ((transmit_what == noise) or (transmit_what == vibrate) or (transmit_what == power) )
     make_coap_header(1, &coap_packet[0], message_id++, token++, &access_token[0]);  // telemetry 
  else
     make_coap_header(0, &coap_packet[0], message_id++, token++, &access_token[0]);  // attribute
//  debug_out("COAP Header is   ");
//  debug_outln(coap_packet);
  
  string_length += strlen(coap_packet)/2;  // coap packet is already in hex format so string length should be half
  uint32_tostr(string_length, string_length_str);
  strtohex((unsigned char*)ascii_str, string_length, hex_str, string_length*2);
  //reuse ascii_str for assembly of final output string

  strcpy(ascii_str, "AT+NSOSTF=");  //strcpy(ascii_str, "AT+NSOST=0,\"");  // quotes sripped out in contradiction to docs
  strcat(ascii_str, socket);
  strcat(ascii_str, ",");
  strcat(ascii_str, remote_ip);
  strcat(ascii_str, ",");
  strcat(ascii_str, remote_port);
  strcat(ascii_str, ",");
  strcat(ascii_str, rai_flag); // Release indicator: indicate release after next message
  strcat(ascii_str, ",");
  strcat(ascii_str, (string_length_str)); //Maximum data size is 512 bytes.
  strcat(ascii_str,",");
  strcat(ascii_str, coap_packet);
  strcat(ascii_str, hex_str);


  

 //***********************************************************************************************************************
//  Temporary disable modem command here for debug purposes 21/11/2018 kos

  if (connect_state == live) {

  DEBUG_STREAM.print(F("Full MODEM COMMAND is "));
  DEBUG_STREAM.println(ascii_str);

led_state.bt = network;
led_state.val = 3; // network val = 0, disconnected; val=1 connecting, val=2 online, val=3 transmitting data.
xQueueSend( x_led_queue, &led_state, 0 );

    MODEM_STREAM.println(ascii_str);
    
    result = validate_OK_resp();
    switch (result)
     {
      case 0:
        debug_outlnF(F(" Packet Send was not acknowledged"));
        break;
      case -1:
        debug_outln1(" ERROR Returned.  Disconnecting from Network");
  //    MODEM_STREAM.println("AT+COPS=2");  // not sure that this did anything for us.
        delay(100);
        MODEM_STREAM.println(F("AT+CFUN=0"));  // THis will disconnect the module form the network and force a reset elsewhere 
       break;
     case 1:
        debug_outln1("Packet Sent");
        break;
   
      }
led_state.bt = network;
led_state.val = 2; // network val = 0, disconnected; val=1 connecting, val=2 online, val=3 transmitting data.
xQueueSend( x_led_queue, &led_state, 0 );

  
// *************************************************************************************************************************

  }
#endif


}

void uint8_tostr(uint8_t in, char * out){
/*
// Converts a byte number in the range 0 to 199 into a 2 or 3 character string
// Its used for the leq conversion in decibels which is typiclly in the range 0 - 120dB
*/
 
  uint8_t temp;
    char * pout = out;
    temp = in;
 
  if (temp < 100) {
      memcpy(pout, &str100p[temp], sizeof(uint16_t));
      pout[2] = 0;
      return;
  }
  if (temp < 200){ 
        *pout++ = 0x31;
        temp= temp % 100;
        memcpy(pout, &str100p[temp], sizeof(uint16_t)); 
        return;  
      }
   if (temp < 255){
        *pout++ = 0x32;
        temp= temp % 200;
       memcpy(pout, &str100p[temp], sizeof(uint16_t)); 
        return;
   }
}

void uint32_tostr(uint32_t in, char * out)

{   uint32_t temp;
    char * pout = out;
    temp = in;
   sprintf( pout, "%u", in);
      
}


void float_tostr(float in, int dp, char * out)
{
  
  float tmp_fltval;
  float tmp_fractval;
  int tmp_intval, tmp_intval2;
  tmp_intval = (int)fabsf(in);
  tmp_fractval = fabsf(in) - (float)tmp_intval;
  if (in < 0)
    *out++ = '-';  // append negative sign to string
  switch (dp) 
   { 

   case 0:     
     sprintf (out, "%d", tmp_intval);
     break;
   case 1 :
     tmp_intval2 = (int)truncf(tmp_fractval * 10);
     sprintf (out, "%d.%01d", tmp_intval,tmp_intval2);
     break;
   case 2 :
     tmp_intval2 = (int)truncf(tmp_fractval * 100);
     sprintf (out, "%d.%02d", tmp_intval,tmp_intval2);
     break;
   case 3 :
     tmp_intval2 = (int)truncf(tmp_fractval * 1000);
     sprintf (out, "%d.%03d", tmp_intval,tmp_intval2);
     break;
   case 4 :
     tmp_intval2 = (int)truncf(tmp_fractval * 10000);
     sprintf (out, "%d.%04d", tmp_intval,tmp_intval2);
     break;
   case 5 :
     tmp_intval2 = (int)truncf(tmp_fractval * 100000);
     sprintf (out, "%d.%05d", tmp_intval,tmp_intval2);
     break;
   case 6 :
     tmp_intval2 = (int)truncf(tmp_fractval * 1000000);
     sprintf (out, "%d.%06d", tmp_intval,tmp_intval2);
     break;
   } 
  return;
}

void double_tostr(double in, int dp, char * out)
{
  
  double tmp_fltval;
  double tmp_fractval;
  int tmp_intval, tmp_intval2;
  tmp_intval = (int)fabs(in);
  tmp_fractval = fabs(in) - (double)tmp_intval;
  if (in < 0)
    *out++ = '-';  // append negative sign to string
  switch (dp) 
   { 

   case 0:     
     sprintf (out, "%d", tmp_intval);
     break;
   case 1 :
     tmp_intval2 = (int)trunc(tmp_fractval * 10);
     sprintf (out, "%d.%01d", tmp_intval,tmp_intval2);
     break;
   case 2 :
     tmp_intval2 = (int)trunc(tmp_fractval * 100);
     sprintf (out, "%d.%02d", tmp_intval,tmp_intval2);
     break;
   case 3 :
     tmp_intval2 = (int)trunc(tmp_fractval * 1000);
     sprintf (out, "%d.%03d", tmp_intval,tmp_intval2);
     break;
   case 4 :
     tmp_intval2 = (int)trunc(tmp_fractval * 10000);
     sprintf (out, "%d.%04d", tmp_intval,tmp_intval2);
     break;
   case 5 :
     tmp_intval2 = (int)trunc(tmp_fractval * 100000);
     sprintf (out, "%d.%05d", tmp_intval,tmp_intval2);
     break;
   case 6 :
     tmp_intval2 = (int)trunc(tmp_fractval * 1000000);
     sprintf (out, "%d.%06d", tmp_intval,tmp_intval2);
     break;
   } 
  return;
}

void uint16tohex(uint16_t in, char * out)
{
    int i;
    const char * hex = "0123456789ABCDEF";
    char * pout = out;
    for (i=0;i<4;i++){
        pout[i] = hex[(in>>(12-4*i)) & 0xF];
    }   
    pout[4] = 0; // end of string        
}

void uint32tohex(uint32_t in, char * out)
{
    int i;
    const char * hex = "0123456789ABCDEF";
    char * pout = out;
    for (i=0;i<8;i++){
        pout[i] = hex[(in>>(28-4*i)) & 0xF];
    }    
        pout[8] = 0;
}

void strtohex(unsigned char * in, size_t insz, char * out, size_t outsz)
{
    unsigned char * pin = in;
    const char * hex = "0123456789ABCDEF";
    char * pout = out;
    for(; pin < in+insz; pout +=2, pin++){
        if (*pin==0){
          pout[0] = 0;
           break;
        }
        pout[0] = hex[(*pin>>4) & 0xF];
        pout[1] = hex[ *pin     & 0xF];

        if (pout + 2 - out > outsz){
            /* Better to truncate output string than overflow buffer */
            /* it would be still better to either return a status */
            /* or ensure the target buffer is large enough and it never happen */
            break;
        }
    }
//    pout[-1] = 0;
}




boolean initialise_nbiot(nbiot_start_t start, cell_list_t clearEARFCN)  // called from a cold restart
{
  debug_outln1("Initialising NB-IOT Module...");
  if (start == COLD_START)
  {
    digitalWrite(powerPin, LOW);
    MODEM_STREAM.end();
    #ifdef GPS_VERSION
     GPS_STREAM.end();
    #endif
    //pinMode(TX_PIN, INPUT); // disable pin to stop 3.3V goinginto BC95 module.
    WatchDogTimer.reset();
    myDelayMs(2500);
    WatchDogTimer.reset();
  }
    digitalWrite(powerPin, HIGH);
    MODEM_STREAM.begin(baud);
    #ifdef GPS_VERSION
      GPS_STREAM.begin(baud);
    #endif
    myDelayMs(3000); // wait for nb-iot module to powerup and then reset
    WatchDogTimer.reset();
    if (start != FIRST_START){
      MODEM_STREAM.println(F("AT+NRB"));
      myDelayMs(5000);
      validate_OK_resp();
      WatchDogTimer.reset();
     }
       
  MODEM_STREAM.println(F("AT+NCONFIG=\"CR_0859_SI_AVOID\",\"FALSE\""));
  DEBUG_STREAM.println(F("AT+NCONFIG=\"CR_0859_SI_AVOID\",\"FALSE\""));
  if ( validate_OK_resp()!=1 )
    return false;
  MODEM_STREAM.println(F("AT+NCONFIG=\"CR_0354_0338_SCRAMBLING\",\"TRUE\"")); //Enable Scrambling, necessary for Vodafone
  DEBUG_STREAM.println(F("AT+NCONFIG=\"CR_0354_0338_SCRAMBLING\",\"TRUE\""));
    if ( validate_OK_resp()!=1 )
    return false;

MODEM_STREAM.println(F("AT+CFUN=0"));
    debug_outln1("AT+CFUN=0");
  if ( validate_OK_resp()!=1 )
    return false;

  if (clearEARFCN ==CLEAR_CELL_LIST) {
     MODEM_STREAM.println(F("AT+NCSEARFCN"));
     DEBUG_STREAM.println(F("AT+NCSEARFCN"));
     if ( validate_OK_resp()!=1 )
    return false;
  }

   MODEM_STREAM.println(F("AT+NCONFIG=\"CELL_RESELECTION\",\"TRUE\""));
   DEBUG_STREAM.println(F("AT+NCONFIG=\"CELL_RESELECTION\",\"TRUE\""));   
    if ( validate_OK_resp()!=1 )
    return false;
     
  MODEM_STREAM.println(F("AT+NBAND=20"));
  debug_outln1("AT+NBAND=20");
  if ( validate_OK_resp()!=1 )
    return false;
 // THis command disables the OK response for AT+CFUN and AT+COPS  so may have to comment it out to get it working

//delay(5000);
 
  // Now turn back on radio with settings  
  MODEM_STREAM.println(F("AT+CFUN=1"));
  debug_outln1("AT+CFUN=1");
  if ( validate_OK_resp()!=1 )
    return false;


    MODEM_STREAM.println(F("AT+CGDCONT=1, \"IP\",\"nb.inetd.gdsp\""));
      debug_outln1("AT+CGDCONT=1, \"IP\",\"nb.inetd.gdsp\"");
  if ( validate_OK_resp()!=1 )
    return false;
    

  MODEM_STREAM.println(F("AT+COPS=1,2,\"27201\""));
  debug_outln1("AT+COPS=1,2,\"27201\"");
  if ( validate_OK_resp()!=1 )
    return false;  
   
   debug_outln1(" NB-IOT Module Successfully Initialised");
  
   return true;
}

/*
int validate_OK_resp( void )  
{ 
 static char temp[200];
  int bytecount=0;
  time_t timestamp, elapsed;
  myDelayMs(5);
  timestamp = now();
  myDelayMs(5);
    elapsed = (now() - timestamp);
   char strval[16];  
   int i = 0;  
    while (elapsed < 15) {
      //DEBUG_STREAM.print("elapsed= ");DEBUG_STREAM.println(elapsed);
      while (MODEM_STREAM.available() && i < 200) 
     { 
       myDelayMs(5);
       WatchDogTimer.reset();
          temp[i++] = MODEM_STREAM.read();
          temp[i] = 0; // always terminate the growing string 
          DEBUG_STREAM.print(&temp[i-1]);
          //DEBUG_STREAM.print("AT Response: ");DEBUG_STREAM.println(temp);
          if (strstr(temp, "ERROR") != 0){
           // DEBUG_STREAM.print(temp);
            return -1;

          }
          if (strstr(temp, "OK\r\n") != 0){
            //DEBUG_STREAM.print(temp);
            return 1;
          }
       }
    elapsed = (now() - timestamp);
  }
debug_out1(" Validate_OK_resp timed out. Elapsed = ");
//debug_outln(elapsed);
sprintf(strval, "%d", elapsed);
debug_outln1(strval);
return 0;  // timed out if it got this far 
}
*/
int validate_OK_resp( void )  
{ 
 static char temp[200];
  int bytecount=0;
  TickType_t timestamp, elapsed;
  
  timestamp = xTaskGetTickCount();
  myDelayMs(5);
    elapsed = (xTaskGetTickCount() - timestamp)/1000;
   char strval[16];  
   int i = 0;  
    while (elapsed < 15) {
      //DEBUG_STREAM.print("elapsed= ");DEBUG_STREAM.println(elapsed);
      while (MODEM_STREAM.available() && i < 200) 
     { 
       myDelayMs(5);
       WatchDogTimer.reset();
          temp[i++] = MODEM_STREAM.read();
          temp[i] = 0; // always terminate the growing string 
          DEBUG_STREAM.print(&temp[i-1]);
          //DEBUG_STREAM.print("AT Response: ");DEBUG_STREAM.println(temp);
          if (strstr(temp, "ERROR") != 0){
           // DEBUG_STREAM.print(temp);
            return -1;

          }
          if (strstr(temp, "OK\r\n") != 0){
            //DEBUG_STREAM.print(temp);
            return 1;
          }
       }
   elapsed = (xTaskGetTickCount() - timestamp)/1000;
   //DEBUG_STREAM.print(F("elapsed="));DEBUG_STREAM.println(elapsed);
  }
debug_out1(" Validate_OK_resp timed out. Elapsed = ");
//debug_outln(elapsed);
sprintf(strval, "%d", elapsed);
debug_outln1(strval);
return 0;  // timed out if it got this far 
}






boolean Wait_until_connected()
{  
     
     int byte_count =0;
 static    char temp[200];
     char temp1[24];
     char * str_ptr;
    int i = 0;
    char val;
     time_t timestamp, elapsed;
    timestamp = now();
     myDelayMs(1);
    elapsed = (now() - timestamp);
    delay (250);
  //THis shoud have a robust get_csq call here to avoid duplication of code.
    while (MODEM_STREAM.available()) {  MODEM_STREAM.read(); myDelayMs(1); } // clear out rx buffer
    MODEM_STREAM.println(F("AT+CSQ"));

    while (elapsed < 20) {
      i=0;
      while (MODEM_STREAM.available() && i < 20){
         temp[i++] = MODEM_STREAM.read();
         temp[i] = 0; // always terminate the growing string 
         DEBUG_STREAM.print(&temp[i-1]);
         myDelayMs(2);
         WatchDogTimer.reset();
      } 
      if (strstr(temp, "+CSQ:") !=0  && strstr(temp, "+CSQ:99,99") ==0){ // we have a CSQ val that is not +CSQ:99,99
         DEBUG_STREAM.println(F("CSQ signal acquired"));
         break;
      }
      myDelayMs(500);
      MODEM_STREAM.println(F("AT+CSQ"));
      elapsed = (now() - timestamp);
    }         
    WatchDogTimer.reset();
    if (strstr(temp,"ERROR") !=0){  // THe modem is not accepting AT+CSQ commands so reset
      debug_outln1("NB-IoT Module returning ERROR with AT+CSQ command. Needs reset"); 
      return false;
    }
    
    if (elapsed > 18){
      debug_out1("NB-IOT failed to connect to the NB-IOT network ");
      debug_outln1(temp);
      return false;
     }
   
      debug_out1("CSQ is ");
      debug_outln1(temp);
      //delay(1000);
      myDelayMs(500);
      //return 1; // test
      i=0;
      //    Diagnostic Info Here. Itcan be comented out if desired when system is working reliably *************************
      myDelayMs(500);
      MODEM_STREAM.println(F("AT+NUESTATS"));
      debug_outln1("AT+NUESTATS");
      myDelayMs(500);
      while(MODEM_STREAM.available()){
        byte_count = MODEM_STREAM.readBytes(temp, 32);
        temp[byte_count]=0;
        debug_out1(temp);
      }
       MODEM_STREAM.println(F("AT+NUESTATS=CELL"));
       debug_outln1("AT+NUESTATS=CELL");
      myDelayMs(100);
      while(MODEM_STREAM.available()){
        byte_count = MODEM_STREAM.readBytes(temp, 32);
        temp[byte_count]=0;
        debug_out1(temp);
      }
      
      //    End of diagnostic section **************************************************************************************
timestamp = now();
myDelayMs(1);
elapsed = (now() - timestamp);
while (MODEM_STREAM.available()) {  MODEM_STREAM.read(); myDelayMs(1); } // clear out rx buffer
 MODEM_STREAM.println(F("AT+CGATT?"));
while (elapsed < 20) {
      i=0;
      while (MODEM_STREAM.available() && i < 20){
         temp[i++] = MODEM_STREAM.read();
         temp[i] = 0; // always terminate the growing string 
         DEBUG_STREAM.print(&temp[i-1]);
         myDelayMs(2);
      } 
      if (strstr(temp, "+CGATT:1") !=0 ){ // registered on network
         DEBUG_STREAM.println(F("Connected :-)"));
         return true;
      }
      myDelayMs(500);
      MODEM_STREAM.println(F("AT+CGATT?"));
      elapsed = (now() - timestamp);
    }         
    WatchDogTimer.reset();
    if (strstr(temp,"ERROR") !=0){  // THe modem is not accepting AT+CGATT? commands so reset
      debug_outln1("NB-IoT Module returning ERROR with AT+CGATT? command. Needs reset"); 
      return false;
    }

        if (elapsed > 18){
      debug_out1("NB-IOT failed to connect to the NB-IOT network ");
      debug_outln1(temp);
      return false;
     }

   
       
 }

boolean Check_connected()
{
  char temp[64];
  int byte_count=0;
  if (digitalRead(powerPin) == 1) {
    while (MODEM_STREAM.available()) {  MODEM_STREAM.read(); myDelayMs(1); }
     MODEM_STREAM.println(F("AT+CGATT?"));
     myDelayMs(250);
    byte_count= MODEM_STREAM.readBytes(temp, 20);
    temp[byte_count]=0;
     debug_outln1(temp);
     
      if (strstr(temp,"+CGATT:1") !=0){
         return true;
         }
      else {
        return false;
      }
  }
   else {
    return false;
   }
}



boolean connect_nbiot()
{
  char temp[64];
  char * str_ptr;
  led_blink_mode_t led_state;
  //digitalWrite(blueLed, LOW); //kk
  //digitalWrite(powerPin, HIGH); // Power up the Quectel module
  //pinMode(TX_PIN, OUTPUT); 
  MODEM_STREAM.begin(baud);
  #ifdef GPS_VERSION
     GPS_STREAM.begin(baud);
  #endif
  //delay(2000);
   WatchDogTimer.reset(); // 9/3/2020

led_state.bt = network;
led_state.val = 1; // network val = 0, disconnected; val=1 connecting, val=2 online, val=3 transmitting data.
xQueueSend( x_led_queue, &led_state, 0 );
  
 delay(1000);
 

 debug_outln1("Connecting to NB-IoT Network");
 if (Wait_until_connected()==false){
      debug_outln1("ERROR 100: Failed to connect to network");
      return false;
 }
// now open a socket

 if (Check_connected() == false){
       debug_outln1("Not attached to network:  ATTACHING!");
       //MODEM_STREAM.println("AT+CGATT=1");
       //debug_outln("AT+CGATT=1");
       return false;
 }
    WatchDogTimer.reset(); // 27/2/2023
    MODEM_STREAM.println(F("AT+NSOCR=DGRAM,17,16667,1")); 
    debug_outlnF(F("AT+NSOCR=DGRAM,17,16667,1")); 
//  while (MODEM_STREAM.available()==0)        
//         WatchDogTimer.reset(); 
            
  MODEM_STREAM.readBytes(temp,16);
  temp[15] = 0;
  
 
str_ptr = strpbrk(temp, "0123456");
        if (str_ptr !=0){
        strncpy(socket, str_ptr,1);
        debug_out1("DGRAM socket opened: ");debug_outln1(socket);
        led_state.bt = network;
        led_state.val = 2; // network val = 0, disconnected; val=1 connecting, val=2 online, val=3 transmitting data.
        xQueueSend( x_led_queue, &led_state, 0 );
        //digitalWrite(blueLed, HIGH); //kk
        return true;
         }  
    else {
         debug_out1("Unexpected DGRAM socket: ");debug_outln1(socket);
         return false;     
         }
 
}


boolean disconnect_nbiot()
{

 //digitalWrite(powerPin, LOW);     
//    MODEM_STREAM.println("AT+COPS=2");
      
    connect_state = disconnected;
    MODEM_STREAM.println(F("AT+CSCON?"));
    if (validate_OK_resp()!=1){
       debug_outlnF(F("*** Failed to release RRC, reset module"));
       MODEM_STREAM.println(F("AT+NRB"));
       }
 /*
    else { 
       MODEM_STREAM.println("AT+CFUN=0");
       if (validate_OK_resp()!=1){
           debug_outln("*** Failed to turn off radio, rebooting module");
           MODEM_STREAM.println("AT+NRB");
           }
*/
       debug_outlnF(F("Disconnected from NB-IoT Network"));
 
 return true;  
}


/*
 * coap_pkt is pointer to a string that will conain the fully formatted COAP header
 * message_id is a value that should increment on each transmission and wrap_around.
 * token is currently a 4 byte value that we can set to uniquely identify responses from the server 
 * uri is the uri to post telemetry data such as noise and vibration. Its currently  api/v1/$ACCESS_TOKEN/telemetry
 * For now, we'll just hardwire the URI and insert the access token into it.
 * where $ACCESS_TOKEN is the Thingsboard access token that identifies the device and steers the message to the correct dashboard
 */


void make_coap_header( bool telem_nattrib, char *coap_pkt, uint16_t message_id, uint32_t token, char * access_token)
   { 
      
    int i, j;
    char ascii_str[25] = "";
    coap_pkt[0] = 0;
   strcat(coap_pkt, "5402");  // Version 1 COAP, non confirmable, token length 4
     uint16tohex(message_id, &ascii_str[0]);
     strcat(coap_pkt, ascii_str);
     uint32tohex(token, &ascii_str[0]);
     /*
      * Ideally we should accept the URI as a function parameter and parse it out to create a COAP packet from it. THis requires the parser to do a few things
      * including finding each '/' character in the uri and breaking it into multiple options values. It also means calculating the option codes
      * and values that work on a delta system.  As time is short in the development, the URI is relatively static  "/api/v1/$ACCESS_TOKEN/telemetry" and "/api/v1/$ACCESS_TOKEN/attributes"
      * We will hard wire this value and pass the access token and insert it into the packet as a quick temporary solution.
      */
     strcat(coap_pkt, ascii_str);
     strcat(coap_pkt, "b3617069");  // hardwired this to be a URI path option be b3+"api"
     strcat(coap_pkt, "027631");  // hardwired this to be continuiation of URI path option  02+"v1"
     strcat(coap_pkt, "0D07"); // 0x0D = Opt Length 13, 0x07 Opt Length extended 7 -> total length 20 for access token
     strcat(coap_pkt, access_token);
     if (telem_nattrib==1)
        strcat(coap_pkt, "0974656C656D65747279"); // 0x09 = Opt Length 9  + "telemetry"
     else
        strcat(coap_pkt, "0A61747472696275746573"); // 0x0A = Opt Length 10  + "attributes"
     strcat(coap_pkt, "FF"); // End of options. Payload will be bolted onto the end of this.
   }

void get_time()
{ 
  // time format returned by AT+CCLK?
  // +CCLK:YY/MM/DD,HH:MM:SS+ZZ
  char buf_str[64];
  char temp[64];
  size_t bytecount=0;
  int i=0;
  char * str_ptr;
  long l_year, l_month, l_day, l_hour, l_min, l_sec;
  if (Check_connected() == 1) {
    myDelayMs(1000); // give th network time to update the clock
     MODEM_STREAM.println(F("AT+CCLK?"));
     while ((strstr(temp,"+CCLK:") ==0) && i <10)  { 
        delay (200);
        bytecount= MODEM_STREAM.readBytes(temp, 64);
        temp[bytecount] = 0; // string terminator
        i++;
      }
DEBUG_STREAM.println(temp);
     str_ptr = strstr(temp,"+CCLK:"); 
     strncpy( buf_str, str_ptr+6,2);
     l_year = strtol(buf_str,NULL,10 );
     strncpy( buf_str, str_ptr+9,2);
     l_month = strtol(buf_str,NULL,10 );
     strncpy( buf_str, str_ptr+12,2);
     l_day =  strtol(buf_str,NULL,10 );
     str_ptr = strstr(temp,","); 
     strncpy( buf_str, str_ptr+1,2);
     l_hour = strtol(buf_str,NULL,10 );
     strncpy( buf_str, str_ptr+4,2);
     l_min = strtol(buf_str,NULL,10 );
     strncpy( buf_str, str_ptr+7,2);
     l_sec = strtol(buf_str,NULL,10 );

     if (l_year > 19 && l_year < 99){  // ensures we dont program the clock with a bad date
         WatchDogTimer.reset(); 
         WatchDogTimer.disable();  // disable here because the clock will change and it could trip the watchdog timer.
         setTime(l_hour, l_min, l_sec, l_day, l_month, (l_year+2000));
         //DEBUG_STREAM.println("WDT:" + String(WatchDogTimer.enable(15000, 0)) + "ms " ); 
         WatchDogTimer.enable(15000, 0); // reenable here 3v04
         sprintf(temp, "Time Set is: %d/%d/%d    %02d:%02d:%02d",(int)day(), (int)month(), (int)year(), (int)hour(), (int)minute(), (int)second() );
         debug_outln1(temp);
         WatchDogTimer.reset(); 
        
         //debug_outln1(now()); // epoch timestamp
         return;
        }
   #ifdef GPS_VERSION
    GPS_serialEvent();
    GPS_CLI_Check();
   
    if ((hgps.year > 19) && (hgps.year < 80)) // use GPS RTC as backup because cell time is corrupt. GPS EPOCH is 1980 hence check for < 80.
      {
        WatchDogTimer.reset(); 
        WatchDogTimer.disable();  // disable here because the clock will change and it could trip the watchdog timer.
        setTime( (long)hgps.hours, (long)hgps.minutes, (long)hgps.seconds, (long)hgps.date,(long)hgps.month, ((long)hgps.year +2000));
        debug_outlnF(F("** BAD Cell time. Using GPS RTC")); 
        WatchDogTimer.enable(15000, 0); // reenable here 3v04
        WatchDogTimer.reset(); 
        return;   
      }
    
    else{
        debug_outlnF(F("** ERROR Bad time, Cell Time and Bad GPS RTC"));
        DEBUG_STREAM.println(F("** ERROR Bad time, Cell Time and Bad GPS RTC"));
        return;
        }
   #endif
    }  
    
}


void get_imei(char * answer)
{
  char temp[32];
  char * str_ptr;
  int byte_count=0;
  /*
 * Now GET the IMEI number as it becomes the system's serial number and Token ID
 */
 while(MODEM_STREAM.available()) MODEM_STREAM.read(); // clear out receive buffer
 MODEM_STREAM.println("ATE0"); 
  if (validate_OK_resp()){   
   MODEM_STREAM.println(F("AT+CGSN=1"));
   debug_outln1("AT+CGSN=1");
   delay(800);
   byte_count = MODEM_STREAM.readBytes(temp, 32);
   temp[byte_count]=0;
   debug_outln1(temp);
   str_ptr = strstr(temp, "CGSN:");
     if (str_ptr !=0){
      strncpy(answer, str_ptr+5, 15);
       }
     else
       debug_outln1("Error Retreiving IMEI number");
  }
  else 
      debug_outln1("Error Retreiving IMEI number, No Response");
}


void get_imsi(char *answer)
{
  char temp[32];
  int byte_count=0;
  char * str_ptr;
  /*
 * Now GET the IMSI number as it is useful for Vodafone tech support
 */
 while(MODEM_STREAM.available()) MODEM_STREAM.read(); // clear out receive buffer
 MODEM_STREAM.println("ATE0"); 
  if (validate_OK_resp()){   
   MODEM_STREAM.println("AT+CIMI");
   debug_outln1("AT+CIMI");
   delay(800);
  byte_count = MODEM_STREAM.readBytes(temp,24 );
  temp[byte_count]=0;
   debug_outln1(temp);
   str_ptr = strstr(temp,"\r\n9");
      if (str_ptr !=0){
        strncpy(answer, str_ptr+2, 15);
        return; 
       }
     else
       debug_outln1("Error Retreiving IMSI number");
  }
  else 
      debug_outln1("Error Retreiving IMSI number, No Response");
}

void get_cell_id(char *answer)
{
 char temp[256];
  int byte_count=0;
  char * str_ptr;
 while(MODEM_STREAM.available()) MODEM_STREAM.read(); // clear out receive buffer
 MODEM_STREAM.println("ATE0"); 
  if (validate_OK_resp()){   
   MODEM_STREAM.println(F("AT+NUESTATS"));
   delay(800);
   byte_count = MODEM_STREAM.readBytes(temp, 192);
   temp[byte_count]=0;
   debug_outln1(temp);
   str_ptr = strstr(temp, "Cell ID:");
   if (str_ptr !=0){
        strncpy(answer, str_ptr+8, 8);
        return;
   }
       debug_outln1("Error Retreiving Cell ID");
  }
  else 
      debug_outln1("Error Retreiving Cell ID, No Response");
}


void get_network_status(char * answer)
{
  char temp[32];
  int byte_count=0;
  char * str_ptr;
  if (digitalRead(powerPin) == 1) {
     while(MODEM_STREAM.available()) MODEM_STREAM.read(); // clear out receive buffer
     MODEM_STREAM.println("ATE0"); 
     if (validate_OK_resp()){   
        MODEM_STREAM.println(F("AT+CEREG?"));
        debug_outln1("AT+CEREG?");
        delay(800);
        byte_count = MODEM_STREAM.readBytes(temp, 192);
        temp[byte_count]=0;
        debug_outln1(temp);
        str_ptr = strstr(temp, "\r\n+CEREG:");
        if (str_ptr !=0){
           strncpy(answer, str_ptr+10, 8);
           return;
           }
        else
           debug_outln1("Error Network Status");
        }
     else 
       debug_outln1("Error Retreiving Network Status, No Response");
     }
}

void get_csq(char * answer)
{
  /*
 * Get the connected signal quality 0=noise floor, 1 - 6 weak, 7- 14 ok, 15 - 30 very good
 */
 char temp[32];
  int byte_count=0;
  char * str_ptr;
 while(MODEM_STREAM.available()) MODEM_STREAM.read(); // clear out receive buffer
 MODEM_STREAM.println(F("AT")); 
  if (validate_OK_resp()){   
   MODEM_STREAM.println(F("AT+CSQ"));
   debug_outln1("AT+CSQ");
   myDelayMs(500);
   byte_count = MODEM_STREAM.readBytes(temp, 16);
   temp[byte_count]=0;
   debug_outln1(temp);
   str_ptr = strstr(temp, "\r\n+CSQ:");
   if (str_ptr !=0){
           strncpy(answer, str_ptr+7, 2);
        return;
       }
     else{
       debug_outln1("Error Retreiving CSQ signal quality");
       strcpy(answer,"Err");
       return;
     }
  }
  else {
      debug_outln1("Error Retreiving CSQ, No Response");
      strcpy(answer,"Err");
      return;
  }
}



void debug_outln1(char * msg)
{
// debug out is sent to the serial port as well as the SD card.
DEBUG_STREAM.println(msg);
if (sd_card_enabled)  {
   if (logFile) {
              logFile.println(msg);
              logFile.flush();
             }
    else
     DEBUG_STREAM.println(F("SD Card Write Error"));
    }
}

void debug_out1(char * msg)
{
// debug out is sent to the serial port as well as the SD card.
DEBUG_STREAM.print(msg);
if (logFile and sd_card_enabled) {
              logFile.print(msg);
             }
}


// This function supports printing constant strings from Flash so saves RAM
void debug_outlnF(const __FlashStringHelper *msg)
{
  DEBUG_STREAM.println(msg);
if (sd_card_enabled)  {
   if (logFile) {
              logFile.println(msg);
              logFile.flush();
             }
    else
     DEBUG_STREAM.println(F("SD Card Write Error"));
    }
}

// This function supports printing constant strings from Flash so saves RAM
void debug_outF(const __FlashStringHelper *msg)
{
// debug out is sent to the serial port as well as the SD card.
DEBUG_STREAM.print(msg);
if (logFile and sd_card_enabled) {
              logFile.print(msg);
             }
}

void get__nbiot_type( char * answer)
{
  char temp[192];
  int byte_count = 0;
 while (MODEM_STREAM.available()){ byte_count = MODEM_STREAM.readBytes(temp,191); temp[byte_count]=0;DEBUG_STREAM.print(temp);} // flush receive buffer
 DEBUG_STREAM.println(F("Sending AT.."));
 MODEM_STREAM.println("AT"); 
 DEBUG_STREAM.println("AT");
 validate_OK_resp();
 MODEM_STREAM.println("ATI");
 delay(2000);
 //board_type = MODEM_STREAM.readString();
 byte_count = MODEM_STREAM.readBytes(temp, 127);
 temp[byte_count] = 0;
 myDelayMs(1000);
 DEBUG_STREAM.println(temp);
 myDelayMs(1000);
// board_type = MODEM_STREAM.readString();
// DEBUG_STREAM.println(board_type);
 
   if ( strstr(temp,"BC95-G") != 0){
      strcpy(answer, "BC95-G");
      return;
   }
   else{
     if ( strstr(temp,"BC95-B20") != 0)  
       strcpy(answer, "BC95-B20");
       return;
   }
   strcpy(answer, "UNKNOWN");
   return; 
}

/* *****************************************************
 *  Converts floating point to a string x.x            *
 *  Places resulting string into memory pointed by buf *
 */
void float_to_str( char*buf, float val)
{
  int whole, fract;
  whole = (int)val;
  fract = (int)(10* val) - 10*whole;
  sprintf(buf, "%d.%d", whole, fract);
}


void int_to_str( char*buf, int val)
{
 
  sprintf(buf, "%d", val);
}

//**************************************************************************
// Can use these function for RTOS delays
// Takes into account procesor speed
//**************************************************************************

void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}

//*****************************************************************
// Task will periodically print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
// Run time stats are generated from all task timing collected since startup
// No easy way yet to clear the run time stats yet
//*****************************************************************



static char ptrTaskList[400]; //temporary string bufer for task stats

void prvtaskMonitor(void *pvParameters)
{
    int x;
    int measurement;
    
    
    DEBUG_STREAM.println(F("Task Monitor: Started"));

    // run this task afew times before exiting forever
    
    while(1)
    {
      
      myDelayMs(62000); // print every 

      DEBUG_STREAM.println(F("****************************************************"));
      DEBUG_STREAM.print(F("Free Heap: "));
      DEBUG_STREAM.print(xPortGetFreeHeapSize());
      DEBUG_STREAM.println(F(" bytes"));

      DEBUG_STREAM.print(F("Min Heap: "));
      DEBUG_STREAM.print(xPortGetMinimumEverFreeHeapSize());
      DEBUG_STREAM.println(" bytes");

      DEBUG_STREAM.println(F("****************************************************"));
      DEBUG_STREAM.println(F("Task            ABS             %Util"));
      DEBUG_STREAM.println(F("****************************************************"));

      vTaskGetRunTimeStats(ptrTaskList); //save stats to char array
      DEBUG_STREAM.println(ptrTaskList); //prints out already formatted stats

    DEBUG_STREAM.println(F("****************************************************"));
    DEBUG_STREAM.println(F("Task            State   Prio    Stack   Num     Core" ));
    DEBUG_STREAM.println(F("****************************************************"));

    vTaskList(ptrTaskList); //save stats to char array
    DEBUG_STREAM.println(ptrTaskList); //prints out already formatted stats

    DEBUG_STREAM.println(F("****************************************************"));
    DEBUG_STREAM.println(F("[Stacks Free Bytes Remaining] "));

    measurement = uxTaskGetStackHighWaterMark( Handle_HubTask );
    DEBUG_STREAM.print(F("Thread HubTask: "));
    DEBUG_STREAM.println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_NoiseTask );
    DEBUG_STREAM.print(F("NoiseTask: "));
    DEBUG_STREAM.println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_VibTask );
    DEBUG_STREAM.print(F("VibTask: "));
    DEBUG_STREAM.println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_AttrTask );
    DEBUG_STREAM.print(F("AttrTask: "));
    DEBUG_STREAM.println(measurement);

     measurement = uxTaskGetStackHighWaterMark( Handle_AttrTask );
    DEBUG_STREAM.print(F("ClITask: "));
    DEBUG_STREAM.println(measurement);
    

    measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
    DEBUG_STREAM.print(F("Monitor Stack: "));
    DEBUG_STREAM.println(measurement);

    DEBUG_STREAM.println(F("****************************************************"));

    }

    // delete ourselves.
    // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
    DEBUG_STREAM.println(F("Task Monitor: Deleting"));
    vTaskDelete( NULL );

}




/* **********************************************************************
 * NB-IoT Comms Task 
 * **********************************************************************
 */
static void prvTaskNB_IoT_Hub( void *pvParameters )
 {
led_blink_mode_t led_state;
generic_struct_t  telem; // structure that will accept any telemetry or attributes from xQueue
time_leq_struct_t tls; // noise structure
time_vib_struct_t tvs;
generic_struct_t  tps; // battery percentage
char board_type[32];
static char temp_str[64];
//MODEM_STREAM.println(F("AT"));
//DEBUG_STREAM.println(F("Resetting NB_IoT: AT+NRB..."));
//MODEM_STREAM.println(F("AT+NRB"));
myDelayMs(4000);
//MODEM_serialEvent();
DEBUG_STREAM.println(F(SW_VERSION));
get__nbiot_type(board_type);
  if (strstr(board_type,"BC95-B20") !=0){
    debug_out1(board_type);debug_outln1("SixFab NB-IoT Module Detected"); 
  }
  else{ if (strstr(board_type,"BC95-G") !=0) {
           debug_outln1("BC95-G Glenside NB-IoT Module Detected");
       }
       else {
        debug_out1(board_type);debug_outln1(" NB-IoT board detected");
       }
  }
  DEBUG_STREAM.print(F("Retreiving IMEI..."));

get_imei(serial_no);// IMEI is 15 digits long
strcpy(token_id, "IMEI:");
strcat(token_id, serial_no);
DEBUG_STREAM.print(F("token_id= "));DEBUG_STREAM.println(token_id);
strtohex((unsigned char *)token_id, strlen(token_id), access_token, strlen(token_id)*2);
 debug_outln1(access_token);

 while( get_batt_voltage() <=MIN_BATT_VOLTAGE + 0.1){ // wait until battery gets enough charge for the IOT not to brown out the supply
  myDelayMs(10000);
 }
 //SETUP THE NB-IOT HERE
  #ifdef NBIOT_ENABLED
  if (initialise_nbiot(FIRST_START, KEEP_CELL_LIST)== false){
     DEBUG_STREAM.println(F("Failed to configure NB-IoT"));
     connect_state = retry1;
     get_imsi(imsi_no); // IMSI is 15 digits long
     debug_out1("IMSI:");
     debug_outln1(imsi_no);
     }
  else {
     get_imsi(imsi_no); // IMSI is 15 digits long
     debug_out1("IMSI:");
     debug_outln1(imsi_no);
     if (connected_flag == true){
        if (connect_nbiot() == false){   
           DEBUG_STREAM.println(F("ERROR101: NB-IoT Connection Failure"));
           get_time();
           connect_state = retry1;
        }
        else {
          delay(4000);
          get_time();  // Synchronise the RTC to the cellular network time
          delay(400);
          start_time = now();  // capture time of boot for posting as attrbute
          connect_state = live;
          WatchDogTimer.reset();
          get_cell_id(cell_id); // Cell ID is 8 digits long
          DEBUG_STREAM.print(F("Cell ID:")); DEBUG_STREAM.println(cell_id);
          transmit_pkt(info_attr, false);
          transmit_pkt(status_attr, true);
          // Start 5 minute network diconnect timer here     ********************************************************************************************
          Network_Keepconnect_Timer.reset();
          Network_Keepconnect_Timer.enabled = 1;    
        }
    } 
    else
    {
      connect_state = disconnected;  
      get_time();
    }
}
#endif  


#ifdef GPS_VERSION
    gps_init(&hgps);
    GPS_serialEvent();
    GPS_CLI_Check();
    myDelayMs(1000);
    GPS_serialEvent();
    GPS_CLI_Check();
    char temp_gps_buf[200];
    sprintf(temp_gps_buf,"GPS Time: %d/%d/20%d  %02d:%02d:%02d",hgps.date, hgps.month,hgps.year,hgps.hours,hgps.minutes, hgps.seconds);
    debug_outln1(temp_gps_buf);
#endif

DEBUG_STREAM.print(F("Enabling watchdog timer.......SET TO "));
DEBUG_STREAM.print(WatchDogTimer.enable(15000, 0));
DEBUG_STREAM.println("ms");
WatchDogTimer.reset();

Watchdog_Reset_Timer.duration = WATCHDOG_RESET_INTERVAL;
Watchdog_Reset_Timer.reset();




DEBUG_STREAM.println(F("###HUB Task started###"));
  while(1)
   {
myDelayMs(100);
    DEBUG_serialEvent();
    MODEM_serialEvent();

//if( xQueueReceive( x_iot_queue, &telem, portMAX_DELAY ) == pdPASS )
if( xQueueReceive( x_iot_queue, &telem, 200 ) == pdPASS )
        {
          //DEBUG_STREAM.println(F("*** pkt rx into Hub Task***"));
            // push data onto one of the 3 internal non FreeRTOS queues so they are sorted by telemetry type
            // alarm Leq or vib or battery get pushed to the fron of the queue  
            switch (telem.tt) {
                  case noise:
                       memcpy(&tls, &telem, sizeof(tls));        
                       noise_q.push(tls); // push the timestamped leq data onto the queue for later assembly into a json payload
                       DEBUG_STREAM.print(F("*  Noise_q.count="));DEBUG_STREAM.println(noise_q.count());
                    break;
                  case vibrate:
                        memcpy(&tvs, &telem, sizeof(tvs)); 
                        vibration_q.push(tvs);
                        DEBUG_STREAM.print(F("*  vibration_q.count="));DEBUG_STREAM.println(vibration_q.count());
                    break;
                  case vib_alarm: //TODO:  Need to find a way to escalate vibration alarm to immediate transmit_pkt
                        memcpy(&tvs, &telem, sizeof(tvs)); 
                        vibration_q.push(tvs);
                        DEBUG_STREAM.print(F("* V Alrm  vibration_q.count="));DEBUG_STREAM.println(vibration_q.count());
                  break;
                  case power:
                       memcpy(&tps, &telem, sizeof(tps)); 
                       power_q.push(tps);
                       DEBUG_STREAM.print(F("*  power_q.count="));DEBUG_STREAM.println(power_q.count());
                       
                  case status_attr:

                    break;                
            }
        }
 //DEBUG_STREAM.print(".");


 // THis performs a watchdog timer reset around every 6 seconds while the HW WDT is set to 8 seconds
// The reason for the infrequent reset is that it takes 5.8ms which would slow down the loop and 
// interrupt noise capture and processing.
  if (Watchdog_Reset_Timer.check() ==1){
      WatchDogTimer.reset(); 
      //  debug_outln("RWDT"); 
      Watchdog_Reset_Timer.reset();
}

  // ************************************************************************************************
// Data Upload Timer Check
//*************************************************************************************************

  if (Data_Upload_Timer.check() == 1){
    DEBUG_STREAM.println(F("upload timer up"));
      sprintf(temp_str,"***Data_Upload_Timer up,  connect state=%d", connect_state);
      debug_outln1(temp_str);
       WatchDogTimer.reset();  // placed here after WDT reset occured
      if ((Check_connected() == false) and (connected_flag==true) )    
         {
        get_network_status(network_status);
        DEBUG_STREAM.println(network_status);
        switch (connect_state) {
         
        case live :
           connect_state = retry1;
           debug_outln1("NB-IoT connect error: Retrying 1st try");
            debug_outln1(" Applying a forced reset  AT+NRB");
            MODEM_STREAM.println("AT+CFUN=0");
            myDelayMs(2000);
            WatchDogTimer.reset();
            myDelayMs(3000);
            WatchDogTimer.reset();
            myDelayMs(3000);
            WatchDogTimer.reset();
            initialise_nbiot(WARM_START, KEEP_CELL_LIST);
           break;
        case retry1 :
           connect_state = retry2;
           debug_outln1("NB-IoT connect error: Retrying 2nd try");
            debug_outln1(" Applying a forced reset  AT+NRB");
            MODEM_STREAM.println("AT+CFUN=0");
            myDelayMs(5000);
            WatchDogTimer.reset();
            myDelayMs(5000);
            WatchDogTimer.reset();
            myDelayMs(5000);
            WatchDogTimer.reset();
            initialise_nbiot(WARM_START, CLEAR_CELL_LIST);
           break;
        case retry2 :
           connect_state = backoff1;
           debug_out1("NB-IoT connect error: 2 attempts failed, backing off and powering down NB-IoT module for ");
           WatchDogTimer.reset();
           sprintf(temp_str,"%d seconds");
           debug_outln1(temp_str);

           WatchDogTimer.reset();
           MODEM_STREAM.println("AT+CFUN=0");
           myDelayMs(5000);
            WatchDogTimer.reset();
            myDelayMs(5000);
            WatchDogTimer.reset();
            myDelayMs(5000);
            WatchDogTimer.reset();
           digitalWrite(powerPin, LOW); // power down the NB-IoT Shield for the backoff period
           MODEM_STREAM.end();
           WatchDogTimer.reset();
           #ifdef GPS_VERSION
             GPS_STREAM.end();
           #endif
          // pinMode(TX_PIN, INPUT); // disable pin to stop 3.3V goinginto BC95 module.
           Backoff_Timer.reset();
           break;

         case disconnected :
            connect_state = connecting;
         break;

         case connecting :
           connect_state = retry1;
         break;
       
        case backoff1 :
           if (Backoff_Timer.check() == 1) {
              debug_out1("Powering up NB-IoT module");
             initialise_nbiot(COLD_START, CLEAR_CELL_LIST);
              //pinMode(TX_PIN, OUTPUT); // enable TX pin.
              myDelayMs(1000);
              connect_state = retry1;
           }
           break;
        }
       if ((connect_state == retry1) or (connect_state == retry2) or (connect_state == connecting)  )
          {
           
            if (connect_state != connecting)
                nb_iot_rst_cnt++;
            myDelayMs(500); 
            
            if (connect_nbiot() == false){
//               
               sprintf(temp_str,"ERROR102: NB-IoT Connection Failure reset_count=%d",nb_iot_rst_cnt);
               debug_outln1(temp_str);
            }
            else { 
               connect_state = live;
               get_cell_id(cell_id); // Cell ID is 8 digits long
               DEBUG_STREAM.print(F("Cell ID:")); DEBUG_STREAM.println(cell_id);
               transmit_pkt(info_attr, false);
            }
          }
      }



else { //**** we are connected to the network here


 
        
         get_time();  // update the RTC here with the latest time 
         if (start_time < 10000)  //This corrects the start_time if it was incorrectly set to 1970 as a result of 1st attempt to connect to NB-IoT failed.
             start_time = now();
         Network_Keepconnect_Timer.reset();  // resets the keep connect timer so that we dont lose connection if previous connection was a vibration alarm
         Network_Keepconnect_Timer.enabled = 1;
         WatchDogTimer.reset(); 
       //DEBUG_STREAM.print(".");


         while (power_q.count() >= PWR_PAYLOAD_SIZE){
          DEBUG_STREAM.println(F("DBG: transmit power pkt)"));
          transmit_pkt(power, false);
         }
       
         while (noise_q.count() >= 2* LEQ_PAYLOAD_SIZE ){ // Enough Leq samples to assemble at lease 2  packets
             DEBUG_STREAM.println(F("DBG: transmit noise pkt)"));
             transmit_pkt(noise, false);
            }
            
         if (noise_q.count() >= LEQ_PAYLOAD_SIZE ) // we should have one noise packet left?
         {
           if (vibration_q.count() < VIB_PAYLOAD_SIZE )  // there there is no vibration data to go out, then set the release RAI indiactor flag
            transmit_pkt(noise, true);
           else
            transmit_pkt(noise, false); // nope, there will be vibration data to go out
         }
//DEBUG_STREAM.print(".");
            
         while (vibration_q.count() >= 2*VIB_PAYLOAD_SIZE ){ // Enough vib samples to assemble the packet and switch over to audio next time
             transmit_pkt(vibrate, false);
            }
         if  (vibration_q.count() >= VIB_PAYLOAD_SIZE )  // last vibration packet so will set release indicator.
              {
               transmit_pkt(vibrate, true);
              }
 //DEBUG_STREAM.print(".");             
         // Start a 5 minute timer here to disconnect the network on expiration so that buffered data will have had plenty of time to be transmitted
         Network_Keepconnect_Timer.reset();
         Network_Keepconnect_Timer.enabled = 1;
         Data_Upload_Timer.duration = DATA_UPLOAD_INTERVAL;
         Data_Upload_Timer.reset();  debug_outln1("***RESET Data_Upload_Timer");         
           
         if (attribute_update == ATTRIBUTE_ITERATION_CNT) {
            #ifdef GPS_VERSION
           //   GPS_CLI_Check(); // parse the latest from the GPS and fill the hgps structure.
            DEBUG_STREAM.print(F("GPS Date: "));DEBUG_STREAM.print(hgps.date);DEBUG_STREAM.print("/");DEBUG_STREAM.print(hgps.month);DEBUG_STREAM.print("/");DEBUG_STREAM.print(hgps.year);
            DEBUG_STREAM.print(F("Time: "));DEBUG_STREAM.print(hgps.hours);DEBUG_STREAM.print(":");DEBUG_STREAM.print(hgps.minutes);DEBUG_STREAM.print(":");DEBUG_STREAM.println(hgps.seconds);
            #endif
            get_time();  // update the RTC here with the latest time 
            transmit_pkt(status_attr, true);
            //RESET ALL THE IMPORTANT TIMERS HERE AS RTC HAS BEEN UPDATED
            WatchDogTimer.reset();
            Watchdog_Reset_Timer.reset();
            Data_Upload_Timer.reset();
            Network_Keepconnect_Timer.reset();
           
            // transmit_pkt(info_attr); 
            attribute_update = 0;
         }
         else attribute_update++;
         
      }  
  
  }

//DEBUG_STREAM.print("#");

    if (Network_Status_Timer.check() ==1){
       //DEBUG_STREAM.println("checking network status");
      #ifdef GPS_VERSION
         GPS_serialEvent();
         GPS_CLI_Check();  
      #endif
     if (Check_connected() == true){
      led_state.bt = network;
      led_state.val = 2; // network val = 0, disconnected; val=1 connecting, val=2 online, val=3 transmitting data.
      xQueueSend( x_led_queue, &led_state, 0 );
      //digitalWrite(blueLed, HIGH); //kk
      connect_state = live;
     }
     else {
      //digitalWrite(blueLed, LOW); //kk
      led_state.bt = network;
      led_state.val = 1; // network val = 0, disconnected; val=1 connecting, val=2 online, val=3 transmitting data.
      xQueueSend( x_led_queue, &led_state, 0 );
      // dont want to set connect_state here as it could be retrying in a future version
     }
      Network_Status_Timer.reset();
    }
    
   }
  DEBUG_STREAM.println(F("Thread NB_IoT_Hub: Deleting"));
  vTaskDelete( NULL );
 }



/* **********************************************************************
 * Noise Task 
 * **********************************************************************
 */
static void prvTaskNoise( void *pvParameters )
 {  DEBUG_STREAM.println(F("Noise Task Started")); 
 
 
  int count;
   filter1Type *inst_filter_H1;
//   filter2Type *inst_filter_H2;
   float * ptrNoise_In_Buf;
 
static   uint16_t adc_noise_buf[ADC_BLOCKSIZE];
static   float noise_in_buf[ADC_BLOCKSIZE];
static   float noise_filt1_buf[ADC_BLOCKSIZE];
//static   short noise_filt2_buf[ADC_BLOCKSIZE];
   float32_t Mean_square;

   float * ptrNoise_Filt1_Buf;
//   short * ptrNoise_Filt2_Buf;
   float32_t * ptrMean_square;
   float32_t acc_mean_squareA;
   float32_t acc_mean_squareB;
   float32_t acc_mean_squareC;
   
DEBUG_STREAM.print(F("Initialising noise filter1"));
   inst_filter_H1 = filter1_create();
//   DEBUG_STREAM.print(" ,Filter2"); 
//   inst_filter_H2 = filter2_create();
DEBUG_STREAM.println(F(": Done"));   
   ptrNoise_In_Buf = noise_in_buf;//(short*) malloc (ADC_BLOCKSIZE * sizeof(short)); // buffer between ADC buffer and filter_H1
   ptrNoise_Filt1_Buf = noise_filt1_buf;//(short*) malloc (ADC_BLOCKSIZE * sizeof(short)); // buffer between filter_H1 and filter_H2
//   ptrNoise_Filt2_Buf = noise_filt2_buf;//(short*) malloc (ADC_BLOCKSIZE * sizeof(short)); // buffer for filter_H2 output
   ptrMean_square = &Mean_square;//(q63_t *) malloc (2 * sizeof(q63_t)); // Stores the mean square of a ADC_BLOCKSIZE array of values. ADC_BLOCKSIZE restricted to 64 samples max. 
   //initialise the noise buffer so there is valid data on first connect.
   myDelayMs(15000); // let the HUB-IoT task get setup.
   digitizer.Start_transfer(1, ONE_SHOT, adc_noise_buf, ADC_BLOCKSIZE); // Grab the next block whileprocessing the previous
   acc_mean_squareA = 0;
   acc_mean_squareB = 0;
   acc_mean_squareC = 0;
   
   time_leq_struct_t tls ;  // variable to hold timestamp and leq pair for pushing onto queue
   tls.tt = noise;

   while( get_batt_voltage() <=MIN_BATT_VOLTAGE+0.11){ // wait until battery gets enough charge for the Noise capture not to brown out the supply
  myDelayMs(5000);
 }
  DEBUG_STREAM.println(F("###NOISE Task started###"));
  while(1)
   {  
      
  /* *****************************************
    *  NOISE CAPTURE AND PROCESSING SECTION  *
    *  ***************************************
    */ 

   if (dmadone[1] == 1 && enable_noise_flag==1){ // ready for another transfer
   
      for(int i=0; i < ADC_BLOCKSIZE; i++){
         *(ptrNoise_In_Buf + i) = (float)(adc_noise_buf[i] - 4096);
         //*(ptrNoise_In_Buf + i) = 1;
         //DEBUG_STREAM.println(*(ptrNoise_In_Buf + i));
      }
      digitizer.Start_transfer(1, ONE_SHOT, adc_noise_buf, ADC_BLOCKSIZE); // Grab the next block whileprocessing the previous
      myDelayUs(1000);
      
      count = filter1_filterBlock(inst_filter_H1, ptrNoise_In_Buf, ptrNoise_Filt1_Buf, ADC_BLOCKSIZE);
  //    count = filter2_filterBlock(inst_filter_H2, ptrNoise_Filt1_Buf, ptrNoise_Filt2_Buf, ADC_BLOCKSIZE);

 //       for(int i=0; i < ADC_BLOCKSIZE; i++) {
 //          DEBUG_STREAM.print(i);DEBUG_STREAM.print(":  "); 
 //          DEBUG_STREAM.print(*(ptrNoise_In_Buf + i));DEBUG_STREAM.print("  "); DEBUG_STREAM.println(*(ptrNoise_Filt1_Buf + i));
 //         }
 
     // arm_power_q15( (q15_t *)ptrNoise_Filt2_Buf ,ADC_BLOCKSIZE, ptrMean_square);
      arm_power_f32( (float32_t *)ptrNoise_Filt1_Buf ,ADC_BLOCKSIZE, ptrMean_square);

      acc_mean_squareB += *ptrMean_square/ADC_BLOCKSIZE;
      acc_mean_squareA += *ptrMean_square/ADC_BLOCKSIZE;
      acc_mean_squareC += *ptrMean_square/ADC_BLOCKSIZE;
      
      noise_block_countB++;
      noise_block_countA++;
      noise_block_countC++;
        
    }

   
  //  digitalWrite(TEST_PIN12,LOW);

 //   /* This bit is from the DUE code and had the following characteristics
 //   *  1) The analog acquisition was blocking in that the program waited until the block of data was acquired
 //   *  2) Audio and Vibration were mutually exclusive as they shared the one ADC
 //   *  Our new scheme is non blocking so we need a method to process new dtaa as it becomes available and to skip
 //   *  when there is no new data sotah the loop continues ti loop. THis can be done with a 
 //   *  a ring buffer FIFO storing the output of the filters and then we can pop the data when there is enough to fill a payload
 //   *  for the transmission over NB-IoT.
 //   *  
 //   


 
    if (noise_block_countA == leq_iterationsA){
     //   Leq = 10 * log10((uint32_t)(acc_mean_squareA/leq_iterationsA));
        Leq = 10 * log10(acc_mean_squareA/leq_iterationsA);
        Leq+=mic_cal;
        if ((uint32_t)Leq > LeqMx)
          LeqMx = (uint32_t)Leq;
        noise_block_countA = 0;
        acc_mean_squareA = 0;
        audioAlarm.post((int)Leq);
        Leq10.post((int)Leq);
        Leq90.post((int)Leq);
        //DEBUG_STREAM.print("Noise= ");DEBUG_STREAM.print(Leq);DEBUG_STREAM.println("dB\r");
    }


 if (noise_block_countC== leq_iterationsC){
    if (calibration_enabled == 1){
        Leq = 10 * log10(acc_mean_squareC/leq_iterationsC);
        Leq+=mic_cal;    
        acc_mean_squareC = 0;
        
            DEBUG_STREAM.print(F("Noise= "));DEBUG_STREAM.print(Leq);DEBUG_STREAM.print("dB\r");
        }
        noise_block_countC = 0;
    }




    if (noise_block_countB == leq_iterationsB){    
       // Leq = 10 * log10((uint32_t)(acc_mean_squareB/leq_iterationsB));
        Leq = 10 * log10(acc_mean_squareB/leq_iterationsB);
        Leq+=mic_cal;
//        debug_out( "  Leq=");debug_out( Leq);
        //debug_out( "dac[0]=");
        //for (i=0;i<ADC_BLOCKSIZE;i++) {debug_out(" ,"); debug_out( aaAudio.dacBuffer16[i]);}
        //debug_outln("*");debug_out("accum msq=");debug_outln((uint32_t)(acc_mean_squareB/leq_iterationsB));
        noise_block_countB = 0;
        acc_mean_squareB = 0;
        tls.ts = now();  // add the timestamp
        tls.leq = Leq;
        tls.leq10 = (float)Leq10.get_percentile();
        tls.leq90 = (float)Leq90.get_percentile();
        tls.leqmx = (float)LeqMx;
        Leq10.reset(); 
        Leq90.reset();
        LeqMx = 0; 
 
 
        if( xQueueSend( x_iot_queue, &tls, 0 ) != pdPASS )
        {
            debug_outln1("  Failed to post the message, probably due to queue full ");        
        }
        else {
          DEBUG_STREAM.println(F("xQueue Snd Noise"));
        }
    
        
        
        audioAlarm.post((int)Leq);
      
     }
     
   }
   
  DEBUG_STREAM.println(F("Thread Noise: Deleting"));
  vTaskDelete( NULL );
 }

/* **********************************************************************
 * Vibration Task 
 * **********************************************************************
 */ 
static void prvTaskVib( void *pvParameters )
{
int k;  
long dc_offset_x, dc_offset_y, dc_offset_z;
long vibx_avg=0;
long viby_avg=0;
long vibz_avg=0;
long vibx_max=0;
long viby_max=0;
long vibz_max=0;
float f_vibx = 0;
float f_viby = 0;
float f_vibz = 0;
float f_vibxyz;
static uint16_t adc_vib_buf[VIB_ADC_BLOCKSIZE];
static  long vibration[3][VIB_ADC_BLOCKSIZE/3]; // x at AIN5 (A1), Y at AIN6 (A2) and Z at AIN4 (A3)
static time_vib_struct_t tvs; // variable to hold timestamp and vibration data for pushing onto queue
tvs.tt = vibrate;
static char temp[64];
char temp1[8], temp2[8];
myDelayMs(12000); // give the hub task a chance to start and connect to the network
 //initialise the vibration buffer so there is valid data on first connect.
digitizer.Start_transfer(0, ONE_SHOT, adc_vib_buf, VIB_ADC_BLOCKSIZE); //Start the next block capture
myDelayMs(200);
while( get_batt_voltage() <=MIN_BATT_VOLTAGE+0.11){ // wait until battery gets enough charge for the Vibration task not to brown out the supply
  myDelayMs(5000);
 }
DEBUG_STREAM.println(F("###VIB Task started###"));
 while(1)
   {
   /* *******************************************
  *  VIBRATION CAPTURE AND PROCESSING SECTION *
  * *******************************************
  */ 
 
myDelayMs(5);

 if (enable_measurements == 1){
    if (dmadone[0] == 1 && enable_vibration_flag ==1){ // ready for another transfer  
    /*
        * We need to convert the ADC data to Q15 format for RMS calculations. 
        * ADC data is 12 bits offset binary so zero appears around 0x7fff 0x8000 0x8001
        * To convert offset binary to 2s compliment, the msb should be inverted by XOR'ing with 0x8000
        * and then sign extended. Subtracting 2048 from a 12 bit binary offset value also converts to 2s compliment.
        * To normalise the ADC12 bit to Q15, bit shift left by 4.
        * In the Metro M4 implementation, we are digitizing vibration to 14 bits with some noise reduction.
       */

    // Copy and convert the dma ADC data to signed Binary as it is sitting on an offset of 8192
    // remove any DCoffset by averaging the block of data and subtractig the average from each axis
     k = 0;
     dc_offset_x = 0;
     dc_offset_y = 0;
     dc_offset_z = 0;
     for(int i=0; i < VIB_ADC_BLOCKSIZE/3; i++){
       dc_offset_x += adc_vib_buf[k++];
       dc_offset_y += adc_vib_buf[k++];
       dc_offset_z += adc_vib_buf[k++];   
     }
     dc_offset_x = dc_offset_x/(VIB_ADC_BLOCKSIZE/3);
     dc_offset_y = dc_offset_y/(VIB_ADC_BLOCKSIZE/3);
     dc_offset_z = dc_offset_z/(VIB_ADC_BLOCKSIZE/3);
     k = 0;  

     //debug_out("dc_offset_x = ");debug_outln(dc_offset_x);
  
     for(int i=0; i < VIB_ADC_BLOCKSIZE/3; i++){
       vibration[0][i] = (adc_vib_buf[k++]-dc_offset_x) << 12; // was << 14 for 12 bit ADC data 
       vibration[1][i] = (adc_vib_buf[k++]-dc_offset_y) << 12; // was << 14 for 12 bit ADC data
       vibration[2][i] = (adc_vib_buf[k++]-dc_offset_z) << 12; // was << 14 for 12 bit ADC data
      //       DEBUG_STREAM.print("vibx=");DEBUG_STREAM.print((int)vibration[0][i]>>12);
      //       DEBUG_STREAM.print(",   viby=");DEBUG_STREAM.print((int)vibration[1][i]>>12);
      //       DEBUG_STREAM.print(",   vibz=");DEBUG_STREAM.println((int)vibration[2][i]>>12);
     }

   digitizer.Start_transfer(0, ONE_SHOT, adc_vib_buf, VIB_ADC_BLOCKSIZE); //Start the next block capture

     myDelayMs(195);
 

     // get the max values instead of avg
     for(int i=0; i < VIB_ADC_BLOCKSIZE/3; i++){
      if (abs(vibration[0][i]) > vibx_max)
        vibx_max = abs(vibration[0][i]);
      if (abs(vibration[1][i]) > viby_max)
        viby_max = abs(vibration[1][i]);
      if (vibration[2][i] > vibz_max)
        vibz_max = abs(vibration[2][i]);


     //      DEBUG_STREAM.print("x_max=");DEBUG_STREAM.print((int)vibx_max>>12);
     //      DEBUG_STREAM.print(",   y_max=");DEBUG_STREAM.print((int)viby_max>>12);
     //      DEBUG_STREAM.print(F(",   z_max="));DEBUG_STREAM.println((int)vibz_max>>12);       
     }

     // calaculate ppv here for alarm PPV level comparison
   
     f_vibx = (float)(vibx_max >> 14) * ADC_CONV_RATIO;
     f_viby = (float)(viby_max >> 14) * ADC_CONV_RATIO;
     f_vibz = (float)(vibz_max >> 14) * ADC_CONV_RATIO; 
// ******************************************************************************************************************************************************EDIT THE ALARM SECTION
     /* 
      *  Vibration Alarm will immediately connect to network to upload all data while the alarm is valid
      *  The network will then stay connected with the keepconnected timer until itexpires. It also calculates FFT
      *  when an alarm condition is present
      */
      //   if ((f_vibx > VIB_ALARM_PPV_LEVEL_MM) or (f_viby > VIB_ALARM_PPV_LEVEL_MM) or (f_vibz > VIB_ALARM_PPV_LEVEL_MM) ){
      if ((f_vibx > 15) or (f_viby > 15) or (f_vibz > 15) ){
         // Connect to NB-IoT and post alarm data. Later do FFT here as well
    //     debug_outln1("** Vibration Alarm **");
/* kos 7/4/2021       
         if ((Check_connected() == true) ){ 
             connect_state = live;      
             Network_Keepconnect_Timer.reset();
             Network_Keepconnect_Timer.enabled = 1;  
         }
      else {
           if (connected_flag == true){
              if (connect_nbiot() == 1){
                  connect_state = live; 
              }            
           }
      }
*/
      if (vibration_alarm == 0) { // there was an alarm status change so we can update status over NB-IoT
            vibration_alarm = 1;
           // transmit_pkt(vib_alarm, false);
//TODO: kos add alarm attribute to tt and place at front of zqueue as an alarm.  MAYBE LET TB decide what is an alarm or not.
         }


      f_vibxyz = sqrt(f_vibx*f_vibx + f_viby*f_viby + f_vibz*f_vibz); // brute force magnitide without any machine optimisatiion 
      // push the vibration alarm data onto the queue    
      tvs.ts=now();

      /*
       * Fast Fourier Transform the vibration data.
       */
      for(int i=0; i < 128; i++){ // this is non ideal because vibration has 160 elements and FFT only allows 128 or 256
      vReal[i] = vibration[0][i] >> 14;
      vImag[i] = 0.0;
      }
      FFT.Windowing(vReal, 128, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
      FFT.Compute(vReal, vImag, 128, FFT_FORWARD); /* Compute FFT */ 
      FFT.ComplexToMagnitude(vReal, vImag, 128); /* Compute magnitudes */
      double freq = FFT.MajorPeak(vReal, 128, SAMPLERATE_VIB);
      
      sprintf(temp, "Peak Freq:  %dHz", (int)freq);
  //    debug_outln1(temp);
      
      tvs.vibx = f_vibx;
      tvs.viby = f_viby;
      tvs.vibz = f_vibz;
      tvs.vibxyz = f_vibxyz;  
      tvs.pk_freq = (float)freq; 
      tvs.tt = vib_alarm;
 //     vibration_q.push(tvs);
 
      if( xQueueSendToFront( x_iot_queue, &tvs, 0 ) != pdPASS )
         {
            DEBUG_STREAM.println(F("  Failed tvs message, due to queue full "));        
         }
         else
            DEBUG_STREAM.println(F("xQueue Snd Vib alrm"));
 
    // ***************
    /*    
      while (vibration_q.count() >= 2*VIB_PAYLOAD_SIZE ){ // Enough vib samples to assemble the packet and switch over to audio next time
             transmit_pkt(vibrate, false);
      }
      if (vibration_q.count() >= VIB_PAYLOAD_SIZE )
          {
          transmit_pkt(vibrate, true);  // set release indicator rai flag
          }
          */
    // *****************      
      Network_Keepconnect_Timer.reset();      // keep it connected after the last alarm update
      Network_Keepconnect_Timer.enabled = 1;
      vibx_max = 0; viby_max = 0; vibz_max = 0;

      }
 
      else {
         if (vibration_alarm == 1) { // there was an alarm status change so we can update status over NB-IoT
            vibration_alarm = 0;
            //transmit_pkt(vib_alarm, true);  //TODO:  kos 7/4/21   remove vib alarm flag as TB can do that
         }
      }

// ******************************************************************************************************************************************************  
      vib_block_countA++;
      vib_block_countB++;

      k=0;
      /*
      while (k<VIB_ADC_BLOCKSIZE){
      debug_out(adc_vib_buf[k]); debug_out(" "); debug_out(adc_vib_buf[k+1]);debug_out(" "); debug_outln(adc_vib_buf[k+2]);
      k+=3;  
      */
   }

   vibAlarm.event(); // update alarm timers
   
   if (vib_block_countA == vib_iterationsA){
   // fast vibration loop to eval alarms
   //TODO: kos  DO WE NEED THIS???, Yes, It  activates alarm relay.
    vibAlarm.post((int)f_vibxyz);
    vib_block_countA = 0;
   }
  
   if (vib_block_countB == vib_iterationsB){
   // slow vibration loop to eval ppv

   /*
      vibx_avg = (vibx_avg/vib_iterationsB) >> 14;
      viby_avg = (viby_avg/vib_iterationsB) >> 14;
      vibz_avg = (vibz_avg/vib_iterationsB) >> 14;

      f_vibx = (float)vibx_avg * ADC_CONV_RATIO;
      f_viby = (float)viby_avg * ADC_CONV_RATIO;
      f_vibz = (float)vibz_avg * ADC_CONV_RATIO;
   */
        
      vibx_max = vibx_max >> 14;
      viby_max = viby_max >> 14;
      vibz_max = vibz_max >> 14;
      f_vibx = (float)vibx_max * ADC_CONV_RATIO;
      f_viby = (float)viby_max * ADC_CONV_RATIO;
      f_vibz = (float)vibz_max * ADC_CONV_RATIO;

        
      f_vibxyz = sqrt(f_vibx*f_vibx + f_viby*f_viby + f_vibz*f_vibz); // brute force magnitide without any machine optimisatiion 

   if (f_vibxyz > VIB_AMPLITUDE_THRESHOLD) {
      tvs.ts=now();


      if (f_vibxyz > 2) {  // menaningful to do vibration when it exceeds 2mm/s

        /*
         * Fast Fourier Transform the vibration data.
         */
        for(int i=0; i < 128; i++){ // this is non ideal because vibration has 160 elements and FFT only allows 128 or 256
          vReal[i] = vibration[0][i] >> 14;
          vImag[i] = 0.0;
        }
        FFT.Windowing(vReal, 128, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
        FFT.Compute(vReal, vImag, 128, FFT_FORWARD); /* Compute FFT */ 
        FFT.ComplexToMagnitude(vReal, vImag, 128); /* Compute magnitudes */
        double freq = FFT.MajorPeak(vReal, 128, SAMPLERATE_VIB);
        float_tostr(freq, 1, temp1);
        float_tostr(f_vibxyz, 2, temp2);
        sprintf(temp, "Peak Freq:  %sHz Vibxyz:  %smm/s", temp1,temp2 );
       // debug_outln1(temp);
        //debug_out("Peak Freq:  ");debug_out(String(freq,2));debug_out("  Vibxyz:  ");debug_outln(String(f_vibxyz,2));
        tvs.pk_freq = (float)freq;
      }
      else {
        tvs.pk_freq = 0; // no frequency analysis when under alarm threshold (for the moment)
      }
      tvs.tt = vibrate;
      tvs.vibx = f_vibx;
      tvs.viby = f_viby;
      tvs.vibz = f_vibz;
      tvs.vibxyz = f_vibxyz;
      //vibration_q.push(tvs);
      
      if( xQueueSend( x_iot_queue, &tvs, 0 ) != pdPASS )
         {
            DEBUG_STREAM.println(F("  Failed to post the tvs message, probably due to queue full "));        
         }
         else
            DEBUG_STREAM.println(F("xQueue Snd Vib"));
       
    }
      
      //debug_out("f_vibxyz = ");debug_outln(  f_vibxyz);
        
        
     // debug_out( "f_vibx=");debug_out(f_vibx); debug_out( ", f_viby=");debug_out(f_viby);
     // debug_out( ", f_vibz=");debug_outln(f_vibz);
 
      vib_block_countB = 0;
        
      vibx_avg = 0;viby_avg=0;vibz_avg=0;
      vibx_max = 0; viby_max = 0; vibz_max = 0;
        
    }
   
   }
   }
  DEBUG_STREAM.println(F("Thread Vib: Deleting"));
  vTaskDelete( NULL );
 }

/* **********************************************************************
 * Power monitoring and control Task 
 * Reports battery percentage as telemetry so power consumption trend over time is captured
 * **********************************************************************
 */ 
static void prvTaskPwr( void *pvParameters )
{
  unsigned char charger_stat[4];
 Egg_Timer Pwr_Upload_Timer;
 byte batt_percent;
 word save_batt_params_flag; // derived from bit 6 of charging cycles
 byte prev_batt_percent;
 byte chrg_status_local;
 byte prev_chrg_status_local;
 float batt_voltage;
 float batt_current;
 float avg_batt_current;
  unsigned char chgin_ilim;
 byte increase_current_phase;
 unsigned char vchgin_reg;
 typedef enum {enter_vloop, vloop_plus, vloop_minus, enter_iloop, iloop_plus, iloop_minus } ifsm_t;
 ifsm_t ifsm;
 unsigned int aicl_ok;
 unsigned int chginlim_ok;
 unsigned int  uvlo;
 const byte MAX_CHG_CURRENT = 0x3D;//1500mA // 0x31 1200mA  0x29; // 1000mA     0x21; // 800mA
 const byte MAX_VCHGIN = 0x17; // 14.55V
 int increment_loop, decrement_loop, bug_fix_loop;
 int loop_count;
 static led_blink_mode_t led_state;
 generic_struct_t tps;
 tps.tt = power;
 batt_voltage = get_batt_voltage();
 DEBUG_STREAM.println(batt_voltage);
 while( batt_voltage <=MIN_BATT_VOLTAGE){ // wait until battery gets enough charge for the init_charger not to brown out the supply
  myDelayMs(10000);
  batt_voltage = get_batt_voltage();
  DEBUG_STREAM.print("Batt Volt=");DEBUG_STREAM.println(batt_voltage);
  DEBUG_STREAM.print("Charger I=");DEBUG_STREAM.println(get_batt_current());
 }
 DEBUG_STREAM.println(F("Initialising charger"));
 init_charger();
 DEBUG_STREAM.println(F("Charger Initialised"));
 myDelayMs(50);
 DEBUG_STREAM.print(".");
 myDelayMs(8450);
 DEBUG_STREAM.print(".");
 myDelayMs(7500);
 DEBUG_STREAM.print(".");
 myDelayMs(7500);
 DEBUG_STREAM.println(F(" Power and battery monitoring task started"));
i2c_read(MAX77962_DEV_ADDR, 0x13, charger_stat, 3);
DEBUG_STREAM.print(F("Lithium Charger 0x13 = ")); DEBUG_STREAM.println(charger_stat[0]);
DEBUG_STREAM.print(F("Lithium Charger 0x14 = ")); DEBUG_STREAM.println(charger_stat[1]);
DEBUG_STREAM.print(F("Lithium Charger 0x15 = ")); DEBUG_STREAM.println(charger_stat[2]);

init_batt_mon();
DEBUG_STREAM.print(F("Battery %= "));DEBUG_STREAM.println( get_batt_percent());
DEBUG_STREAM.print(F("Battery time to empty (Days) "));DEBUG_STREAM.println((float)get_batt_timetoempty()/15360); // can only measure out to 4.27 days 
DEBUG_STREAM.print(F("Charge cycles % = "));DEBUG_STREAM.println(get_batt_mon_cycles());
if (is_charging() == 1)
  DEBUG_STREAM.println(F("Battery Charging"));
else
  DEBUG_STREAM.println(F("Battery NOT Charging"));
charger_status = get_charger_stat();
parse_charger_stat(charger_status, charger_status_str, CHARGER_STATUS_STRLEN);
DEBUG_STREAM.println(charger_status_str);



 led_state.bt = charger;
 myDelayMs(13000); // delay here so that the time is successfully retreived
 batt_percent =get_batt_percent();
 prev_batt_percent = batt_percent;
 chrg_status_local = get_charger_stat();
 prev_chrg_status_local = chrg_status_local;
 batt_voltage = get_batt_voltage();
 batt_current = get_batt_current();
 batt_current = get_batt_current();
 tps.param1 = (float)batt_percent;
 tps.param2 = (float)chrg_status_local;
 tps.param3 = batt_voltage;
 tps.param4 = batt_current;
 tps.time_val=now();
 
 Pwr_Upload_Timer.duration = 600; // push battery status up every 10 mins
 Pwr_Upload_Timer.enabled = 1;
 Pwr_Upload_Timer.reset();

 chgin_ilim = MAX_CHG_CURRENT/2; // 50mA
 //vchgin_reg = 0x1a; // 16.35V
 //vchgin_reg = 0x15; // 13.35V
// vchgin_reg = 0x0f; // 10.15V
//vchgin_reg = 0x0b; // 8.05V
 vchgin_reg = 0x05; // 4.9VV
 if( xQueueSend( x_iot_queue, &tps, 0 ) != pdPASS )
            DEBUG_STREAM.println(F("  Failed to post the tps message, probably due to queue full "));
         else{
            DEBUG_STREAM.print(tps.time_val);
            DEBUG_STREAM.print(F("  xQueue Snd pwr batt% "));
            DEBUG_STREAM.print(tps.param1);
            DEBUG_STREAM.print(F("  charger status="));
            DEBUG_STREAM.println(tps.param2);
         }
		 
	 
	set_vchgin_reg(vchgin_reg);
 set_chgin_ilim(chgin_ilim); 
  ifsm = enter_vloop;

  increment_loop = 0;
  decrement_loop = 0;
  bug_fix_loop = 0;
  loop_count = 1;
  save_batt_params_flag = get_batt_mon_cycles()& (word)0x0040;
  increase_current_phase = 1;
 while(1)
   {
    batt_percent =get_batt_percent();
    batt_voltage = get_batt_voltage();
    batt_current = get_batt_current();
    chrg_status_local = get_charger_stat();
    //if ((batt_percent != prev_batt_percent) || ( chrg_status_local != prev_chrg_status_local ) ||(Pwr_Upload_Timer.check()==1) ) {
    if ((batt_percent != prev_batt_percent)  ||(Pwr_Upload_Timer.check()==1) ) {
      avg_batt_current = avg_batt_current/(float)loop_count;
      Pwr_Upload_Timer.reset();
	    prev_batt_percent = batt_percent;
      prev_chrg_status_local = chrg_status_local;
      tps.param1 = (float)batt_percent;
      tps.param2 = (float)chrg_status_local;  // telemetry timestampted data of charger status
      tps.param3 = batt_voltage;
      tps.param4 = avg_batt_current;
      tps.time_val=now();
      avg_batt_current = 0;
      loop_count = 1;
      
      if ((get_batt_mon_cycles() & (word)0x0040) != save_batt_params_flag){
         save_battmon_params();
         save_batt_params_flag = get_batt_mon_cycles()& (word)0x0040;
         DEBUG_STREAM.println(F("Saving battery params to EEPROM\r\n"));
      }
      if( xQueueSend( x_iot_queue, &tps, 0 ) != pdPASS )
          DEBUG_STREAM.println(F("  Failed to post the tps message, probably due to queue full "));
      else
          DEBUG_STREAM.println(F("xQueue Snd pwr"));
    }
    else{
       loop_count++;
       avg_batt_current += batt_current;
    }
   charger_status = get_charger_stat();  /* charger_status is a global variable and updates the TB attribute*/

/* Inner loop here to actively control the current limit
 *  on the charger.  Start at 50mA and increase the limit until
 *  under voltage. This is to harvest maximum current before the
 *  5V external regulator drops off due to insufficient current
 *  from the solar panel. We use the +5V as a voltage down converter
 *  current up converter because low light solar panel cannot deliver 
 *  the minimum 50mA for the MAX77962 at 17V but with down conversion to 5V, it can.
 */
get_chg_loopvars(&aicl_ok, &chginlim_ok, &uvlo); // update the flags
//DEBUG_STREAM.print(F("aicl_ok="));DEBUG_STREAM.print(aicl_ok);DEBUG_STREAM.print(F("   chgin_ilim="));DEBUG_STREAM.print(chginlim_ok);DEBUG_STREAM.print(F("   increment_loop="));DEBUG_STREAM.println(increment_loop);

if ((aicl_ok ==1) && (chginlim_ok==1) && (chgin_ilim >2 ) && decrement_loop >= 1 ){
    decrement_loop = 0;
    chgin_ilim--;
    increase_current_phase = 0; // we are decreasing Ilim so only increment on the 30 sec loop.
    DEBUG_STREAM.print(F("Decr-,chgin_ilim=  "));DEBUG_STREAM.println(chgin_ilim);
    if (chgin_ilim==2){
      led_state.val = 0; // val=0, not charging,  1, partially powering system, 2, solar powering system and charging
      xQueueSend( x_led_queue, &led_state, 0 );
    }
    else{
       if (chgin_ilim < 6){
         led_state.val = 1; // val=0, not charging,  1, partially powering system, 2, solar powering system and charging
         xQueueSend( x_led_queue, &led_state, 0 );
       }
       else{
        led_state.val = 2; // val=0, not charging,  1, partially powering system, 2, solar powering system and charging
         xQueueSend( x_led_queue, &led_state, 0 );
       }      
    }
}

if ((aicl_ok ==1) && (chginlim_ok==0) && (chgin_ilim <MAX_CHG_CURRENT ) && (increment_loop >= 32 || increase_current_phase == 1)){
    increment_loop = 0;
    increase_current_phase = 1;
    chgin_ilim++;
    DEBUG_STREAM.print(F("Incr+,chgin_ilim=  "));DEBUG_STREAM.println(chgin_ilim);
    if (chgin_ilim< 6){    
      led_state.val = 1; // val=0, not charging,  1, partially powering system, 2, solar powering system and charging
      xQueueSend( x_led_queue, &led_state, 0 );
    }
    else{
      led_state.val = 2; // val=0, not charging,  1, partially powering system, 2, solar powering system and charging'b'
      xQueueSend( x_led_queue, &led_state, 0 );
    }
}

if ((aicl_ok == 0) && (chgin_ilim >2)){
//DEBUG_STREAM.println(F("AICL_OK is zero so decrementing chg_ilim"));
//DEBUG_STREAM.print(F("aicl_ok= "));DEBUG_STREAM.println(aicl_ok);
//DEBUG_STREAM.print(F("chginlim_ok= "));DEBUG_STREAM.println(chginlim_ok);
//DEBUG_STREAM.print(F("chgin_ilim= "));DEBUG_STREAM.println(chgin_ilim);
//chgin_ilim--;
}

/*
 * Every 4 mins, decrement ilim because sometime the charger stops incrementing and decrementing
 * when full solar power is present and the charge current changes from 800mA to 160mA.  Forcing a
 * decrement seems to jolt it back into operation. THis is because the charger enters trickle charge mode
 */
if (bug_fix_loop > 240 && chgin_ilim > 25){
  bug_fix_loop = 0;
  
  DEBUG_STREAM.println(F("Forced ilim decrement by 10"));
  DEBUG_STREAM.print(F("aicl_ok= "));DEBUG_STREAM.println(aicl_ok);
  DEBUG_STREAM.print(F("chginlim_ok= "));DEBUG_STREAM.println(chginlim_ok);
  DEBUG_STREAM.print(F("uvlo= "));DEBUG_STREAM.println(uvlo);
  // chgin_ilim-=10;  // disable this because it does not do much
}


bug_fix_loop++;
decrement_loop++;
increment_loop++;
set_chgin_ilim(chgin_ilim); 
// **************************Need a way to converge quickly on optimum ilim and then stay there iwth small adjustments periodically
/*
switch (ifsm) {
  case enter_vloop:
      vchgin_reg = 0x05; // 4.9V
      set_vchgin_reg(vchgin_reg);
      set_chgin_ilim(0x00); // set to minimum before we start raising voltage
      ifsm = vloop_plus;
      break;

  case vloop_plus: // increase vchgin_reg until...
      
      if (vchgin_reg >=0x16 ||   ){ // its too high so reduce voltage and goto iloop
        ifsm = vloop_minus;
        break;
      }
      
      vchgin_reg++;
      set_vchgin_reg(vchgin_reg);
      
      

      break;

   case vloop_minus: // decrease vchgin_reg until...
      if (vchgin_reg <=0x5 || uvlo == 0 ||   ){
        ifsm = iloop;
        break;
      }
      vchgin_reg--;
      set_vchgin_reg(vchgin_reg);
      break;
   
   case iloop:
   if ((aicl_ok == 1) && (chginlim_ok == 0) && (uvlo ==0) ){ // adaptive current is off and inlim not reached so we can raise inlim.
     if (chgin_ilim <= MAX_CHG_CURRENT)
        chgin_ilim++;
     set_chgin_ilim();
     break;
   }
   if ((aicl_ok == 0) && (chginlim_ok == 1) && (uvlo == 0)){ // adaptive current is on and inlim is reached so raise ilim
     if (chgin_ilim <= MAX_CHG_CURRENT)
         chgin_ilim++;
     set_chgin_ilim();
     break;
     }

   if ((aicl_ok == 1) && (chginlim_ok == 1) && (uvlo == 0)){ 
    
   }

     
   break;

   
      
}

*/
   
   myDelayMs(1000); // 4 secs  
   }
  DEBUG_STREAM.println(F("Thread Pwr: Deleting"));
  vTaskDelete( NULL );
}


/* **********************************************************************
 * Attribute Task 
 * **********************************************************************
 */
static void prvTaskAttrib( void *pvParameters )
 {
  attr_stat_struct_t tas; // variable to hold start time, gps data, battery, reset_count and signal strength
  tas.tt = status_attr;
  while(1)
   {
  
   }
  DEBUG_STREAM.println(F("Thread Attr: Deleting"));
  vTaskDelete( NULL );
 }


 
 /* **********************************************************************
 * CLI Task 
 * **********************************************************************
 */
static void prvTaskCLI( void *pvParameters )
 {
  while(1)
   {
  
   }
  DEBUG_STREAM.println(F("Thread CLI: Deleting"));
  vTaskDelete( NULL );
 }

 void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
 {
  DEBUG_STREAM.print(F("*****STACK OVERFLOW :  Task Name:-"));DEBUG_STREAM.println("pcTaskName");
 }


/*
 * LED indicator task.  Multiple blink modes are supported to indicate the state of the system
 */
 static void prvTaskLed( void *pvParameters )
{
  unsigned int counter=0;
  byte charger_val=0;
  byte network_val=0;
  byte blink_pattern_select = 3;
  byte _byte;
  /*
   * Blink patterns for all combinations.
   * 0 - connected with charging val 0  1 short blink in 32
   * 1 - connected with charging val 1  2 successive short blinks in 32
   * 2 - connected with charging val 2  3 successive short blinks in 32
   * 3 - connecting // slow blink 50% on off.
   * 4 - disconnected  // fast intermittent short blinks
   * 5 - transmitting data // random rapid blinking
   */
  unsigned int blink_pattern[6] ={0x01000000, 
                          0x0a000000,
                          0x2a000000,
                          0xff00ff00,
                          0x88888888,
                          0x5a5a55aa}; 
  led_blink_mode_t param;
  /*
   * period of blink cycle in units of 100ms
   * hi_mark is proportion of blink cycle that LED is on in units of 200ms
   */
  DEBUG_STREAM.println(F("LED indicator task started"));
  while(1)
  {
  if( xQueueReceive( x_led_queue, &param, 0 ) == pdPASS ){
     if (param.bt == network)
       network_val = param.val;
     if (param.bt == charger)
       charger_val = param.val;
    

    switch (network_val){
      case 0: // disconnected
            blink_pattern_select = 4;
            break;
      case 1: // connecting
         blink_pattern_select = 3; // network connecting so slow on off blink
         break;
      case 2: // online
             switch (charger_val){
                case 0:
                  blink_pattern_select = 0; // not charging - one short blink every 3 sec
                  break;
                case 1:
                  blink_pattern_select = 1; // not charging but system is partly powered by solar -  double blink every 3 secs
                  break;
                case 2:
                  blink_pattern_select = 2; // charging and system is spowered by solar - triple blink every 3 sec
                  break;
            }
         
         break;
      case 3: // transmitting data
         blink_pattern_select = 5;
         break;
   }
   //DEBUG_STREAM.print("blink_pattern_select =");DEBUG_STREAM.println(blink_pattern_select);
}
   _byte = (byte)((blink_pattern[blink_pattern_select] >> counter) & (unsigned int)0x01);
   //DEBUG_STREAM.print(_byte);
    if (counter >= 31)
          counter = 0;
    else
          counter ++;
    digitalWrite(blueLed, _byte);
    
      
      myDelayMs(100); // loops 10 times per second.
   }
   
  DEBUG_STREAM.println(F("Thread LED: Deleting"));
  vTaskDelete( NULL );
 }
