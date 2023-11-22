
/* i2c peripheral functions
*  From Ver 4.00 onwards
* Author: KOS  Majentix Ltd
* Assigned to Glenside Environmental Ltd
*/

#include <Arduino.h>
#include "i2c_periphs.h"
#include <Wire.h>
#include <FreeRTOS_SAMD51.h>

/* Function Prototype */
extern void myDelayUs(int us);
extern void myDelayMs(int ms);
//extern void debug_out1(char * msg);
//extern void debug_outln1(char * msg);

/*  Maxim MAX77962 Lithium Charger sub addresses  */
 
#define CHG_CNFG_01_SADDR 0x17
#define CHG_CNFG_02_SADDR 0x18
#define CHG_CNFG_03_SADDR 0x19
#define CHG_CNFG_04_SADDR 0x1A
#define CHG_CNFG_05_SADDR 0x1B
#define CHG_CNFG_06_SADDR 0x1C
#define CHG_CNFG_07_SADDR 0x1D
#define CHG_CNFG_08_SADDR 0x1E
#define CHG_CNFG_09_SADDR 0x1F
#define CHG_CNFG_10_SADDR 0x20      
    

void init_i2c()
{
	Wire.begin(); // enable i2c
}


int i2c_write(byte dev_addr, byte sub_addr, unsigned char * pdata, byte len) // len must be <  32 bytes
{ int res;
  Wire.begin(); // enable i2c
	Wire.beginTransmission(dev_addr);
     Wire.write(byte(sub_addr)); // sub address
	 for (int i=0; i<len;i++){
		 Wire.write(*pdata); 
	 }
      
	  res =Wire.endTransmission();// 0: success, 1: data too long 2: No ACK for address 3: No Ack for data 4: Other error
    Wire.end();
    return res;
     
}

int i2c_read(byte dev_addr, byte sub_addr, unsigned char * pdata, byte len)
{
	int x;
   const int timeout = 100;
   //Serial.print(".");
   Wire.begin(); // enable i2c
   //Serial.print(".");
   Wire.beginTransmission(dev_addr);
   //Serial.print(".");
   Wire.write(byte(sub_addr)); 
   //Serial.print(".");
   x=Wire.endTransmission();
   if (x != 0){
        Wire.end();
        return x;
   }
   Wire.requestFrom(dev_addr,len);
   //Serial.print(".");  
   //myDelayMs(1);  //TODO: may need to calibrate this delay
   delay(len);
   int i2c_elapsed = 0;
   while (Wire.available() < len && i2c_elapsed <timeout){
     //myDelayMs(1); //TODO: may need to calibrate this delay
     delay(1);
     i2c_elapsed++;
     }
	if (timeout == i2c_elapsed){ 
	   Wire.end();
	   return -1; // timeout
	}
    for (int i=0;i<len;i++)
      *pdata++ = Wire.read();
    Wire.end();
   return 0; // success
	 
}



void i2c_write16( byte addr, byte sub_addr, word data)
{
  static unsigned char *tempptr;
  byte _byte[2];
  _byte[0] = (byte) (data & 0x00ff);
  _byte[1] = (byte) ((data >> 8) & 0x00ff);
  tempptr = _byte;
  i2c_write( addr, sub_addr, tempptr, 2);
}


word i2c_read16( byte addr, byte sub_addr)
{
  byte _byte[2];
  word result;
  i2c_read( addr, sub_addr, _byte, 2);
  result = _byte[0] | (_byte[1] << 8);
  return result;
}

void i2c_write_verify16( byte addr, byte sub_addr, word data)
{
  int attempt=0;
  word register_value_read;
  do
  {
    i2c_write16( addr, sub_addr, data);
    delay (1000);
    //myDelayMs(100);
    register_value_read = i2c_read16(  addr,  sub_addr);
  } while (register_value_read != data && attempt++<5);
}



void init_batt_mon()
{
  
  word status;
  word por;
  word HibCFG;
  int attempt =0;
  word devName;  // chip id


    devName = i2c_read16( (byte)MAX17261_DEV_ADDR, (byte)0x21);
    if (devName == 0x4033)
        Serial.println( "Battery Monitor IC Detected");
    else
      Serial.println( "Battery Monitor Not Detected");

  por = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x00) & 0x0002;
      Serial.println( "Battery monitor initialising .1.");
  if (2 == por)
     {
    Serial.println( "Batt mon POR, resetting");
    while (   1== (i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x3d) & 1)  && attempt++ < 300 )
        {
           delay(1000);
        }
    Serial.print( "0x3d=");Serial.print(i2c_read16( (byte)MAX17261_DEV_ADDR, (byte)0x3d));
    Serial.print(", attempt=" ); Serial.println( attempt );
    if (attempt > 299) Serial.println( "** ERR Batt mon too many attempts resetting");
    HibCFG = i2c_read16( (byte)MAX17261_DEV_ADDR, (byte)0xBA); // Store original HibCFG
    i2c_write16( (byte)MAX17261_DEV_ADDR, (byte)0x60, (word)90 ); // Exit Hibernate Mode Step 1
    i2c_write16( (byte)MAX17261_DEV_ADDR, (byte)0xBA, (word)0 ); // Exit Hibernate Mode Step 2
    i2c_write16( (byte)MAX17261_DEV_ADDR, (byte)0x60, (word)0 ); // Exit Hibernate Mode Step 3
    i2c_write16( (byte)MAX17261_DEV_ADDR, (byte)0x18, (word)BATTERY_CAPACITY );
    i2c_write16( (byte)MAX17261_DEV_ADDR, (byte)0x1e, (word)BATTERY_IchargeTerm );
    i2c_write16( (byte)MAX17261_DEV_ADDR, (byte)0x3a, (word)BATTERY_VEmpty );
    i2c_write16( (byte)MAX17261_DEV_ADDR, (byte)0xdb, (word)0x8000 ); // set V charge for up to 4.2V per cell

    // Poll ModelCFG.Refresh(highest bit)
    attempt = 0;
    while (   0x8000== (i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0xdb) & 0x8000)  &&  (attempt++ < 300) )
            {
                   delay (1000);
                   //myDelayMs(100);
            }
    if (attempt > 299) Serial.println( "** ERR Batt mon too many attempts committing settings");

    restore_battmon_params(); // Try to restore previously saved values.

    i2c_write16((byte)MAX17261_DEV_ADDR, (byte)0xba, HibCFG ); // Restore Original HibCFG value
     }

  // Clear the PoR bit
  status = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x00);

  i2c_write_verify16( (byte)MAX17261_DEV_ADDR,(byte)0x00, (status & 0xfffd)  );
  Serial.println( "Batt monitor initialised");
}

byte get_batt_percent()
{
  word result;
  result = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x06);
  result = result >>8;
  return (byte)result;
}

float get_batt_voltage()
{
  word result;
  float fresult;
  result = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x19);
  fresult = (2*(float)result *(float)78.125)/(float)1000000 ; // result in volts for 2 cells
  return fresult;
}

/* works on a 5 second average so wont catch transmit data surges */
/* will capture charging current quite accurately */
float get_batt_current() // 
{
  word uresult;
  short sresult;
  float fresult=4000;
  int count =0;
  /* THis checks that there is no error in reading current as seen in earlier versions */
  while ((count < 8) && (fresult > 3000)){
    count++;
    uresult = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x0b);
    sresult = (short)uresult;
    fresult = ((float)sresult *(float)312.5)/(float)1000 ; // result in mA
  }
  return fresult;
}

typedef struct {word RCOMP0; word TempCo; word FullCapRep; word Cycles; word FullCapNom;} batt_mon_params_t;

/* 
 *  Reads the battery monitor paramaters from the MAX17261 battery gauge
 *  and saves them to EEPROM. THis allows learned characteristics of the battery 
 *  such as capacity, discharge cycles and ageing to be backed up in case of a
 *  complete power loss and power on reset.
 */
void save_battmon_params(){
  unsigned char batt_eeprom_param_flag = 0x01;
  batt_mon_params_t batt_mon_params;
  batt_mon_params.RCOMP0 = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x38);
  batt_mon_params.TempCo = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x39);
  batt_mon_params.FullCapRep = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x10);
  batt_mon_params.Cycles = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x17);
  batt_mon_params.FullCapNom = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x23);
  i2c_write16( (byte)EEPROM_DEV_ADDR, (byte)0x08, batt_mon_params.RCOMP0);
  i2c_write16( (byte)EEPROM_DEV_ADDR, (byte)0x0a, batt_mon_params.TempCo);
  i2c_write16( (byte)EEPROM_DEV_ADDR, (byte)0x0c, batt_mon_params.FullCapRep);
  i2c_write16( (byte)EEPROM_DEV_ADDR, (byte)0x0e, batt_mon_params.Cycles);
  i2c_write16( (byte)EEPROM_DEV_ADDR, (byte)0x10, batt_mon_params.FullCapNom);
  i2c_write( (byte)EEPROM_DEV_ADDR, (byte)0x07, &batt_eeprom_param_flag,1); // marks the eeprom as having saved valid data
}

void wipe_batt_mon_params(){
  unsigned char batt_eeprom_param_flag = 0xFF;
  i2c_write( (byte)EEPROM_DEV_ADDR, (byte)0x07, &batt_eeprom_param_flag,1);
}

word get_batt_mon_cycles(){
  word cycles;
  cycles = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x17);
  return cycles;
}
/* 
 *  Retrieves the battery monitor paramaters from EEPROM and writes them to the 
 *  MAX17261 battery gauge in the event of a power loss and POR event.
 */
int restore_battmon_params(){
  word dQacc;
  byte batt_eeprom_param_flag;
  // First check if the EEPROM has valid battery monitor parameters by checking its flag
  i2c_read((byte)EEPROM_DEV_ADDR, (byte)0x07, &batt_eeprom_param_flag,1);
  if (batt_eeprom_param_flag == 0x01){
  batt_mon_params_t batt_mon_params;
  batt_mon_params.RCOMP0 = i2c_read16((byte)EEPROM_DEV_ADDR, (byte)0x08);
  batt_mon_params.TempCo = i2c_read16((byte)EEPROM_DEV_ADDR, (byte)0x0a);
  batt_mon_params.FullCapRep = i2c_read16((byte)EEPROM_DEV_ADDR, (byte)0x0c);
  batt_mon_params.Cycles = i2c_read16((byte)EEPROM_DEV_ADDR, (byte)0x0e);
  batt_mon_params.FullCapNom = i2c_read16((byte)EEPROM_DEV_ADDR, (byte)0x10);
  i2c_write_verify16((byte)MAX17261_DEV_ADDR,(byte)0x38, batt_mon_params.RCOMP0);
  i2c_write_verify16((byte)MAX17261_DEV_ADDR,(byte)0x39, batt_mon_params.TempCo);
  i2c_write_verify16((byte)MAX17261_DEV_ADDR,(byte)0x10, batt_mon_params.FullCapRep);
  dQacc = batt_mon_params.FullCapNom/2;
  i2c_write_verify16((byte)MAX17261_DEV_ADDR,(byte)0x46, (word)0x0c80); //Write and Verify dPacc
  i2c_write_verify16((byte)MAX17261_DEV_ADDR,(byte)0x45, dQacc);
  i2c_write_verify16((byte)MAX17261_DEV_ADDR,(byte)0x17, batt_mon_params.Cycles);
  
  return 0;
  }else{
      Serial.println(F("Battery EEPROM params not present"));
      return -1;
  }
}


word get_batt_timetoempty()
{
  word result; 
  result = i2c_read16((byte)MAX17261_DEV_ADDR, (byte)0x11);
  return result; // units of 5.625 seconds
}


void init_charger()
{
  unsigned char charger_stat[3];
  int i;
  byte temp;
  byte *tempptr;
  struct iic_register {
    byte addr;
    byte value;
  };
  static  struct iic_register max77962_init_regs[] = {
    {0x1c, 0x0C}, // unlock charger settings registers
    {0x17, 0x97}, // set charge timer to 10 hrs
    {0x18, 0x78}, // set fast charging current limit to 800mA
    {0x1a, 0x32},  // Charge voltage  Ansmann battery is 8.4V+-1%
    {0x1b, 0x11}, // BATT to SYS Overcurrent threshold 3A, this is the minimum.  Trickle charge 250mA
    {0x1d, 0x0a}, // REGTEMP bits (6:3) set to 90 degrees and JEITA disabled
//    {0x1e, 0x22}, // bits 6:0 set charge in Ilimit (e CHGIN_ILIM), 825mA
//    {0x1e, 0x11}, // bits 6:0 set charge in Ilimit (e CHGIN_ILIM), 360mA
    {0x1e, 0x00}, // bits 6:0 set charge in Ilimit (e CHGIN_ILIM), 50mA  Adjust this for different light conditions
    {0x19, 0x10}, //sys tracking disabled.
//    {0x20, 0x18}, // VCHGIN_REG bits 5:1 (4.025 to 19V) set to 8.575V
    {0x20, 0x05}, // VCHGIN_REG bits 5:1 (4.025 to 19V) set to 4.9V (0x05),   4.025V
    {0x1c, 0x8C}, // write command to transfer fast charge current limit in 0x18
    {0x1c, 0x80}, // turn protection bits back on
    {0x16, 0x85},  //Charger activated
    {0xff, 0xff}, /*over */
  };
  i=0;
while(1)
        {
          if(max77962_init_regs[i].addr==0xff)
            break;
          temp = max77962_init_regs[i].addr;
          tempptr = &max77962_init_regs[i].value;
          i2c_write( MAX77962_DEV_ADDR, temp, tempptr, 1);
          Serial.print(F("Write to charger Addr: "));Serial.print(temp);Serial.print("Val: ");Serial.println(max77962_init_regs[i].value);
          i++;
        }


  }
  
/*
 * Sets the maximum limit that current will draw in current adaptive loop moide.
 * 0x03 to 0x7f corresponding to 50mA to 3150mA in steps of 25mA
 * 0x00, 0x0x1, 0x02, 0x03 are all setr to the floor of 50mA. An energy source unable to deliver
 * 50mA at vchgn_reg will shut off the charger.
 */
void set_chgin_ilim(unsigned char val) 
{
  byte prot_bits = 0x0c;
	unsigned char temp;
	temp = val & 0x7f; // mask out msb
   
  i2c_write( MAX77962_DEV_ADDR, 0x1c, &prot_bits, 1); // turn off protection bits
	i2c_write( MAX77962_DEV_ADDR, 0x1e, &temp, 1);
  prot_bits = 0x80;
  i2c_write( MAX77962_DEV_ADDR, 0x1c, &prot_bits, 1); // turn on protection bits
  
}

/*
 * Sets the charger input regulation voltage. The charger will draw the amount of current until
 * the input charger voltage drops to vchgin_reg and then draws no more.
 * 0x00 = 4.025V up to 0x1f = 19.050 in non linear steps.
 * 0x13 = 13.35V which is the mpt of solar panel 
 * 0x05 = 4.9V if using a buck converter to harvest low light and generate at least 50mA
 * which is the ilim minimum.
 */
void set_vchgin_reg(unsigned char val)
{
	unsigned char temp;
	temp = (val & 0x1f) << 1; // mask in bits 4:0 and shift left
	i2c_write( MAX77962_DEV_ADDR, 0x20, &temp, 1);
}
	

/*
 * Updates the aicl_ok and chginlim_ok flags from the charger IC
 * Used to control the power harvesting loop
 */
void get_chg_loopvars( unsigned int * aicl_ok, unsigned int * chginlim_ok, unsigned int * uvlo)
{
 
  byte result;
  i2c_read(MAX77962_DEV_ADDR, 0x12, &result,1);
  *aicl_ok = (unsigned int)((result >> 7) & 0x01);
  *chginlim_ok = (unsigned int)((result >> 2) & 0x01);
  //Serial.print("Reg 0x12 = ");Serial.println(result);
  i2c_read(MAX77962_DEV_ADDR, 0x13, &result,1);
  if( (  (result >> 5) & 0x03) == 0)
  *uvlo = 1; // under voltage lockout could be caused by too much current draw on charger input
  else
  *uvlo = 0;
}

int is_charging()
{
  byte  result;
  i2c_read(MAX77962_DEV_ADDR, 0x14, &result,1);
  if ((result & 0x0f) < 4) // yes its charging
  return 1;
  else
  return 0;
}

charger_stat get_charger_stat()
{
  byte chrg_details[3];
  byte val;
  byte charge_i;
  i2c_read(MAX77962_DEV_ADDR, 0x12, chrg_details,3);
//  Serial.print("MAX77962_DEV_ADDR, 0x12 = ");Serial.print(chrg_details[0]); Serial.print("  0x13= "); Serial.print(chrg_details[1]);Serial.print("  0x14= ");Serial.println(chrg_details[2]);
  if ((chrg_details[2] & 0x0f) < 4) // we are charging
      return charging;

/* battery details  */
val = (chrg_details[2] >> 4) & 0x07; 
 switch (val){

  case 0x00  :
    Serial.println(F("Battery Removal detected on THM pin"));
    return batt_fault;

    case 0x01  :
    Serial.println(F("Vbatt <  Vprecharge"));
    break;
  case  0x02  :
    Serial.println(F("Battery Fault"));
    return batt_fault;
  case 0x03   :
  /* The battery is okay and its voltage is greater than the minimum system voltage*/
  //Serial.println(F("Battery OK"));
  break;

  case 0x04   :
     Serial.println(F("Battery ok but voltage is low Vprecharge < Vbatt < Vsysmin-500mv"));
     break;
  case  0x05  :
     Serial.println(F("Battery Over Voltage > CHG_CV_PRM"));
    return batt_OV;
  case  0x06  :
     Serial.println(F("Battery Over Current"));
    return batt_fault;

  case  0x07  :
     Serial.println(F("Battery level unavailable from charger"));
     break;
 }

/* charger voltage input details */
  val = (chrg_details[1] >> 5) & 0x03;
switch (val)  {
  case  0x00  :
  Serial.println(F("Charge UVLO, insufficient input voltage"));
  return charge_UVLO;  // input voltage insufficiently low to charge
  case  0x01  :// reserved
  break;
  case  0x02  :
  return charge_OVLO; // input voltage too high to charge
  case  0x03  :
  break;  
}

if (chrg_details[1] & 0x0f == 0x0a)
   return charge_thermal_shdn; // thermal shutdown
    
  if  ((chrg_details[2] & 0x0f) == 4){
      return charge_done; 
  }

  else {
   /* if it gets this far, then status is unknown*/
   return invalid;
  }
}

void get_charger_mode()
{
  byte chrg_details;
  byte val;
  byte charge_i;
  i2c_read(MAX77962_DEV_ADDR, 0x14, &chrg_details,1);
  chrg_details = chrg_details & 0x0f; // mask out irrelevant bits.
  switch (chrg_details) {
    case 0x00 :
    Serial.println(F("Precharge or trickle charge mode"));
    break;
    case 0x01 :
    Serial.println(F("Fast charge constant current mode"));
    break;
    case 0x02 :
    Serial.println(F("Fast charge vonstant voltage mode"));
    break;
    case 0x03 :
    Serial.println(F("Top off mode"));
    break;
    case 0x04:
    Serial.println(F("Charger is done mode"));
    break;
    case 0x05 :
    Serial.println(F("Charger off because at least one pin has valid resistance INLIM, ITO, ISET VSET"));
    break;
    case 0x06 :
    Serial.println(F(" Timer fault mode"));
    break;
    case 0x07 :
    Serial.println(F("Charger suspended bacause Qbat disabled"));
    break;
    case 0x08 :
    Serial.println(F("Charger is off because charger input invalid or disabled"));
    break;
    case 0x0a :
    Serial.println(F("Charger is off because Junction temperature"));
    break;
    case 0x0b :
    Serial.println(F("Charger is off because WDT expired"));
    break;
    case 0x0c :
    Serial.println(F("Charger suspended or reduced current because of JEITA thermal limit"));
    break;
    case 0x0d :
    Serial.println(F("Charger suspended because battery removal detected via THM pin."));
    break;
    
  }


  
}

void parse_charger_stat(charger_stat val, char *desc, int max_len)
{
  switch (val) {
    case charge_done :
    strncpy(desc, "Charge Done", max_len);
    break;
    case charging :
    strncpy(desc, "Charging", max_len);
    break;
    case charge_UVLO :
    strncpy(desc, "Charge Input UVLO", max_len);
    break;
    case charge_OVLO :
    strncpy(desc, "Charge Input OVLO", max_len);
    break;
    case batt_fault :
    strncpy(desc, "Battery Fault", max_len);
    break;
    case charge_thermal_shdn :
    strncpy(desc, "Charger thermal shutdown", max_len);
    break;
    case batt_OV :
    strncpy(desc, "Battery overvoltage", max_len);
    break;
    case invalid :
    strncpy(desc, "Charger status invalid", max_len);
    return;
  }
}
