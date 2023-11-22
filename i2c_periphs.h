#ifndef i2c_periphs_h
#define i2c_periphs_h

#define EEPROM_DEV_ADDR 0x57
#define MAX77962_DEV_ADDR 0x69
#define CHG_CNFG_00_SADDR 0x16

#define MAX17261_DEV_ADDR   0x36
#define BATTERY_CAPACITY  56000  // milliamp hours
#define BATTERY_IchargeTerm  256  // 80mA converted at 312.5uA per LSB
#define BATTERY_VEmpty 0xA561    // default 3.3V empty and 3.88 recovery. see page 28 of UG

void init_i2c();
int i2c_write(byte dev_addr, byte sub_addr, unsigned char * pdata, byte len); // len must be <  32 bytes
int i2c_read (byte dev_addr, byte sub_addr, unsigned char * pdata, byte len);
byte get_batt_percent();
float get_batt_voltage();
float get_batt_current();
void init_batt_mon();
int restore_battmon_params();
void save_battmon_params();
word get_batt_mon_cycles();
void wipe_batt_mon_params();
void init_charger();
word get_batt_timetoempty();
int is_charging();
void set_chgin_ilim(unsigned char val);
void set_vchgin_reg(unsigned char val);
void get_chg_loopvars( unsigned int * aicl_ok, unsigned int * chginlim_ok, unsigned int * uvlo);
#endif

 enum charger_stat {charge_done, charging, charge_UVLO, charge_OVLO, batt_fault, charge_thermal_shdn, batt_OV, invalid} ; 
 typedef enum charger_stat charger_stat;

 charger_stat get_charger_stat();
 void parse_charger_stat(charger_stat val, char *desc, int max_len);
 void get_charger_mode();
