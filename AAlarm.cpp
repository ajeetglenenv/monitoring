/*
  AAlarm.cpp - Library for managing alarms
    Created by Killian O'Sullivan 1/8/2018 Majentix Ltd
*/

#include <Arduino.h>
#include "AAlarm.h"


AAlarm::AAlarm()
{
// Defaults
mag_threshold = 5;
time_threshold = 10000;
alarm_timeout = 30000;
hysteresis = 3;
//relay_pin = 2;

//pinMode(relay_pin, OUTPUT);

_when_alarm_finished = 0;
_when_alarm_timer_started = millis();
_alarm_detected = 0;
_hysteresis_lo = 0;
_hysteresis_hi = 0;
} 

void AAlarm::post(int mag)
{
  if (_alarm_detected == 1){
    if (mag < mag_threshold) {
        _hysteresis_lo++;
        if (_hysteresis_lo > hysteresis) {  // have we reached hysteresis number of iterations to reset the alarm
           _hysteresis_lo = 0;  // reset the hysteresis counter
           _alarm_detected = 0;
           _when_alarm_finished = millis();
          }
        }
        else
          _hysteresis_lo = 0; // reset the hysteresis counter because mag is > mag  threshold
        
    }
    else
     if (mag >= mag_threshold){
        _hysteresis_hi++;
        if (_hysteresis_hi > hysteresis) { // have we reached hysteresis number of iterations to set the alarm
           _hysteresis_hi = 0;
           _alarm_detected = 1;
           _when_alarm_timer_started = millis();
           }
       }
       else
          _hysteresis_hi = 0;

}


void AAlarm::event()
{   
  int elapsed1, elapsed2;
  elapsed1 = millis() - _when_alarm_timer_started;
  elapsed2 = millis() - _when_alarm_finished;
  /*
  if (_alarm_detected == 1){
    if (elapsed1 >= time_threshold)
     //      digitalWrite(relay_pin, HIGH);
     // DEBUG_STREAM.print("elapsed=");Serial.print(elapsed);Serial.println(":  AA Act");
  }
  else{
    if (elapsed2 >= alarm_timeout)
//      digitalWrite(relay_pin, LOW);
//    DEBUG_STREAM.print("AA Deactivated");

  }
 */   


}
