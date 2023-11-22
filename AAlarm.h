/* AAlarm.h - Library for managing alarms
   Created by Killian O'Sullivan 1/8/2018 Majentix Ltd

  mag_threshold - The magnitude above which an alarm condition is detected
  time_threshold - The amount of time that elapses for which the mag_threshold has been breached before alarm is activated
  alarm_timeout  - The minimum amount of time that an alarm will be activated and enable the re
  hysteresis - the number of consecutive posted mag entries over threshold to start the alarm timer or under treshold to reset the alarm 
  relay_pin      - The physical pin that activates the alarm relay

  AAlarm::post(mag) - Posts magnitude data for comparison with mag_threshold
  AAlarm::event() - simply updates timers, activates or deactvates alarms as needed. Should be in a periodic loop
*/
#ifndef AAlarm_h
#define AAlarm_h

#include <Arduino.h>

class AAlarm
{
  public:
    AAlarm();
    int mag_threshold;
    unsigned long time_threshold;
    unsigned long alarm_timeout;
    int hysteresis;
    int relay_pin;
    
    
    void post(int mag);
    void event();
    
  private:
     int _hysteresis_lo; //counts the number of times that mag < mag_threshold
     int _hysteresis_hi; //counts the number of times that mag > mag_threshold
     unsigned long _when_alarm_timer_started;
     unsigned long _when_alarm_finished;
     bool _alarm_detected;

};
#endif
