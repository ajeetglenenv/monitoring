/* AAlarm.h - Library for managing alarms
   Created by Killian O'Sullivan 14/12/2018 Majentix Ltd

  duration - The duration of the timer in seconds
  time_up - A flag that indicates that the timer has expired since it was last reset and duration has passed.
  
  Egg_Timer::reset() - Resets and starts the timer
  Egg_Timer::check() - simply updates timer and checks if itis time up. If so, the time_up flag is set 
*/
#ifndef Egg_Timer_h
#define Egg_Timer_h

#include <Arduino.h>
#include <TimeLib.h>

class Egg_Timer
{
  public:
    Egg_Timer();
    time_t duration;
    boolean enabled;
    
    void reset();
    bool check();
    
  private:
     bool _time_up;
     time_t _when_timer_started;
     
     
};
#endif
