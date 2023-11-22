/*
  egg_timer.cpp - enables timed events to be executed in the main loop of a program
    Created by Killian O'Sullivan 1/8/2018 Majentix Ltd
 
*/

#include <Arduino.h>
#include "Egg_Timer.h"
#include <TimeLib.h>


Egg_Timer::Egg_Timer()
{
// Defaults
_when_timer_started = now();
_time_up = 1;  // defaulting to 1 will ensure an early call at startup
enabled = 0;
} 

void Egg_Timer::reset()
{
  _when_timer_started = now();
  _time_up = 0;

}


bool Egg_Timer::check()
 {   
  int elapsed;
  elapsed = now() - _when_timer_started;
    if (elapsed >= duration){
      _time_up = 1;
    }
   return _time_up;
 }
