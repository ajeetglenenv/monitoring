/*
  Percentile_Table.cpp - enables Leq10 and Leq90 percentile calculations
    Created by Killian O'Sullivan 1/8/2018 Majentix Ltd
 
*/

#include <Arduino.h>
#include "Percentile_Table.h"



Percentile_Table::Percentile_Table()
{
// Defaults
percentile=10;  
//_data_count = 0;
Percentile_Table::reset();
} 

void Percentile_Table::reset()
{
  for (int i = 0; i<128;i++) 
   _table[i] = 0;
_data_count = 0;
}


bool Percentile_Table::post(int leq)
 {   
  if ((leq > -1) && (leq < 128)){
    _table[leq]++;
    _data_count++;

    return true; 
  }
   else
    return false;
  }


/*
For Leq 10, the _data_count/10 is the number of entries that exceed a certain threshold
 and we need to find this threshold.  We do this by starting at the loudest end of the table (table[127])
 and read and accumulate its contents while decrementing the table entry
*/
uint8_t Percentile_Table::get_percentile()
 {   
   uint8_t addr = 127;
   uint32_t entries = 0;
   
   while (( entries <= (_data_count*percentile)/100) && (addr > 0)){
        entries += _table[addr];
         addr--;
     } 

//Serial.print("_data_count=");Serial.println(_data_count);
//Serial.print("percentile=");Serial.println(percentile);
//Serial.print("addr=");Serial.println(addr);

  
   return addr;  // Leqxx
 }
