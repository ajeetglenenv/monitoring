/* Percentile_Table.h - Calculates Leq10 and Leq90.
   Created by Killian O'Sullivan 14/12/2018 Majentix Ltd

  percentile - The percentile to be reported on the table contents
   - A flag that indicates that the timer has expired since it was last reset and duration has passed.
  
  Percentile_Table::reset() - Resets the table
  Percentile_Table::post() - simply updates timer and checks if itis time up. If so, the time_up flag is set 
*/
#ifndef Percentile_Table_h
#define Percentile_Table_h

#include <Arduino.h>

class Percentile_Table
{
  public:
    Percentile_Table();
    uint8_t percentile;
    
    void reset();
    bool post(int leq);
    uint8_t get_percentile();
    
  private:
     uint32_t _data_count;  
     uint32_t _table[128];  
     
};
#endif
