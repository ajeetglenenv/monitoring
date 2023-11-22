/* Analg ADC signal capture class  


 *  Adafruit Metro SAMD51 processor with 2ADCs
 *  DMA, Event Trigger and dual ADC interface
 *  Timer TC generates sampling trigger that goes through event system and connects
 *  to ADC that is triggered by each TC trigger.
 *  ADC RESRDY writes new result to DMA which places in a buffer. When buffer is full, DMA
 *  should trigger an interrupt that sets the dmadone flag and switch to alternate buffer.
 *
 *  Uses DMA, TC4, TC5 EVSYS, GCLK4 and the one instance initialises both ADCs with DMA
 *  Only 1 instances can be used.
 *
 *  Author: K O'Sullivan
 *  (c) 2018 Majentix Ltd 

*/

#ifndef AnalogCapture_h
#define AnalogCapture_h

#include <Arduino.h>

#define NUM_DMA_CHANNELS 4
#define ADCPIN A3

/*
 * ADC Pin mappings
 * ADC1 [AIN0 Port pin A4, device pin 11] used for noise.
 * ADC0 [AIN6 (A1), AIN5 (A2), AIN4 (A3) ] used for Vibration
 */

/*
#define noise_ch  ADC_INPUTCTRL_MUXPOS_AIN0
#define vib_chx ADC_INPUTCTRL_MUXPOS_AIN6
#define vib_chy ADC_INPUTCTRL_MUXPOS_AIN5
#define vib_chz ADC_INPUTCTRL_MUXPOS_AIN4
*/
 
    

#define GCLK_TC4 30 // same as TC5
#define GCLK_TC5 30
#define GCLK_ADC0 40
#define GCLK_ADC1 41


// adc_id will either be ADC0 or ADC1

typedef enum {
   ONE_SHOT,
   CONTINUOUS
} transfer_mode_t;

typedef struct {
    uint16_t btctrl;
    uint16_t btcnt;
    uint32_t srcaddr;
    uint32_t dstaddr;
    uint32_t descaddr;
} dmacdescriptor;



extern  volatile uint32_t  dmadone[2];


void enable_ahb_clocks(uint8_t adc_id);


/* Class declaration for Noise and vibration capture object
 *  AnalogCapture uses 2 channels od the DMAC, 2 ADCs, the Event system,
 *  TC4 and TC 5 timers.
 *  adc_id is 0 or 1 corresponding with ADC0 or ADC1
 */

class AnalogCapture  {
 public:
  AnalogCapture ();
  void Initialise();
  void AllocateDMAChannel(uint8_t channel); // sets up descriptors and initialises specific DMA channel
  void Start_transfer(uint8_t adc_id, transfer_mode_t transfer_mode, void *rxdata, size_t hwords);
  void Stop_transfer(uint8_t adc_id);

  void Sleep(uint8_t adc_id);  // Turns off clocks to evsys, dma, adc, TC and disables TC. May add power management stuff here as well
  void End();  // End DMA, both adcs. common clocks disabled
  void Wake(uint8_t adc_id);  // Reverses Sleep function to put everything back in an active state ready for Transfer.
  void Wake(); // Wake both adcs
  void Set_samplerate(uint8_t adc_id, uint32_t samplerate);
  void Set_buffersize(uint8_t adc_id, uint32_t buffersize);
  void onService();  // DMAC service interrupt on completion of transfer
 // ~AnalogCapture (); // deconstructor to deallocate everything

  



 private:
 uint32_t _samplerate[2] = {800, 44100};
 uint32_t _buffersize[2] = {1024, 1024};
 static int _beginCount;
 
 
 void _dma_init();
 void _adc_init( uint8_t adc_id);
 void _tcConfigure(uint8_t adc_id);
 void _tcStartcounter(uint8_t adc_id);
 void _tcDisable(uint8_t adc_id);
 void _setup_evsys(uint8_t adc_id);
 

// descriptors should be in the class but I had difficulty reconciling it with the DMA handler so made it global.
 volatile dmacdescriptor _wrb[NUM_DMA_CHANNELS] __attribute__ ((aligned (16)));
  dmacdescriptor _descriptor_section[NUM_DMA_CHANNELS] __attribute__ ((aligned (16)));
  dmacdescriptor _descriptor __attribute__ ((aligned (16)));

 

};

inline AnalogCapture :: AnalogCapture (void) {
int dmadone[2] = {0,0};
 }


#endif
