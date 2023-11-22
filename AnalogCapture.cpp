/* Analg ADC signal capture class  


 *  Adafruit Metro SAMD51 processor with 2ADCs
 *  DMA, Event Trigger and dual ADC interface
 *  Timer TC generates sampling trigger that goes through event system and connects
 *  to ADC that is triggered by each TC trigger.
 *  ADC RESRDY writes new result to DMA which places in a buffer. When buffer is full, DMA
 *  should trigger an interrupt that sets the dmadone flag and switch to alternate buffer.
 *
 *  Uses DMA, TC, EVSYS, GCLK4 per instance
 *  Up to 2 instances can be used with ADC0 using DMA Ch.0 and ADC1 using Ch.1.
 *
 *  Author: K O'Sullivan
 *  (c) 2018 Majentix Ltd 

*/

#include <Arduino.h>
#include "AnalogCapture.h"


bool toggle=0;  //used to toggle gpio in inteerupt routines fortest

/*
 * ADC Pin mappings
 * ADC1 [AIN0 Port pin A4, device pin 11] used for noise.
 * ADC0 [AIN6 (A1), AIN5 (A2), AIN4 (A3) ] used for Vibration
 */
//uint16_t adc_inputctrl[3] = {ADC_INPUTCTRL_MUXPOS_AIN5, ADC_INPUTCTRL_MUXPOS_AIN6, ADC_INPUTCTRL_MUXPOS_AIN4};  // table for round robin
uint16_t adc_inputctrl[3] = {ADC_INPUTCTRL_MUXPOS_AIN4, ADC_INPUTCTRL_MUXPOS_AIN5, ADC_INPUTCTRL_MUXPOS_AIN6};  // table for round robin
uint8_t adc_pin;

/*
TC timer generates sampling trigger that connects to the event system. The adc is triggered
by event system output
*/






int AnalogCapture ::_beginCount = 0;

void AnalogCapture :: Start_transfer(uint8_t adc_id, transfer_mode_t transfer_mode, void *rxdata, size_t hwords){
 
    
    DMAC->Channel[adc_id].CHINTENSET.reg = DMAC_CHINTENSET_MASK;  // Enable all 3 interrupts: SUSP, TCMPL and TERR
    
// Setup ADC DMA Transfer    
    _descriptor.descaddr = 0;
    if (adc_id == 1)
       _descriptor.srcaddr = (uint32_t) &ADC1->RESULT.reg;
    else
       _descriptor.srcaddr = (uint32_t) &ADC0->RESULT.reg;
       
    _descriptor.btcnt =  hwords;
    _descriptor.dstaddr = (uint32_t)rxdata + (hwords)*2;   // end address
    _descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID | DMAC_BTCTRL_BLOCKACT_INT;  // kos 11/10/18 added DMAC_BTCTRL_BLOCKACT_INT to generate an interrupt at end
    memcpy(&_descriptor_section[adc_id],&_descriptor, sizeof(dmacdescriptor));

    dmadone[adc_id] = 0;
    // start channel
         DMAC->Channel[adc_id].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;    // Enable the DMAC  // SAMD51 kos
 
 
 // turn on the timing triggers to the ADC.
 /*
  * We need this because the desequencer DMA and the data capture DMA need to start on the same trigger and
  * this is the easiest place to enable adc conversions so the the DESEQ and RESRDY will trigger both DMAs
  */
   if (adc_id==0){
      ADC0->EVCTRL.reg = ADC_EVCTRL_STARTEI;
      adc_pin = 1; // reset adc pin round robin to zero
   }
   else
      ADC0->EVCTRL.reg = ADC_EVCTRL_STARTEI;
   
}

void AnalogCapture :: Stop_transfer(uint8_t adc_id){
}

void AnalogCapture :: Sleep(uint8_t adc_id){
}
void AnalogCapture :: End(){
_beginCount--;

  if (_beginCount == 0) {
    // disable the interrupt
        NVIC_DisableIRQ(DMAC_0_IRQn);
      NVIC_DisableIRQ(DMAC_1_IRQn);
      NVIC_DisableIRQ(DMAC_2_IRQn);
      NVIC_DisableIRQ(DMAC_3_IRQn);
      NVIC_DisableIRQ(DMAC_4_IRQn);
      DMAC->CTRL.bit.DMAENABLE = 0;
      MCLK->AHBMASK.bit.DMAC_ = 0;

// Need to shut down ADCs and Evesys below. To be done later.
  
    }
}

void AnalogCapture :: Wake(uint8_t adc_id) {
}
void AnalogCapture :: Wake() {
}

void  enable_ahb_clocks(uint8_t adc_id){

   // Turn the peripheral clocks on
    MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS; 

  if (adc_id ==1){
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC5;
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC1;
   } else{
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4;
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC0;
   }
}


void AnalogCapture :: _dma_init(){
MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC; //bit.DMAC = 1;
   DMAC->CTRL.bit.SWRST = 1;    

    DMAC->BASEADDR.reg = (uint32_t)_descriptor_section;
    DMAC->WRBADDR.reg = (uint32_t)_wrb;
    DMAC->CTRL.reg =  DMAC_CTRL_LVLEN(0xf) | DMAC_CTRL_DMAENABLE;  //  enable just yet?
//DMAC->PRICTRL0.bit.RRLVLEN0 = 1;  // Set the arbitration to round robin so both ADCs get alternating DMA channel priority

   NVIC_DisableIRQ(DMAC_0_IRQn);
    NVIC_ClearPendingIRQ(DMAC_0_IRQn);
    NVIC_EnableIRQ(DMAC_0_IRQn);
    NVIC_SetPriority(DMAC_0_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    NVIC_DisableIRQ(DMAC_1_IRQn);
    NVIC_ClearPendingIRQ(DMAC_1_IRQn);
    NVIC_EnableIRQ(DMAC_1_IRQn);
    NVIC_SetPriority(DMAC_1_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    NVIC_DisableIRQ(DMAC_2_IRQn);
    NVIC_ClearPendingIRQ(DMAC_2_IRQn);
    NVIC_EnableIRQ(DMAC_2_IRQn);
    NVIC_SetPriority(DMAC_2_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    NVIC_DisableIRQ(DMAC_3_IRQn);
    NVIC_ClearPendingIRQ(DMAC_3_IRQn);
    NVIC_EnableIRQ(DMAC_3_IRQn);
    NVIC_SetPriority(DMAC_3_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    NVIC_DisableIRQ(DMAC_4_IRQn);
    NVIC_ClearPendingIRQ(DMAC_4_IRQn);
    NVIC_EnableIRQ(DMAC_4_IRQn);
    NVIC_SetPriority(DMAC_4_IRQn, (1 << __NVIC_PRIO_BITS) - 1);


      // clear the descriptor for all the channels 
      for (int i=0;i<NUM_DMA_CHANNELS;i++)
      memset((void*)&_descriptor_section[i], 0x00, sizeof(dmacdescriptor));
}


void AnalogCapture :: AllocateDMAChannel(uint8_t channel){
/* dma channel is 0 or 1 and is hardwired to the same numbered adc.  
*  ADC0 uses dmac channel 0, ADC1 uses dmac channel1
*/ 

      // select the channel and reset it
      DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 0;
      DMAC->Channel[channel].CHCTRLA.bit.SWRST = 1;

 

    // Set the DMAC level, trigger source and trigger action to burst (trigger for every byte transmitted)

switch (channel) {
  case 0:  // ADC0
    DMAC->Channel[channel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(0x44) | DMAC_CHCTRLA_TRIGACT_BURST; // 0x44 is the ADC0 RESRDY 0x46 for ADC1 kos datasheet pg446
    break;
  case 1:  // ADC1
    DMAC->Channel[channel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(0x46) | DMAC_CHCTRLA_TRIGACT_BURST;
    break;
}


    DMAC->Channel[channel].CHPRILVL.reg = DMAC_CHPRILVL_PRILVL(0);    // Set the channel priority level   
}


void AnalogCapture :: _adc_init(uint8_t adc_id){

Adc * adc_inst;


  
   if (adc_id == 0){
      adc_inst = ADC0;
      // Enable GCLK for ADC0  (ADC0 input clock) 
      GCLK->PCHCTRL[GCLK_ADC0].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK4;   //kos  using ADC1 and driving it with GCLK_PCHCTRL_GEN_GCLK4 which is a 12MHz clock setup in startup.c
    }
   else {
     adc_inst = ADC1;
      // Enable GCLK for ADC1  (ADC1 input clock)  may need to do same for ADC0
      GCLK->PCHCTRL[GCLK_ADC1].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK4;   //kos  using ADC1 and driving it with GCLK_PCHCTRL_GEN_GCLK4 which is a 12MHz clock setup in startup.c
    }

  adc_inst->CTRLA.bit.ENABLE = 0x00;             // Disable ADC because some registers can only be written when ENABLE is low. 
  while (adc_inst->SYNCBUSY.bit.ENABLE == 1); //Just wait till the ADC is free  SAMD51 kos

  adc_inst->CTRLA.reg = ADC_CTRLA_SWRST; // reset ADC to its initial settings
   while (adc_inst->SYNCBUSY.bit.SWRST == 1); //Just wait till the ADC is free  SAMD51 kos
  
  
  adc_inst->CTRLA.reg |= ADC_CTRLA_PRESCALER_DIV4;
 
  // event system control input triggered by TC5 and output RESRDYEO triggers DMAC
  adc_inst->EVCTRL.reg = ADC_EVCTRL_STARTEI; 
  // Select port pins AIN0 (MCU Port PB04,Board Pin A8) for Audio
if (adc_id == 1){
     ADC1->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_AIN0;   // ADC1 [AIN0 Port pin A4, device pin 11] used for noise.  ADC0 [AIN4 (A3), AIN6 (A1), AIN5 (A2)] used for Vibration
       while (adc_inst->SYNCBUSY.bit.INPUTCTRL == 1); //Just wait till the ADC is free  SAMD51 kos
    }
     else {

     ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_AIN4;  // ADC 0 [AIN4 Portpin A3, device pin 13] ***Note that we need to use dma sequencing and this has snot been coded yet.
     while (adc_inst->SYNCBUSY.bit.INPUTCTRL == 1); //Just wait till the ADC is free  SAMD51 kos
     // Configure interrupt request
 // This will generate an interrupt for every ADC1 sample. 
 // We dont really need this as a more useful will be an interrupt by the DMA controller when there is captured audio data to process
    NVIC_DisableIRQ(ADC0_1_IRQn);
    NVIC_ClearPendingIRQ(ADC0_1_IRQn);
    NVIC_SetPriority(ADC0_1_IRQn, 0);
    NVIC_EnableIRQ(ADC0_1_IRQn);
     
    ADC0->INTENSET.bit.RESRDY = 1; // enable interrupt on completion of every conversion for ADC0
   
   }
   

  adc_inst->CTRLB.reg |= ADC_CTRLB_RESSEL_16BIT;
  while (adc_inst->SYNCBUSY.bit.CTRLB == 1);
  adc_inst->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // set the reference to analog vcc
  while (adc_inst->SYNCBUSY.bit.REFCTRL == 1);
  if (adc_id == 1)
     adc_inst->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_4 | ADC_AVGCTRL_ADJRES(0x01);  //13 bits with some noise reduction
  else // vibration higher precision
     adc_inst->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0x02);  //14 bits with some noise reduction
     
  adc_inst->SAMPCTRL.reg = 0x03; // Set the sample time to 3 ADC clock cycles . The min is 2 I think.
  while (adc_inst->SYNCBUSY.bit.SAMPCTRL == 1); //Just wait till the ADC is free  SAMD51 kos
  adc_inst->CTRLA.bit.ENABLE = 0x01;
  while (adc_inst->SYNCBUSY.bit.ENABLE == 1); //Just wait till the ADC is free  SAMD51 kos
}

void AnalogCapture :: _tcConfigure(uint8_t adc_id) {
  Tc * tc_inst;
  if (adc_id == 1){
     tc_inst = TC5;
     // Enable GCLK for TC5 and TC5 (timer counter input clock)
     GCLK->PCHCTRL[GCLK_TC5].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK4;   //kos  using GCLK_TC5 and driving it with GCLK_PCHCTRL_GEN_GCLK4 which is a 12MHz clock setup in startup.c
  }
  else{ // assume ADC0
     tc_inst = TC4;
     GCLK->PCHCTRL[GCLK_TC4].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK4;
  }
  delay (100);
  // first disable TC5 before resetting
  tc_inst->COUNT16.CTRLA.reg = 0;
  while (tc_inst->COUNT16.SYNCBUSY.bit.ENABLE ==1);

  tc_inst->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tc_inst->COUNT16.SYNCBUSY.bit.SWRST ==1);

// Set Timer counter Mode to 16 bits
 tc_inst->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set tc_inst mode as match frequency such that CC) is the top value and counter resets on hitting that value
 tc_inst->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;  
 //set prescaler and enable tc_inst
 tc_inst->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;
 //set tc_inst timer counter based off of the system clock and the user defined sample rate or waveform
 tc_inst->COUNT16.CC[0].reg = (uint16_t) (12000000 / _samplerate[adc_id] - 0);   // was supposed to be -1 but seemingly not
 while (tc_inst->COUNT16.SYNCBUSY.bit.CC0 ==1);  
 
// setup an event output when timer hits matching top MCEO0=1
  tc_inst->COUNT16.EVCTRL.bit.MCEO0 = 1;  //  MCEO0 with 1 See SAMD datasheet TC chapter 30.6.4.3

 tc_inst->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

while (tc_inst->COUNT16.SYNCBUSY.bit.ENABLE ==1);  
  
 // Configure interrupt request
 // This will generate an interrupt for every ADC1 sample. 
 // We dont really need this as a more useful will be an interrupt by the DMA controller when there is captured audio data to process
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the tc_inst interrupt request
 tc_inst->COUNT16.INTENSET.bit.MC0 = 1;
 // while (tcIsSyncing()); //wait until TC5 is done syncing 
}

void AnalogCapture :: _tcStartcounter(uint8_t adc_id){

  Tc * tc_inst;
  if (adc_id == 1)
     tc_inst = TC5;   
  else // assume ADC0
     tc_inst = TC4;

  tc_inst->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
 while (tc_inst->COUNT16.SYNCBUSY.bit.ENABLE ==1);
}

void AnalogCapture :: _tcDisable(uint8_t adc_id){

  Tc * tc_inst;
  if (adc_id == 1)
     tc_inst = TC5;   
  else // assume ADC0
     tc_inst = TC4;

  tc_inst->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; //set the CTRLA register
 while (tc_inst->COUNT16.SYNCBUSY.bit.ENABLE ==1);
}

void AnalogCapture :: _setup_evsys(uint8_t adc_id){

/*  The generator is the timer that generates the sampling clock for the ADC so the timer TCn feeds the event system as a generator
 *  The user is the ADC (0 or 1) and this is used to trigger a conversion.
 */
 if (adc_id == 1){
  // kos Enable GCLK for event system generator. PCHCTRL[GCLK_EVSYS[0]].reg is the register than enables GCLK for EVSYS0 and selects its clock source. See Pg.170
   GCLK->PCHCTRL[22].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK4;   //kos  using EVSYS[0] mapped to 22 and driving it with GCLK_PCHCTRL_GEN_GCLK4 which is a 12MHz clock setup in startup.c
   EVSYS->USER[57].reg = (uint8_t)  (EVSYS_USER_CHANNEL(1));  //USER Select:  55 selects start ADC0 and 57 starts ADC1 conversion when timer matches count User Channel n require n+1 so this is for CHANNEL 0
   EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_EVGEN(0x59) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT;
  }
  else{ // assumed ADC0
// kos Enable GCLK for event system generator. PCHCTRL[GCLK_EVSYS[0]].reg is the register than enables GCLK for EVSYS0 and selects its clock source. See Pg.170
   GCLK->PCHCTRL[21].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK4;   //kos  using EVSYS[0] mapped to 22 and driving it with GCLK_PCHCTRL_GEN_GCLK4 which is a 12MHz clock setup in startup.c
   EVSYS->USER[55].reg = (uint8_t)  (EVSYS_USER_CHANNEL(2));  //USER Select:  55 selects start ADC0 and 57 starts ADC1 conversion when timer matches count User Channel n require n+1 so this is for CHANNEL 0
   EVSYS->Channel[1].CHANNEL.reg = EVSYS_CHANNEL_EVGEN(0x56) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT;
  }
}

//this function gets called by the interrupt at <samplerate>Hertz
void  TC5_Handler (void) {
  //YOUR CODE HERE 
  digitalWrite(13,toggle);
  toggle = !toggle;
  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

void  TC4_Handler (void) {
  //YOUR CODE HERE 
   
  // END OF YOUR CODE
  TC4->COUNT16.INTFLAG.bit.MC0 = 1; 
}


void ADC0_1_Handler(void) {
   //YOUR CODE HERE . THis is where we should change the ADC input pin on each conversion.

ADC0->INPUTCTRL.reg = adc_inputctrl[adc_pin++];
if (adc_pin ==3)
    adc_pin = 0;

//digitalWrite(13,toggle);
//toggle = !toggle;

  // END OF YOUR CODE

  ADC0->INTFLAG.bit.RESRDY = 1;  // clear the flag at the end of the interrupt
}



 void AnalogCapture :: Set_samplerate(uint8_t adc_id, uint32_t sample_rate) {
_samplerate[adc_id] = sample_rate;
}

 void AnalogCapture :: Set_buffersize(uint8_t adc_id, uint32_t buffersize) {
_buffersize[adc_id] = buffersize;
}




void AnalogCapture :: Initialise(){
   if (_beginCount == 0) {
   enable_ahb_clocks(1);
   enable_ahb_clocks(0);
   _dma_init();
   _adc_init(1);
   _adc_init(0);
   _tcConfigure(1);
   _tcConfigure(0);
   _tcStartcounter(1);
   _tcStartcounter(0);
   EVSYS->CTRLA.reg = EVSYS_CTRLA_SWRST; // reset the event controller
   _setup_evsys(1);
   _setup_evsys(0);

   }
  _beginCount++;
}


 void  AnalogCapture ::onService(){  // kos did have 'static' before it but compiler error
    int active_channel = DMAC->INTPEND.bit.ID;
    // Invalidate the channel
//    Serial.print("DMA ACTIVE 0x");Serial.print(DMAC->ACTIVE.reg, HEX);
//    Serial.print("DMA ERROR Ch-");Serial.print(active_channel);Serial.print(" CHSTATUS:-0x");Serial.print(DMAC->Channel[active_channel].CHSTATUS.reg, HEX);

if (active_channel ==2)
   Serial.print("DSEQ*********************************");
    
    _descriptor_section[active_channel].btctrl &= (0xFFFE); // | ~DMAC_BTCTRL_VALID);
    if (DMAC->Channel[active_channel].CHINTFLAG.bit.TERR) {
        // clear the error interrupt and call the error callback if there is one        
        Serial.print("   CHINTFLAG:-0x");Serial.println(DMAC->Channel[active_channel].CHINTFLAG.reg, HEX);
        DMAC->Channel[active_channel].CHINTFLAG.bit.TERR = 1;
       }
    
     if (DMAC->Channel[active_channel].CHINTFLAG.bit.TCMPL) {
       // clear the complete interrupt and call the callback if there is one
        DMAC->Channel[active_channel].CHINTFLAG.bit.TCMPL = 1;
        dmadone[active_channel] = 1;
        if (active_channel == 1)
           ADC1->EVCTRL.reg = ~ADC_EVCTRL_STARTEI; // Stop the ADC from converting       
        
        else
           ADC0->EVCTRL.reg = ~ADC_EVCTRL_STARTEI; // Stop the ADC from converting
       }
     
  }


AnalogCapture DMA;

extern "C" {
  static void _dmac_handler(void) {
    DMA.onService();
    }

  void DMAC_0_Handler(void){
      _dmac_handler();
    }

  void DMAC_1_Handler(void){
      _dmac_handler();
    }

  void DMAC_2_Handler(void){
      _dmac_handler();
    }

  void DMAC_3_Handler(void){
      _dmac_handler();
    }

  void DMAC_4_Handler(void){
      _dmac_handler();
    }
}
