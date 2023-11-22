/******************************* SOURCE LICENSE *********************************
Copyright (c) 2020 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to 
the Licensee to use the following Information for both Commercial and Non-Commercial purposes.

This source code is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/
// Begin header file, filter1.h

#ifndef FILTER1_H_ // Include guards
#define FILTER1_H_

#define ARM_MATH_CM4  // Use ARM Cortex M4F
#define __FPU_PRESENT 1   // Use floating point unit
#include <arm_math.h>    // Include CMSIS header

// Link with library: libarm_cortexM4lf_math.a (or equivalent)
// Add CMSIS/Lib/GCC to the library search path
// Add CMSIS/Include to the include search path
/*
Generated code is based on the following filter design:
<micro.DSP.FilterDocument sampleFrequency="#1" arithmetic="float" biquads="Direct1" classname="filter1" inputMax="#1" inputShift="#-1" >
  <micro.DSP.BlankFilter scale="#0.25660004999582553" >
    <micro.DSP.FilterStructure coefficientBits="#0" variableBits="#0" accumulatorBits="#0" biquads="Direct1" >
      <micro.DSP.FilterSection form="Direct1" historyType="WriteBack" accumulatorBits="#0" variableBits="#0" coefficientBits="#0" />
      <micro.DSP.FilterSection form="Direct1" historyType="WriteBack" accumulatorBits="#0" variableBits="#0" coefficientBits="#0" />
      <micro.DSP.FilterSection form="Comb" historyType="Double" accumulatorBits="#0" variableBits="#0" coefficientBits="#0" />
    </micro.DSP.FilterStructure>
    <micro.DSP.PoleOrZeroContainer >
      <micro.DSP.RealZero i="#0" r="#-1" isPoint="#true" isPole="#false" isZero="#true" symmetry="r" N="#1" cascade="#0" />
      <micro.DSP.RealPole i="#0" r="#0.0703" isPoint="#true" isPole="#true" isZero="#false" symmetry="r" N="#1" cascade="#0" />
      <micro.DSP.RealZero i="#0" r="#-1" isPoint="#true" isPole="#false" isZero="#true" symmetry="r" N="#1" cascade="#2" />
      <micro.DSP.RealPole i="#0" r="#0.0703" isPoint="#true" isPole="#true" isZero="#false" symmetry="r" N="#1" cascade="#0" />
      <micro.DSP.RealPole i="#0" r="#0.9848" isPoint="#true" isPole="#true" isZero="#false" symmetry="r" N="#1" cascade="#1" />
      <micro.DSP.RealZero i="#0" r="#1.0002" isPoint="#true" isPole="#false" isZero="#true" symmetry="r" N="#1" cascade="#0" />
      <micro.DSP.RealPole i="#0" r="#0.9001" isPoint="#true" isPole="#true" isZero="#false" symmetry="r" N="#1" cascade="#0" />
      <micro.DSP.RealZero i="#0" r="#0.9998" isPoint="#true" isPole="#false" isZero="#true" symmetry="r" N="#1" cascade="#1" />
      <micro.DSP.RealPole i="#0" r="#0.9971" isPoint="#true" isPole="#true" isZero="#false" symmetry="r" N="#1" cascade="#1" />
      <micro.DSP.RealZero i="#0.0002" r="#1" isPoint="#true" isPole="#false" isZero="#true" symmetry="c" N="#1" cascade="#1" />
      <micro.DSP.RealPole i="#0" r="#0.9971" isPoint="#true" isPole="#true" isZero="#false" symmetry="r" N="#1" cascade="#1" />
    </micro.DSP.PoleOrZeroContainer>
    <micro.DSP.CMSIS.CodeGenerator cpu="M4F" generateTestCases="#false" />
    <micro.DSP.GainControl magnitude="#1.1614486138403428" frequency="#0.056640625" peak="#true" />
  </micro.DSP.BlankFilter>
</micro.DSP.FilterDocument>

*/

extern float32_t filter1_coefficients[15];
static const int filter1_numStages = 3;

typedef struct
{
	arm_biquad_casd_df1_inst_f32 instance;
	float32_t state[12];
	float32_t output;
} filter1Type;


filter1Type *filter1_create( void );
void filter1_destroy( filter1Type *pObject );
 void filter1_init( filter1Type * pThis );
 void filter1_reset( filter1Type * pThis );
#define filter1_writeInput( pThis, input )  \
	arm_biquad_cascade_df1_f32( &pThis->instance, &input, &pThis->output, 1 );

#define filter1_readOutput( pThis )  \
	pThis->output


 int filter1_filterBlock( filter1Type * pThis, float * pInput, float * pOutput, unsigned int count );
#define filter1_outputToFloat( output )  \
	(output)

#define filter1_inputFromFloat( input )  \
	(input)

#endif // FILTER1_H_
	
