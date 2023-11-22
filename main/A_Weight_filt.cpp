/******************************* SOURCE LICENSE *********************************
Copyright (c) 2020 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to 
the Licensee to use the following Information for both Commercial and Non-Commercial purposes.

This source code is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/
#include "A_Weight_filt.h"

#include <Arduino.h> // for serial print debug
//#include <stdlib.h> // For malloc/free
//#include <string.h> // For memset

/*
float32_t filter1_coefficients[15] = 
{
// Scaled for floating point

    0.2566000499958255, -0.00005132000999915944, -0.2566513700058246, 1.0407, -0.13149615,// b0, b1, b2, a1, a2
    1, -2.9998, 2.99960004, 2.979, -2.95809657,// b0, b1, b2, a1, a2
    1, 1, 0, 0, 0// b0, b1, b2, a1, a2

};
*/

// KOS For CMSIS LIbrary a1 and a2 coefficients must be negated
float32_t filter1_coefficients[15] = 
{
// Scaled for floating point

    0.21608552249999996, 0.4321710449999999, 0.21608552249999996, 0.1406, -0.0049420900000000005,// b0, b1, b2, a1, a2
    0.9428296419538987, -1.8856592839077975, 0.942829604240713, 1.8849, -0.88641848,// b0, b1, b2, a1, a2
    //0.9971016139445975, -1.994203227889195, 0.9971035682637609, 1.9942, -0.9942084099999999// b0, b1, b2, a1, a2  zero=1+-.0028j
	0.9971020925488623, -1.9942041850977246, 0.997102132432946, 1.9942, -0.9942084099999999// b0, b1, b2, a1, a2  zero = 1+-.0002j

};


filter1Type *filter1_create( void )
{
	//filter1Type *result = (filter1Type *)malloc( sizeof( filter1Type ) );	// Allocate memory for the object
	static filter1Type result[1];  // kos to replace malloc. should it be static?
	filter1_init( result );											// Initialize it
	return result;																// Return the result
}

void filter1_destroy( filter1Type *pObject )
{
	//free( pObject );
}

 void filter1_init( filter1Type * pThis )
{
	arm_biquad_cascade_df1_init_f32(	&pThis->instance, filter1_numStages, filter1_coefficients, pThis->state );
	filter1_reset( pThis );

}

 void filter1_reset( filter1Type * pThis )
{
	memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
	pThis->output = 0;									// Reset output

}

 int filter1_filterBlock( filter1Type * pThis, float * pInput, float * pOutput, unsigned int count )
{
	arm_biquad_cascade_df1_f32( &pThis->instance, pInput, pOutput, count );
	return count;

}
