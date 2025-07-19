/*
 * conv-direct.h
 *
 *  Created on: Feb 17, 2025
 *      Author: hijaz
 */

#ifndef SRC_CONV_DIRECT_H_
#define SRC_CONV_DIRECT_H_



#include "arm_math.h"
#include "main.h"
#define DMA_ADC_BUFF_SIZE 4

//MACRO FIR FILTER

#define SAMPLING_FREQUENCY 10000.0f
#define CONVDIRECT_KERNEL_GAIN 1.00f


#define CONVDIRECT_TAPS_LENGTH 79
#define CONVDIRECT_BUFFER_LENGTH  DMA_ADC_BUFF_SIZE/2

typedef struct
{
	arm_fir_instance_f32 firProcessor;
	float32_t firBuf[CONVDIRECT_TAPS_LENGTH+CONVDIRECT_BUFFER_LENGTH-1];

	float32_t kernelTimerReversed[CONVDIRECT_TAPS_LENGTH];
	float32_t out[CONVDIRECT_BUFFER_LENGTH];

}ConvDirect_Container;

void ConvDirect_Init(ConvDirect_Container *c, const float32_t *kernel );
void ConvDirect_Update(ConvDirect_Container *c, float32_t *inp);

extern const float32_t filter_taps[CONVDIRECT_TAPS_LENGTH];

#endif /* SRC_CONV_DIRECT_H_ */
