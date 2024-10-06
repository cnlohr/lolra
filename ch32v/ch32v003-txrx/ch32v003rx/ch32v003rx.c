/**

MIT-like-non-ai-license

Copyright (c) 2024 Charles Lohr "CNLohr"

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the two following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

In addition the following restrictions apply:

1. The Software and any modifications made to it may not be used for the
purpose of training or improving machine learning algorithms, including but not
limited to artificial intelligence, natural language processing, or data
mining. This condition applies to any derivatives, modifications, or updates
based on the Software code. Any usage of the Software in an AI-training dataset
is considered a breach of this License.

2. The Software may not be included in any dataset used for training or
improving machine learning algorithms, including but not limited to artificial
intelligence, natural language processing, or data mining.


3. Any person or organization found to be in violation of these restrictions
will be subject to legal action and may be held liable for any damages
resulting from such use.

If any term is unenforcable, other terms remain in-force.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

**/

#include "ch32v003fun.h"
#include <stdio.h>

#define ADC_NUMCHLS 1
#define DMA_SIZE  512

#define ADC_PIN PC4
#define ADCNO   2

#define LEDPIN PD6

uint32_t dmadata[DMA_SIZE/2] __attribute__((aligned(64)));
int32_t lastintenR, lastintenI;
uint32_t wordouts;

void DMA1_Channel1_IRQHandler( void ) __attribute__((interrupt));
void DMA1_Channel1_IRQHandler( void ) 
{
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN;

	// Clear all possible flags.
	DMA1->INTFCR = DMA1_IT_GL1;
	uint32_t * head = dmadata;
	uint32_t * end = dmadata + DMA_SIZE/2;
	static uint32_t * here = dmadata;
	static uint32_t thisintenR;
	static uint32_t thisintenI;

	// We have a pointer to the next data we want to analyize.
	// We have a pointer to where the tail of the new data is.
	// We want to process data from where we are to where the
	// data ends.
	// So, we set and end point, which could be either the end
	// of where the data is or the end of the array with the
	// data.
	// Then we plow through the data until we get to one of those
	// two.  And we either reset the pointer to the beginning of
	// the array, OR, we stop because we ran out of data.

	do
	{
		uint32_t cnt = DMA1_Channel1->CNTR;
		if( cnt == 0 ) cnt = 1;
		uint32_t tail_offset = DMA_SIZE - cnt;
		uint32_t * tail = dmadata + tail_offset/2; // Tuncate down to quads if a pair has not been fully written.
		uint32_t * stopat = (here < tail) ? tail : end;

		if( here == tail ) break;

		do
		{
			int32_t vA = *(here++);
			int32_t vB = *(here++);
			thisintenR += ((int16_t)vA) - (vB >> 16);
			thisintenI += ((int16_t)vB) - (vA >> 16);
		} while( here != stopat );


		if( here == end )
		{
			lastintenR = thisintenR; // Fixup (because when we were subtracting, it should be -1)
			lastintenI = thisintenI; // Fixup (because when we were subtracting, it should be -1)
			thisintenR = 0;
			thisintenI = 0;
			here = head;
			wordouts++;
		}

		if( here == tail ) break;
	}
	while( 1 );
}



int main()
{	
	SystemInit();

	Delay_Ms( 100 );

	funGpioInitAll();

	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	RCC->APB2PCENR |= RCC_APB2Periph_ADC1;

	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
	
	ADC1->RSQR1 = (1-1) << 20;	// One channel.
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = (ADCNO<<(5*0));
	
	// set sampling time for chl 7, 4, 3, 2
	// 0:7 => 3/9/15/30/43/57/73/241 cycles
	ADC1->SAMPTR2 = (0<<(3*ADCNO));

	// turn on ADC
	ADC1->CTLR2 = ADC_ADON;
	
	// Reset calibration
	ADC1->CTLR2 = ADC_ADON | ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	
	// Calibrate
	ADC1->CTLR2 = ADC_ADON | ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);
	
	// Turn on DMA
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;
	
	//DMA1_Channel1 is for ADC
	DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
	DMA1_Channel1->MADDR = (uint32_t)dmadata;
	DMA1_Channel1->CNTR  = DMA_SIZE;
	DMA1_Channel1->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_VeryHigh |
		DMA_MemoryDataSize_HalfWord |
		DMA_PeripheralDataSize_HalfWord |
		DMA_MemoryInc_Enable |
		DMA_Mode_Circular |
		DMA_DIR_PeripheralSRC |
		DMA_IT_TC | DMA_IT_HT; // Transmission Complete + Half Empty Interrupts. 

//	NVIC_SetPriority( DMA1_Channel3_IRQn, 0<<4 ); //We don't need to tweak priority.
	NVIC_EnableIRQ( DMA1_Channel1_IRQn );
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN; // Turn on DMA channel 1

	ADC1->CTLR1 |= ADC_SCAN; 	// enable scanning
	// Enable continuous conversion and DMA, selected by TIM2CC1
	ADC1->CTLR2 = ADC_ADON | ADC_DMA | ADC_EXTSEL_2 | ADC_EXTTRIG;

	// start conversion
	ADC1->CTLR2 |= ADC_SWSTART;

	// PD4 = T2C1
	funPinMode( LEDPIN, GPIO_CFGLR_OUT_50Mhz_PP );
	funPinMode( PD4, GPIO_CFGLR_OUT_50Mhz_AF_PP );
	funPinMode( ADC_PIN, GPIO_CFGLR_IN_ANALOG );

	// Reset TIM2 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	TIM2->PSC = 0x0000; // Prescalar
	TIM2->ATRLR = 47; // TIM2 max before reset.
	TIM2->CHCTLR1 = TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE;
	TIM2->CTLR1 = TIM_ARPE;
	TIM2->CCER = TIM_CC1E | TIM_CC1P | TIM_CC1NP;
	TIM2->SWEVGR = TIM_UG;

	// Enable TIM2 (T2C1 = Trigger ADC)
	TIM2->CTLR1 |= TIM_CEN;
	TIM2->CH1CVR = 1;

	while(1)
	{
		printf( "%5d %5d %6d\n", (int)lastintenR, (int)lastintenI, (int)wordouts );
	}
}
