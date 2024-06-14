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

// NOT LORA!!! -- but experimenting with the possibility of rx.

#include "ch32v003fun.h"
#include <stdio.h>

/* General note:
	Max speed was found to be:
	 ADCclk = RCC / 2
	 SAMPTR2 = 0 (3 cycles) 
     PWM period = 27 (48/28 = 1.714MHz)

	If you go a little slower...
	 ADCclk = RCC / 2
	 SAMPTR2 = 1 (9 cycles)
	 PWM period = 39 (48/40 = 1.2MHz)

	We are going to target 1.5MHz. (Period = 31), Divisor = 32
*/

#define PWM_PERIOD 31

#define ADC_BUFFSIZE 256
volatile uint16_t adc_buffer[ADC_BUFFSIZE];

void SetupADC()
{
	// PD4 is analog input chl 7
	GPIOD->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input
	
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

	// ADCCLK = 12 MHz => RCC_ADCPRE divide by 4
	RCC->CFGR0 &= ~RCC_ADCPRE;  // Clear out the bis in case they were set
	RCC->CFGR0 |= RCC_ADCPRE_DIV2; // Fastest possible (divide-by-2) NOTE: This is OUTSIDE the specified value in the datasheet.

	// Set up single conversion on chl 7
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 7;	// 0-9 for 8 ext inputs and two internals

	// Not using injection group.

	// Sampling time for channels. Careful: This has PID tuning implications.
	// Note that with 3 and 3,the full loop (and injection) runs at 138kHz.
	ADC1->SAMPTR2 = (0<<(3*7)); 

	// Turn on ADC and set rule group to sw trig
	// 0 = Use TRGO event for Timer 1 to fire ADC rule.
	ADC1->CTLR2 = ADC_ADON | ADC_EXTTRIG | ADC_DMA; 

	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);

	// Calibrate ADC
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);

	// ADC_SCAN: Allow scanning.
	ADC1->CTLR1 = ADC_SCAN;

	// Turn on DMA
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	//DMA1_Channel1 is for ADC
	DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
	DMA1_Channel1->MADDR = (uint32_t)adc_buffer;
	DMA1_Channel1->CNTR  = ADC_BUFFSIZE;
	DMA1_Channel1->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_VeryHigh |
		DMA_MemoryDataSize_HalfWord |
		DMA_PeripheralDataSize_HalfWord |
		DMA_MemoryInc_Enable |
		DMA_Mode_Circular |
		DMA_DIR_PeripheralSRC;

	// Turn on DMA channel 1
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN;
	
	// Enable continuous conversion and DMA
	//ADC1->CTLR2 |= ADC_DMA | ADC_EXTSEL; //ADC_CONT

	// start conversion
	ADC1->CTLR2 |= ADC_SWSTART;
}

static void SetupTimer1()
{
	// Enable Timer 1
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	TIM1->PSC = 0x0000;  // Prescalar to 0x0000 (so, 48MHz base clock)
	TIM1->ATRLR = PWM_PERIOD;

	// NOT USED
	//TIM1->CCER = TIM_CC2E | TIM_CC2NP;  // CH2 is control for FET.
	//TIM1->CHCTLR1 = TIM_OC2M_2 | TIM_OC2M_1;

	TIM1->CH2CVR = 0;  // Actual duty cycle (Off to begin with)

	// Setup TRGO for ADC.  This makes is to the ADC will trigger on timer
	// reset, so we trigger at the same position every time relative to the
	// FET turning on.
	TIM1->CTLR2 = TIM_MMS_1;

	// Enable TIM1 outputs
	TIM1->BDTR = TIM_MOE;
	TIM1->CTLR1 = TIM_CEN;
}


int main()
{
	// REQUIRES External 24MHz oscillator
	printf( "Initializing\n" );

	SystemInit();

	printf( "System On\n" );

	// Enable Peripherals
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC |
		RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 |
		RCC_APB2Periph_AFIO;

	RCC->APB1PCENR = RCC_APB1Periph_TIM2;

	SetupADC();

	printf( "ADC Setup\n" );

	SetupTimer1();

	printf( "Timer 1 setup\n" );

	while( 1 )
	{
		int start = DMA1_Channel1->CNTR;
		Delay_Us( 100 );
		int end = DMA1_Channel1->CNTR;
		int v0 = adc_buffer[0];
		int v1 = adc_buffer[1];
		int v2 = adc_buffer[2];
		int v3 = adc_buffer[3];
		printf( "%d %d %d %d %d\n", (uint8_t)(start-end), v0, v1, v2, v3 );
	}
}
