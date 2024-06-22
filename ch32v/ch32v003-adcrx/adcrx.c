// XXX TODO: Play with high bits of ADC control to see if there's a gain cicuit.

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

// SETUP INSTRUCTIONS:
//   (1) `make` in the optionbytes folder to configure `RESET` correctly.
//   (2) Create a tone (if using the funprog, ../ch32v003fun/minichlink/minichlink -X ECLK 1:235:189:9:3 for 27.48387097MHz
//   (2) or, for 24.387096762MHz -  ../ch32v003fun/minichlink/minichlink -X ECLK 1:150:49:8:3




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

	Perform Quadrature Decoding
	  I  =  + + - -
	  Q  =  + - - +

	We want the target waveform to be exactly n * Fs - fs / 4 = FBrd
	for a natural value of n.
	 27 / (n-1/4) = Fs

	Oddly enough, after creating a table with various values of N and
	possible divisors, a PERFECT divisor works out to the "max speed" listed above (1.714MHz)
	but that makes me scared at this juncture.  What if we intentionally just target 1.5MHz?
     1.5 = freq / (n-1/4)
    arbitrarily select n to be 17 (for the 17th harmonic)
     
	TODO: Is it supposed to be n-1/4 or n+1/4 or does it not matter?


	We will be targeting 48/35 MHz - (with PWM_PERIOD 34)
	Calculated to use the 19.75th harmonic @ 27.08571429MHz, but ideal found at 27.08643MHz
*/

#define Q 1000

#define PWM_PERIOD (28-1) //For 27.000500MHz
//#define QUADRATURE
//#define TIGHT_OUT
//#define DUMPBUFF
#define PWM_OUTPUT 7

//#define PWM_PERIOD (32-1)
//#define QUADRATURE
//#define PWM_OUTPUT 3

#define ADC_BUFFSIZE 256
volatile uint16_t adc_buffer[ADC_BUFFSIZE];

void SetupADC()
{
	
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

	// ADCCLK = 12 MHz => RCC_ADCPRE divide by 4
	RCC->CFGR0 &= ~RCC_ADCPRE;  // Clear out the bis in case they were set
	RCC->CFGR0 |= RCC_ADCPRE_DIV2; // Fastest possible (divide-by-2) NOTE: This is OUTSIDE the specified value in the datasheet.

	// Set up single conversion on chl 7
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 6;	// 0-9 for 8 ext inputs and two internals  /// 7 or 6 means one of the ADC inputs.

	// Not using injection group.

	// PD4 is analog input chl 7 + 6
	GPIOD->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input
	GPIOD->CFGLR &= ~(0xf<<(4*6));	// CNF = 00: Analog, MODE = 00: Input

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

#ifdef PWM_OUTPUT
	GPIOC->CFGLR &= ~(0xf<<(4*4));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*4);

	TIM1->CCER = TIM_CC4E | TIM_CC4P;
	TIM1->CHCTLR2 = TIM_OC4M_2 | TIM_OC4M_1;
	TIM1->CH4CVR = 5;  // Actual duty cycle (Off to begin with)
#endif

	// Setup TRGO for ADC.  This makes is to the ADC will trigger on timer
	// reset, so we trigger at the same position every time relative to the
	// FET turning on.
	TIM1->CTLR2 = TIM_MMS_1;

	// Enable TIM1 outputs
	TIM1->BDTR = TIM_MOE;
	TIM1->CTLR1 = TIM_CEN;
}


void InnerLoop() __attribute__((noreturn));


void InnerLoop()
{
	int i = 0;
	int q = 0;
	int tpl = 0;

	// Timer goes backwards when we are moving forwards.
	volatile uint16_t * adc_buffer_end = 0;
	volatile uint16_t * adc_buffer_top = adc_buffer + ADC_BUFFSIZE;
	volatile uint16_t * adc = adc_buffer;

	int frcnt = 0;

	int tstart = 0;

#ifdef DUMPBUFF
	uint16_t shadowbuff[Q+16];
	int shadowplace = 0;
	#define SHADOWSTORE(X) shadowbuff[frcnt+X] = t;
#else
	#define SHADOWSTORE(X)
#endif

	while( 1 )
	{
		tpl = ADC_BUFFSIZE - DMA1_Channel1->CNTR; // Warning, sometimes this is == to the base, or == 0 (i.e. might be 256, if top is 255)
		if( tpl == ADC_BUFFSIZE ) tpl = 0;

		adc_buffer_end = adc_buffer + ( ( tpl / 4) * 4 );
//printf( "%3d %4d %d %04x\n", DMA1_Channel1->CNTR, TIM1->CNT, ADC1->RDATAR, ADC1->STATR );
		while( adc != adc_buffer_end )
		{
#ifdef QUADRATURE
			int32_t t = adc[0]; SHADOWSTORE(0);
			i += t; q += t;
			t = adc[1]; SHADOWSTORE(1);
			i -= t; q += t;
			t = adc[2]; SHADOWSTORE(2);
			i -= t; q -= t;
			t = adc[3]; SHADOWSTORE(3);
			i += t; q -= t;
			adc += 4;
			frcnt += 4;
#else
			i = i + adc[0] - adc[1];
			adc += 2;
			frcnt += 2;
#endif

			if( adc == adc_buffer_top ) adc = adc_buffer;
			if( frcnt >= Q ) break;
		}


		if( frcnt >= Q )
		{

#ifdef DUMPBUFF
		int j;
		for( j = 0; j < Q; j++ )
			printf( "%d,%d\n", j, shadowbuff[j] );
#endif
#ifdef QUADRATURE
			int ti = i>>3;
			int tq = q>>3;
			int is = (ti*ti + tq*tq)>>8;
#else
			int is = ((i<0)?-i:i)>>2;
#endif
		    int s = 1<<( ( 32 - __builtin_clz(is) )/2);
    		s = (s + is/s)/2;

#ifdef TIGHT_OUT
			printf( "%d\n", is );
#elif defined( PWM_OUTPUT )


			int tv = (i>>PWM_OUTPUT) + (PWM_PERIOD/2);
			if( tv < 0 ) tv = 0;
			if( tv >= PWM_PERIOD ) tv = PWM_PERIOD-1;

			TIM1->CH4CVR = tv;
#else

			printf( "%8d I:%7d Q:%7d [%d %d %d %d] / %d\n",s, i ,q, adc_buffer[0], adc_buffer[1], adc_buffer[2], adc_buffer[3], (int)(SysTick->CNT - tstart) );
#endif
			//printf( "%d\n", s );
			frcnt = 0;
			i = 0;
			q = 0;
			tpl = ADC_BUFFSIZE - DMA1_Channel1->CNTR;
			adc = adc_buffer + ( ( tpl / 4) * 4 );
			tstart = SysTick->CNT;
		}
/*
		Delay_Us( 100 );
		int end = DMA1_Channel1->CNTR;
		int v0 = adc_buffer[0];
		int v1 = adc_buffer[1];
		int v2 = adc_buffer[2];
		int v3 = adc_buffer[3];
		printf( "%d %d %d %d %d\n", (uint8_t)(start-end), v0, v1, v2, v3 );
*/
	}


}

int main()
{
	// REQUIRES External 24MHz oscillator
	printf( "Initializing\n" );

	SystemInit();

	Delay_Ms(10);

	printf( "System On\n" );

	// Enable Peripherals
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC |
		RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 |
		RCC_APB2Periph_AFIO;

	RCC->APB1PCENR = RCC_APB1Periph_TIM2;

	// Disable HSI
	RCC->CTLR &= ~(RCC_HSION);

	printf( "CTLR: %08x CFGR0: %08x\n", RCC->CTLR, RCC->CFGR0 );

	SetupADC();


#if 0
	// turn on the op-amp
	EXTEN->EXTEN_CTR |= EXTEN_OPA_EN;

	// select op-amp pos pin: 0 = PA2, 1 = PD7
	EXTEN->EXTEN_CTR |= EXTEN_OPA_PSEL;

	// select op-amp neg pin: 0 = PA1, 1 = PD0
	EXTEN->EXTEN_CTR |= EXTEN_OPA_NSEL;
#endif


	printf( "ADC Setup\n" );

	SetupTimer1();

	printf( "Timer 1 setup\n" );

	InnerLoop();
}
