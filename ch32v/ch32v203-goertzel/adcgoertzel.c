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

/* More notes

 * Minimum sample time with DMA = fCPU / 28 (5.14MHz)

*/


#include "ch32v003fun.h"
#include <stdio.h>
#include <math.h>

#define SH1107_128x128

#include "ssd1306_i2c.h"
#include "ssd1306.h"


#define PWM_PERIOD (31-1) //For 27.0MHz, use 36MHz if quadrature -- It appears to be good for *244 in the table?  WHY 26MHz???!?!!?

#define ADC_BUFFSIZE 512

#define GOERTZEL_BUFFER 16384

volatile uint16_t adc_buffer[ADC_BUFFSIZE];

//const int32_t g_goertzel_omega_per_sample = 151198; // 51/128 * 3.14159 * 65536 * 2
//const int32_t g_goertzel_coefficient   = -88021;//2 * cos( g_goertzel_omega_per_sample / 65536 * 180 / 3.141592) * 65536;
//const int32_t g_goertzel_coefficient_s = 97118;//2 * sin( g_goertzel_omega_per_sample / 65536 * 180 / 3.141592 ) * 65536;
const int32_t g_goertzel_omega_per_sample = 1238618695; // 47/256 -> 27.01920 MHz
const int32_t g_goertzel_coefficient = 870249096;
const int32_t g_goertzel_coefficient_s = 1963250500;


#define LOG_GOERTZEL_LIST 256
int32_t gertzellogs[LOG_GOERTZEL_LIST*2];
int     gertzellogs_head;

void SetupADC()
{
	// XXX TODO -look into PGA
	// XXX TODO - Look into tag-teaming the ADCs

	// PDA is analog input chl 7
	GPIOA->CFGLR &= ~(0xf<<(4*7));	// CNF = 00: Analog, MODE = 00: Input
	
	// ADC CLK is chained off of APB2.

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
	ADC1->CTLR1 = /*ADC_Pga_64 | */ADC_SCAN;


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

//	NVIC_SetPriority( DMA1_Channel1_IRQn, 0<<4 ); //We don't need to tweak priority.
	NVIC_EnableIRQ( DMA1_Channel1_IRQn );
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN | DMA_IT_TC | DMA_IT_HT; // Transmission Complete + Half Empty Interrupts. 




	// Turn on DMA channel 1
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN;
	
	// Enable continuous conversion and DMA
	ADC1->CTLR2 |= ADC_DMA; // | ADC_CONT;

	// start conversion
	ADC1->CTLR2 |= ADC_SWSTART;// | ADC_CONT;

}

static void SetupTimer1()
{
	// Enable Timer 1
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	TIM1->PSC = 0;  // Prescalar to 0x0000 (so, 48MHz base clock)
	TIM1->ATRLR = PWM_PERIOD;

#ifdef PWM_OUTPUT
	// PA10 = T1CH3.
	GPIOA->CFGHR &= ~(0xf<<(4*2));
	GPIOA->CFGHR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*2);

	TIM1->CCER = TIM_CC3E | TIM_CC3P;
	TIM1->CHCTLR2 = TIM_OC3M_2 | TIM_OC3M_1;
	TIM1->CH3CVR = 5;  // Actual duty cycle (Off to begin with)
#endif

	TIM1->CCER = TIM_CC1E;
	TIM1->CHCTLR1 = TIM_OC1M_2 | TIM_OC1M_1;
	TIM1->CH1CVR = 1;

	// Setup TRGO to trigger for ADC (NOTE: Not on the 203! TIM1_TRGO is only connected to injection)
	//TIM1->CTLR2 = TIM_MMS_1;

	// Enable TIM1 outputs
	TIM1->BDTR = TIM_MOE;
	TIM1->CTLR1 = TIM_CEN;
}


void InnerLoop() __attribute__((noreturn));


uint32_t tc;


volatile uint16_t * adc_tail = adc_buffer;


uint32_t g_goertzel_samples;
uint32_t g_goertzel_outs;
int32_t g_goertzel, g_goertzelp, g_goertzelp2;
int32_t g_goertzelp_store, g_goertzelp2_store;

void DMA1_Channel1_IRQHandler( void ) __attribute__((interrupt));
void DMA1_Channel1_IRQHandler( void ) 
{
	//GPIOD->BSHR = 1;	 // Turn on GPIOD0 for profiling

	// Timer goes backwards when we are moving forwards.
	volatile uint16_t * adc_buffer_end = 0;
	volatile uint16_t * adc_buffer_top = adc_buffer + ADC_BUFFSIZE;

	int32_t goertzel_coefficient = g_goertzel_coefficient;
	int32_t goertzelp2 = g_goertzelp2;
	int32_t goertzelp = g_goertzelp;
	int32_t goertzel = g_goertzel;

	uint32_t goertzel_samples = g_goertzel_samples;
	// Backup flags.
	volatile int intfr = DMA1->INTFR;
	do
	{
		// Clear all possible flags.
		DMA1->INTFCR = DMA1_IT_GL1;

		int tpl = ADC_BUFFSIZE - DMA1_Channel1->CNTR; // Warning, sometimes this is == to the base, or == 0 (i.e. might be 256, if top is 255)
		tpl += ADC_BUFFSIZE;
		tpl = (tpl & (ADC_BUFFSIZE-1));
		if( tpl == ADC_BUFFSIZE ) tpl = 0;

		adc_buffer_end = adc_buffer + ( ( tpl / 4) * 4 );

		#define INFADC 2
		// Add a tiny bias to the ADC to help keep goertz in range.
		const int adc_offset = (-2048) << INFADC;


		while( adc_tail != adc_buffer_end )
		{

			uint32_t t; // 1/2 of 4096, to try to keep our numbers reasonable.

			// Here is where the magic happens.

#if 1
			#define XSTR(x) #x
			#define GOERTZELLOOP(idx)  \
			asm volatile("\n\
				lhu     %[adcin]," XSTR(idx) "(%[adc_tail])	\n\
				slli    %[adcin],%[adcin],%[iadc]			/*INFADC = 2*/ \n\
				add		%[adcin],%[adcin],%[adcoffset]		/*adcin += adcoffset*/ \n\
				addi    %[goertzelp2],%[goertzelp],0		/*goertzelp2 = goertzelp*/ \n\
				addi    %[goertzelp], %[goertzel],0			/*goertzelp = goertzel*/ \n\
				slli    %[goertzel], %[goertzelp], 2		/*prescaling up goertzelp*/\n\
				mulh    %[goertzel], %[goertzel_coefficient], %[goertzel]\n\
				sub     %[adcin],%[adcin],%[goertzelp2]		/*adcin -= goertzelp2*/ \n\
				add     %[goertzel], %[goertzel], %[adcin]	/* mulh = signed * signed + adc */ \n"\
			: [goertzel]"+r"(goertzel), [goertzelp]"+r"(goertzelp), [goertzelp2]"+r"(goertzelp2), [adcin]"+r"(t) : \
				[adc_tail]"r"(adc_tail), [adcoffset]"r"(adc_offset), [goertzel_coefficient]"r"(goertzel_coefficient), [iadc]"i"(INFADC) );

			GOERTZELLOOP(0);
			GOERTZELLOOP(2);
			GOERTZELLOOP(4);
			GOERTZELLOOP(6);
#else
			t = ((adc_tail[0])<<INFADC)+adc_offset;
				goertzelp2 = goertzelp;
				goertzelp = goertzel;
				goertzel = t + ( ( (((int32_t)(goertzel_coefficient))) * ((((int64_t)goertzelp)<<2)) ) >> 32 ) - goertzelp2;

			t = ((adc_tail[1])<<INFADC)+adc_offset;
				goertzelp2 = goertzelp;
				goertzelp = goertzel;
				goertzel = t + ( ( (((int32_t)(goertzel_coefficient))) * ((((int64_t)goertzelp)<<2)) ) >> 32 ) - goertzelp2;

			t = ((adc_tail[2])<<INFADC)+adc_offset;
				goertzelp2 = goertzelp;
				goertzelp = goertzel;
				goertzel = t + ( ( (((int32_t)(goertzel_coefficient))) * ((((int64_t)goertzelp)<<2)) ) >> 32 ) - goertzelp2;

			t = ((adc_tail[3])<<INFADC)+adc_offset;
				goertzelp2 = goertzelp;
				goertzelp = goertzel;
				goertzel = t + ( ( (((int32_t)(goertzel_coefficient))) * ((((int64_t)goertzelp)<<2)) ) >> 32 ) - goertzelp2;
#endif

			adc_tail+=4;
			goertzel_samples+=4;
			if( adc_tail == adc_buffer_top ) adc_tail = adc_buffer;
			if( goertzel_samples == GOERTZEL_BUFFER )
			{
				g_goertzelp_store = goertzel - (g_goertzel_omega_per_sample>>(29-16));
				g_goertzelp2_store = goertzelp;

				gertzellogs[gertzellogs_head++] = g_goertzelp_store;
				gertzellogs[gertzellogs_head++] = g_goertzelp2_store;
				gertzellogs_head = gertzellogs_head & ((LOG_GOERTZEL_LIST*2)-1);

				g_goertzel_outs++;
				goertzel = g_goertzel_omega_per_sample>>(29-16);
				goertzelp = 0;
				goertzel_samples = 0;
			}
		}


		intfr = DMA1->INTFR;
	} while( intfr & DMA1_IT_GL1 );

	g_goertzelp2 = goertzelp2;
	g_goertzelp = goertzelp;
	g_goertzel = goertzel;
	g_goertzel_samples = goertzel_samples;

	//GPIOD->BSHR = 1<<16; // Turn off GPIOD0 for profiling
}



void InnerLoop()
{
	int intensity_max = 1;

	while(1){

		int k;
#if 0
		int adcz = adc_buffer[0];
		for( k = 0; k < 128; k++ )
		{
			int y = adc_buffer[k]-adcz + 64;
			if( y < 0 ) y = 0;
			if( y > 127 ) y = 127;
			ssd1306_drawPixel( k, y, 1 );
		}
#endif

		int pxa = 0;

		// Only display half of the list so the other half could
		// be updated by the ISR.
		int glread = gertzellogs_head+LOG_GOERTZEL_LIST*2/2;

		

		for( pxa = 0; pxa < LOG_GOERTZEL_LIST/2; pxa++ )
		{
			glread = (glread)&(LOG_GOERTZEL_LIST*2-1);
			int32_t zp = gertzellogs[glread++];
			int32_t zp2 = gertzellogs[glread++];
			int32_t rr = (((int64_t)(g_goertzel_coefficient  ) * (int64_t)zp<<1)>>32) - (zp2);
			int32_t ri = (((int64_t)(g_goertzel_coefficient_s) * (int64_t)zp<<1)>>32);

			rr>>=4;
			ri>>=4;

			int s = rr * rr + ri * ri;
			int intensity = 1<<( ( 32 - __builtin_clz(s) )/2);
			intensity = (intensity + s/intensity)/2;
			intensity = (intensity + s/intensity)/2;
			if( intensity > intensity_max ) intensity_max = intensity;

			rr = rr * 64 / intensity_max;
			ri = ri * 64 / intensity_max;

			rr += 64;
			ri += 64;

			if( rr < 0 ) rr = 0;
			if( ri < 0 ) ri = 0;
			if( rr > 127 ) rr = 127;
			if( ri > 127 ) ri = 127;
			
			ssd1306_drawPixel( rr, ri, 1 );
		}

		intensity_max = intensity_max - (intensity_max>>4);

		ssd1306_refresh();
		ssd1306_setbuf(0);

		int32_t rr = (((int64_t)(g_goertzel_coefficient  ) * (int64_t)g_goertzelp_store<<1)>>32) - (g_goertzelp2_store); \
		int32_t ri = (((int64_t)(g_goertzel_coefficient_s) * (int64_t)g_goertzelp_store<<1)>>32); \

		rr>>=4;
		ri>>=4;
		int s = rr * rr + ri * ri;
		int x = 1<<( ( 32 - __builtin_clz(s) )/2);
		x = (x + s/x)/2;
		x = (x + s/x)/2;

		char cts[32];
		snprintf( cts, 32, "%6d", x );
		ssd1306_drawstr( 0, 0, cts, 1 );
		
//		printf( "%6d %8d %8d - %8d %8d - %8d\n", g_goertzel_outs,g_goertzelp2_store, g_goertzelp_store, rr, ri, x );

//		Delay_Ms(940);
	}

}

int main()
{
	SystemInit();

	SysTick->CTLR = (1<<2) | 1; // HCLK
	Delay_Ms(100);


	printf( "System On\n" );

	RCC->CTLR |= RCC_HSEON;
	while( ! ( RCC->CTLR & RCC_HSERDY ) );

	RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_HSE;

	RCC->CTLR &= ~RCC_PLLON;

	// Switch PLL to HSE.
	RCC->CFGR0 |= RCC_PLLSRC;

	// x18; 8MHz x 18 = 144 MHz
	RCC->CFGR0 &= ~RCC_PPRE2; // No divisor on APB1/2
	RCC->CFGR0 &= ~RCC_PPRE1;
	RCC->CFGR0 |= RCC_PLLMULL_0 | RCC_PLLMULL_1 | RCC_PLLMULL_2 | RCC_PLLMULL_3;

	RCC->CTLR |= RCC_PLLON;

	// Switch to PLL
	RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_PLL;

	// Disable HSI
	RCC->CTLR &= ~(RCC_HSION);

	Delay_Ms(10);


	//printf( "CTLR: %08x / CFGR0: %08x\n", (RCC->CTLR), (RCC->CFGR0) );
	RCC->AHBPCENR |= 3; //DMA2EN | DMA1EN
	RCC->APB2PCENR |= RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | 0x07; // Enable all GPIO
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	SetupADC();

	printf( "Setting up OLED.\n" );
	ssd1306_i2c_setup();
	uint8_t ret = ssd1306_i2c_init();
	ssd1306_init();
	ssd1306_setbuf(0);

#if 0
	int i = 0;
	int k = 0;
	int frame = 0;
	while( 1)
	{
//		ssd1306_drawLine( (frame)%128, (0)%128, (0)%128, (127-frame)%128, 1 );
		ssd1306_drawstr( frame%128, frame%128, "hello", 1 );

		ssd1306_refresh();
		ssd1306_setbuf(0);
		frame++;
	}

	while(1);
#endif

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
