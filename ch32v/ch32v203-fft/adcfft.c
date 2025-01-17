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

#define SH1107_128x128
#define SSD1306_RST_PIN  PA3
#define SSD1306_CS_PIN   PA4
#define SSD1306_DC_PIN   PA6
#define SSD1306_MOSI_PIN PA7
#define SSD1306_SCK_PIN  PA5
#define SSD1306_BAUD_RATE_PRESCALER SPI_BaudRatePrescaler_4
#include "ssd1306_spi.h"
#include "ssd1306.h"

#define FIX_FFT_IMPLEMENTATION
#include "fix_fft.h"

/* General note:
*/

#define Q 256

#define TARGET_BIN 51

#define PWM_PERIOD (31-1) //For 27.0MHz, use 36MHz if quadrature -- It appears to be good for *244 in the table?  WHY 26MHz???!?!!?

#define ADC_BUFFSIZE 512
volatile uint16_t adc_buffer[ADC_BUFFSIZE];

void SetupADC()
{
	// XXX TODO -look into PGA
	// XXX TODO - Look into tag-teaming the ADCs

	// PA7 is analog input chl 7
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


void InnerLoop()
{
	//int i = 0;
	//int q = 0;
	int tpl = 0;

	// Timer goes backwards when we are moving forwards.
	volatile uint16_t * adc_buffer_end = 0;
	volatile uint16_t * adc_buffer_top = adc_buffer + ADC_BUFFSIZE;
	volatile uint16_t * adc = adc_buffer;

	int frcnt = 0;

	int tstart = 0;

	int16_t shadowbuff[Q+16];

	int shadowplace = 0;
	#define SCALEUP 6
	#define SHADOWSTORE(X) shadowbuff[frcnt+X] = t;

	while( 1 )
	{
		tpl = ADC_BUFFSIZE - DMA1_Channel1->CNTR; // Warning, sometimes this is == to the base, or == 0 (i.e. might be 256, if top is 255)

		tpl += ADC_BUFFSIZE;
		tpl = (tpl & (ADC_BUFFSIZE-1));
		if( tpl == ADC_BUFFSIZE ) tpl = 0;

		adc_buffer_end = adc_buffer + ( ( tpl / 4) * 4 );
//printf( "%3d %4d %d %04x\n", DMA1_Channel1->CNTR, TIM1->CNT, ADC1->RDATAR, ADC1->STATR );
		while( adc != adc_buffer_end )
		{

			int32_t t = adc[0]; SHADOWSTORE(0);
			//i += t; q += t;
			t = adc[1]; SHADOWSTORE(1);
			//i -= t; q += t;
			t = adc[2]; SHADOWSTORE(2);
			//i -= t; q -= t;
			t = adc[3]; SHADOWSTORE(3);
			//i += t; q -= t;
			adc += 4;
			frcnt += 4;

			if( adc == adc_buffer_top ) adc = adc_buffer;
			if( frcnt >= Q ) break;
		}


		if( frcnt >= Q )
		{

#if 0

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
			int is = i>>2;
#endif
		    int s = 1<<( ( 32 - __builtin_clz(is) )/2);
    		s = (s + is/s)/2;


#ifdef TIGHT_OUT
			printf( "%d\n", s );
#elif defined( PWM_OUTPUT )
			int tv = (s>>PWM_OUTPUT) + (PWM_PERIOD/2);
			if( tv < 0 ) tv = 0;
			if( tv >= PWM_PERIOD ) tv = PWM_PERIOD-1;
			TIM1->CH3CVR = tv;
#else

			printf( "%8d I:%7d Q:%7d [%d %d %d %d] / %d\n",s, i ,q, adc_buffer[0], adc_buffer[1], adc_buffer[2], adc_buffer[3], (int)(SysTick->CNT - tstart) );
#endif
			//printf( "%d\n", s );
#endif
			int k;

	//		printf( "Data: " );
			for( k = 0; k < Q; k++ )
			{
		//		printf( "%d, ",shadowbuff[k] );
				shadowbuff[k] = (shadowbuff[k]<<SCALEUP)-32768;
			}
			//printf( "\n" );

			int osb = shadowbuff[0];
			int16_t FSQ[Q] = { 0 };
			for( k = 0; k < 128; k++ )
			{
				ssd1306_drawPixel( k, ((shadowbuff[k+128]-osb)>>(SCALEUP+1))+40, 1 );
			}

			int16_t imag[Q] = { 0 };

			int r = 
				fix_fft(shadowbuff, imag, 8, 0);

					//fix_fftr(shadowbuff, 7 /*1<<7 = 128 bins wide*/, 0);
					//fix_fft(shadowbuff, FSQ, 8 /*1<<7 = 128 bins wide*/, 0);

//			printf( "FFT: 
//			for( k = 0; k < 128; k++ )
//			{
//			}

			int targbin = TARGET_BIN;
			int targv = 0;
			int maxbin = 0;
			int maxbinv = 0;


			for( k = 0; k < 128; k++ )
			{
/*
				int s = shadowbuff[k] * shadowbuff[k] + FSQ[k]*FSQ[k];
				//if( s == 0 ) continue;
				int x = 1<<( ( 32 - __builtin_clz(s) )/2);
				x = (x + i/x)/2;
				x = (x + i/x)/2; //Not really needed.
*/

				// for real
				//	int x = shadowbuff[(k>>1) | ((k&64)>>6)];
				// For faked imag

#if 0
				int s = shadowbuff[k] * shadowbuff[k] + shadowbuff[255-k]*shadowbuff[255-k];
				//if( s == 0 ) continue;
				int x = 1<<( ( 32 - __builtin_clz(s) )/2);
				x = (x + s/x)/2;
				x = (x + s/x)/2; //Not really needed.
				x = (x + s/x)/2; //Not really needed.
				
				//x = shadowbuff[ k ];
				//x = s >> 8;

				x = x >> 2;
#endif

				int s = shadowbuff[k] * shadowbuff[k] + imag[k]*imag[k];
				//if( s == 0 ) continue;
				int x = 1<<( ( 32 - __builtin_clz(s) )/2);
				x = (x + s/x)/2;
				x = (x + s/x)/2; //Not really needed.
				x = (x + s/x)/2; //Not really needed.
				
				//x = shadowbuff[ k ];
				//x = s >> 8;

				//x = x >> 2;
				if( x > maxbinv && k != 0) { maxbinv = x; maxbin = k; }
				if( k == targbin ) targv = x;

				x++;
				if( x < 0 ) x = 0;
				if( x > 127 ) x = 127;
				if( x != 0 )
					ssd1306_drawFastVLine( k, 127-x, x, 1 );

			}

			static int tbhist[128];
			static int tbhead = 0;
			tbhist[tbhead++] = targv;
			if( tbhead == 128 ) tbhead = 0;

			char cts[32];
			snprintf( cts, sizeof(cts), "%5d%5d@%d", osb, targv, targbin );
			ssd1306_drawstr( 0, 0, cts, 1 );
			snprintf( cts, sizeof(cts), "P:%d B%3d/%4d", PWM_PERIOD, maxbin, maxbinv );
			ssd1306_drawstr( 0, 8, cts, 1 );

			for( k = 0; k < 128; k++ )
			{
				ssd1306_drawPixel( k, 105-tbhist[(tbhead-k+256)%128]/2, 1);	
			}

			memset( shadowbuff, 0, sizeof( shadowbuff ) );

			ssd1306_refresh();
			ssd1306_setbuf(0);

			frcnt = 0;
//			i = 0;
//			q = 0;
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


	printf( "CTLR: %08x / CFGR0: %08x\n", (RCC->CTLR), (RCC->CFGR0) );
	RCC->AHBPCENR |= 3; //DMA2EN | DMA1EN
	RCC->APB2PCENR |= RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | 0x07; // Enable all GPIO
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	SetupADC();

	printf( "Setting up OLED.\n" );
	//ssd1306_spi_setup();
	uint8_t ret = ssd1306_spi_init();
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
