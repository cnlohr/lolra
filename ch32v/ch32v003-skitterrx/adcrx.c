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
#include <string.h>
#include "rv003usb.h"

uint8_t scratchout[15];
volatile int outready = 0;
uint8_t scratchin[255];
volatile int inready = 0;


#define PWM_PERIOD (28-1) //For 27.000500MHz
#define QUADRATURE

uint32_t TQ = 128;

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
	int Q = TQ;

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

			if( adc == adc_buffer_top ) adc = adc_buffer;
			if( frcnt >= Q ) break;
		}


		if( frcnt >= Q )
		{

			int ti = i>>3;
			int tq = q>>3;
			int is = (ti*ti + tq*tq)>>8;

		    int s = 1<<( ( 32 - __builtin_clz(is) )/2);
    		s = (s + is/s)/2;

			//int tv = (i>>PWM_OUTPUT) + (PWM_PERIOD/2);
			//if( tv < 0 ) tv = 0;
			//if( tv >= PWM_PERIOD ) tv = PWM_PERIOD-1;
			//TIM1->CH4CVR = tv;

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

	printf( "CTLR: %08lx CFGR0: %08lx\n", RCC->CTLR, RCC->CFGR0 );

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






void usb_handle_user_in_request( struct usb_endpoint * e, uint8_t * scratchpad, int endp, uint32_t sendtok, struct rv003usb_internal * ist )
{
	// Make sure we only deal with control messages.  Like get/set feature reports.
	if( endp )
	{
		usb_send_empty( sendtok );
	}
}

void usb_handle_user_data( struct usb_endpoint * e, int current_endpoint, uint8_t * data, int len, struct rv003usb_internal * ist )
{
	if( outready ) 
	{
		// Send NACK (can't accept any more data right now)
		usb_send_data( 0, 0, 2, 0x5A );
		return;
	}

	usb_send_data( 0, 0, 2, 0xD2 ); // Send ACK
	int offset = e->count<<3;
	int torx = e->max_len - offset;
	if( torx > len ) torx = len;
	if( torx > 0 )
	{
		memcpy( scratchout + offset, data, torx );
		e->count++;
		if( ( e->count << 3 ) >= e->max_len )
		{
			outready = e->max_len;
		}
	}
}


void usb_handle_hid_get_report_start( struct usb_endpoint * e, int reqLen, uint32_t lValueLSBIndexMSB )
{
	if( reqLen > sizeof( scratchin ) ) reqLen = sizeof( scratchin );

	// You can check the lValueLSBIndexMSB word to decide what you want to do here
	// But, whatever you point this at will be returned back to the host PC where
	// it calls hid_get_feature_report. 
	//
	// Please note, that on some systems, for this to work, your return length must
	// match the length defined in HID_REPORT_COUNT, in your HID report, in usb_config.h

	if( reqLen > inready ) inready = inready;
	e->opaque = scratchin;
	e->max_len = reqLen;
}

void usb_handle_hid_set_report_start( struct usb_endpoint * e, int reqLen, uint32_t lValueLSBIndexMSB )
{
	// Here is where you get an alert when the host PC calls hid_send_feature_report.
	//
	// You can handle the appropriate message here.  Please note that in this
	// example, the data is chunked into groups-of-8-bytes.
	//
	// Note that you may need to make this match HID_REPORT_COUNT, in your HID
	// report, in usb_config.h

	if( outready ) reqLen = 0;
	if( reqLen > sizeof( scratchout ) ) reqLen = sizeof( scratchout );
	e->opaque = scratchout;
	e->max_len = reqLen;
}


void usb_handle_other_control_message( struct usb_endpoint * e, struct usb_urb * s, struct rv003usb_internal * ist )
{
	LogUEvent( SysTick->CNT, s->wRequestTypeLSBRequestMSB, s->lValueLSBIndexMSB, s->wLength );
}










