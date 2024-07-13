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
#include <stdlib.h>
#include <math.h>

#define SH1107_128x128
#define SSD1306_REMAP_I2C
//#define PWM_OUTPUT
#define ENABLE_OLED
#define PROFILING_PIN PA0

#include "ssd1306_i2c.h"
#include "ssd1306.h"
#include "./usb_config.h"
#include "../ch32v003fun/examples_v20x/otg_device/otgusb.h"

// Bigger buffer decreases chance of fall-through, but increases the size of each operation.
#define ADC_BUFFSIZE 512

volatile uint16_t adc_buffer[ADC_BUFFSIZE];

int g_volume_pwm = 127; // 0 - 127 (100%) (but you can go over 100)

#if 0
int g_pwm_period = (30-1);
int g_goertzel_buffer = (752);
int g_exactcompute = (0);
int32_t g_goertzel_omega_per_sample = 2485087396; // 0.368351 of whole per step / 27.031915MHz
int32_t g_goertzel_coefficient = -1453756170;
int32_t g_goertzel_coefficient_s = 1580594514;
#endif

#if 1
int g_pwm_period = (30-1);
int g_goertzel_buffer = (180);
int g_exactcompute = (0);
int32_t g_goertzel_omega_per_sample = 5509657063; // 0.816667 of whole per step / 0.880000MHz
int32_t g_goertzel_coefficient = 873460290;
int32_t g_goertzel_coefficient_s = -1961823932;
#endif

#if 0
int g_pwm_period = (31-1);
int g_goertzel_buffer = (412);
int g_exactcompute = (0);
const int32_t g_goertzel_omega_per_sample = 1670254667; // 0.247573 of whole per step / 1.150016MHz
const int32_t g_goertzel_coefficient = 32748822;
const int32_t g_goertzel_coefficient_s = 2147233926;
#endif

#if 0
int g_pwm_period = (30-1);
int g_goertzel_buffer = (576);
int g_exactcompute = (0);
int32_t g_goertzel_omega_per_sample = 1264972285; // 0.187500 of whole per step / 90.300000MHz
int32_t g_goertzel_coefficient = 821806413;
int32_t g_goertzel_coefficient_s = 1984016189;
#endif

#if 0
int g_pwm_period = (30-1);
int g_goertzel_buffer = (320);
int g_exactcompute = (0);
const int32_t g_goertzel_omega_per_sample = 990894956; // 0.146875 of whole per step / 101.505000MHz
const int32_t g_goertzel_coefficient = 1296126516;
const int32_t g_goertzel_coefficient_s = 1712233066;
#endif

#if 0
int g_pwm_period = (30-1);
int g_goertzel_buffer = (384);
int g_exactcompute = (0);
const int32_t g_goertzel_omega_per_sample = 4251712402; // 0.630208 of whole per step / 27.025000MHz
const int32_t g_goertzel_coefficient = -1468003291;
const int32_t g_goertzel_coefficient_s = -1567371161;
#endif

#if 0
int g_pwm_period = (30-1);
int g_goertzel_buffer = (336);
int g_exactcompute = (0);
const int32_t g_goertzel_omega_per_sample = 1827182189; // 0.270833 of whole per step / 89.900000MHz
const int32_t g_goertzel_coefficient = -280302863;
const int32_t g_goertzel_coefficient_s = 2129111628;
#endif


int intensity_average = 1;

#define LOG_GOERTZEL_LIST 512
int32_t qibaselogs[LOG_GOERTZEL_LIST];
volatile int     qibaselogs_head;

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
	ADC1->CTLR1 = ADC_Pga_64 | ADC_SCAN;


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
	TIM1->ATRLR = g_pwm_period;

#ifdef PWM_OUTPUT
	// PA9 = T1CH2.
	funPinMode( PA9, GPIO_CFGLR_OUT_2Mhz_AF_PP );

	TIM1->CCER = TIM_CC2E | TIM_CC2P;
	TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2FE;
	TIM1->CH2CVR = 5;  // Set duty cycle somewhere random.

	// Enable TIM1 outputs
	TIM1->BDTR |= 0xc000;//TIM_MOE;
#endif

	TIM1->CCER |= TIM_CC1E;
	TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;
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

int32_t g_laststart = 0;
int32_t g_lastper;
int32_t g_lastlen;
uint32_t g_accumulate_over_window;

void DMA1_Channel1_IRQHandler( void ) __attribute__((interrupt));
void DMA1_Channel1_IRQHandler( void ) 
{
	int32_t start = SysTick->CNT;
#ifdef PROFILING_PIN
	funDigitalWrite( PROFILING_PIN, 1 );
#endif

	// Timer goes backwards when we are moving forwards.
	volatile uint16_t * adc_buffer_end = 0;
	volatile uint16_t * adc_buffer_top = adc_buffer + ADC_BUFFSIZE;

	int32_t goertzel_coefficient = g_goertzel_coefficient;
	int32_t goertzelp2 = g_goertzelp2;
	int32_t goertzelp = g_goertzelp;
	int32_t goertzel = g_goertzel;
	int32_t accumulate_over_window = g_accumulate_over_window;

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
		static int adc_offset = (-2048) << INFADC;

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
				add     %[accumulate_over_window], %[adcin], %[accumulate_over_window]\n \
				addi    %[goertzelp2],%[goertzelp],0		/*goertzelp2 = goertzelp*/ \n\
				addi    %[goertzelp], %[goertzel],0			/*goertzelp = goertzel*/ \n\
				slli    %[goertzel], %[goertzelp], 2		/*prescaling up goertzelp*/\n\
				mulh    %[goertzel], %[goertzel_coefficient], %[goertzel]\n\
				sub     %[adcin],%[adcin],%[goertzelp2]		/*adcin -= goertzelp2*/ \n\
				add     %[goertzel], %[goertzel], %[adcin]	/* mulh = signed * signed + adc */ \n"\
			: [goertzel]"+r"(goertzel), [goertzelp]"+r"(goertzelp), [goertzelp2]"+r"(goertzelp2), [adcin]"+r"(t), [accumulate_over_window]"+r"(accumulate_over_window) : \
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
			if( goertzel_samples == g_goertzel_buffer )
			{
#ifdef PROFILING_PIN
	funDigitalWrite( PROFILING_PIN, 0 );
#endif

				g_goertzelp_store = goertzel;
				g_goertzelp2_store = goertzelp;

				int32_t zp = g_goertzelp_store;
				int32_t zp2 = g_goertzelp2_store;
				int32_t rr = (((int64_t)(g_goertzel_coefficient  ) * (int64_t)zp<<1)>>32) - (zp2);
				int32_t ri = (((int64_t)(g_goertzel_coefficient_s) * (int64_t)zp<<1)>>32);

				qibaselogs[qibaselogs_head] = ((uint16_t)rr) | (((uint16_t)ri)<<16);
				qibaselogs_head = ( qibaselogs_head + 1 ) & ((LOG_GOERTZEL_LIST)-1);

				rr>>=2;
				ri>>=2;

				int s = rr * rr + ri * ri;
				//int intensity = 1<<( ( 32 - __builtin_clz(s) )/2);
				#define ABS(x) (((x)<0)?-(x):(x))
				int intensity = (ABS(rr) + ABS(ri)) * 26100 / 32768; // Found experimentally (Also try to avoid divide-by-zero.
				if( intensity == 0 )
					intensity = 1;
				intensity = (intensity + s/intensity)/2;
				intensity = (intensity + s/intensity)/2;
				intensity_average = intensity_average - (intensity_average>>12) + (intensity>>6);

				#ifdef PWM_OUTPUT
				intensity = intensity * g_volume_pwm * g_pwm_period / (intensity_average>>(10-12));
				if( intensity >= g_pwm_period-1 ) intensity = g_pwm_period-2;
				if( intensity < 1 ) intensity = 1;
				TIM1->CH2CVR = intensity;  // Actual duty cycle (Off to begin with)
				#endif

				g_goertzel_outs++;
				goertzel = 0;
				goertzelp = 0;
				goertzel_samples = 0;

				// Try to improve bias.
				adc_offset -= accumulate_over_window / g_goertzel_buffer;
				accumulate_over_window = 0;

#ifdef PROFILING_PIN
	funDigitalWrite( PROFILING_PIN, 1 );
#endif

			}
		}

		intfr = DMA1->INTFR;
	} while( intfr & DMA1_IT_GL1 );

	g_goertzelp2 = goertzelp2;
	g_goertzelp = goertzelp;
	g_goertzel = goertzel;
	g_goertzel_samples = goertzel_samples;
	g_accumulate_over_window = accumulate_over_window;

#ifdef PROFILING_PIN
	funDigitalWrite( PROFILING_PIN, 0 ); // For profiling
#endif
	int32_t end = SysTick->CNT;
	g_lastper = start - g_laststart;
	g_laststart = start;
	g_lastlen = end - start;
}

static inline uint32_t gets2()
{
	uint32_t ret;
	asm volatile( "mv %[ret], s2" : [ret]"=&r"(ret) );
	return ret;
}


void InnerLoop()
{
	while(1){
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
		int glread = qibaselogs_head;

		int intensity = 0;

		for( pxa = 0; pxa < LOG_GOERTZEL_LIST; pxa++ )
		{
			uint32_t combiq = qibaselogs[glread];
			glread = ( glread + 1 ) & ( LOG_GOERTZEL_LIST -1 );

			int16_t rr = combiq & 0xffff;
			int16_t ri = combiq >> 16;

			rr = rr * 512 / (intensity_average);
			ri = ri * 512 / (intensity_average);

			rr += 64;
			ri += 64;

			if( rr < 0 ) rr = 0;
			if( ri < 0 ) ri = 0;
			if( rr > 127 ) rr = 127;
			if( ri > 127 ) ri = 127;
			
#ifdef ENABLE_OLED
			ssd1306_drawPixel( rr, ri, 1 );
#endif
		}

#ifdef ENABLE_OLED
		//char cts[32];
		//snprintf( cts, 32, "%d", intensity_average );
		//ssd1306_drawstr( 0, 0, cts, 1 );

		ssd1306_refresh();
		//static int ik = 0;
		//printf( "%d %08x\n", ik, ssd1306_buffer[ik++] );
		//if( ik == sizeof(ssd1306_buffer) ) ik = 0;

		ssd1306_setbuf(0);

#else
		Delay_Ms(17);
#endif
		
//		printf( "%6d %8d %8d - %8d %8d - %8d\n", g_goertzel_outs,g_goertzelp2_store, g_goertzelp_store, rr, ri, x );

//		Delay_Ms(940);
//printf( "!!!!\n ");

	}

}

int main()
{
	SystemInit();

	SysTick->CTLR = (1<<2) | 1; // HCLK
	Delay_Ms(100);

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
	RCC->APB2PCENR |= RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | 0x07 | RCC_APB2Periph_GPIOA; // Enable all GPIO
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	SetupADC();

#ifdef ENABLE_OLED
	ssd1306_i2c_setup();
	ssd1306_i2c_init();

	if( ssd1306_init() )
		printf( "Failed to initialize OLED\n" );
	else
		printf( "Initialized OLED\n" );

	ssd1306_setbuf(1);
	ssd1306_refresh();
#endif

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

#ifdef PROFILING_PIN
	funPinMode( PROFILING_PIN, GPIO_CFGLR_OUT_50Mhz_PP );
#endif

	SetupTimer1();

	USBOTGSetup();


	InnerLoop();
}





uint8_t scratchpad[512];
int g_isConfigurePacket = 0;

int HandleHidUserSetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	int id = req->wValue & 0xff;
	g_isConfigurePacket = 0;
	if( id == 0xaa && req->wLength <= sizeof( scratchpad ) )
	{
		ctx->pCtrlPayloadPtr = scratchpad;
		return req->wLength;
	}
	else if( id == 0xac && req->wLength <= sizeof( scratchpad ) )
	{
		g_isConfigurePacket = 1;
		ctx->pCtrlPayloadPtr = scratchpad;
		return req->wLength;
	}
	return 0;
}

int HandleHidUserGetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	int id = req->wValue & 0xff;
	if( id == 0xaa )
	{
		ctx->pCtrlPayloadPtr = scratchpad;
		return 255;
	}
	else if( id == 0xac )
	{
		g_isConfigurePacket = 1;
		ctx->pCtrlPayloadPtr = scratchpad;
		return 63;
	}
	else if( id == 0xad )
	{
		static int last_baselog;

		int samps_to_send = (qibaselogs_head - last_baselog + LOG_GOERTZEL_LIST * 2 - 1) & (LOG_GOERTZEL_LIST-1);
		if( samps_to_send > 120 ) samps_to_send = 120;

		((uint32_t*)scratchpad)[0] = (intensity_average<<12) | samps_to_send;
		((uint32_t*)scratchpad)[1] = (g_lastper<<16) | g_lastlen;
		((uint32_t*)scratchpad)[2] = (0<<16) | (((g_pwm_period+1)*g_goertzel_buffer)); //LSW = 144MHz / X

		int i;
		for( i = 3; i < samps_to_send + 3; i++ )
		{
			last_baselog = (last_baselog+1)&(LOG_GOERTZEL_LIST-1);
			((uint32_t*)(scratchpad))[i] = ((int32_t*)qibaselogs)[last_baselog];
		}

		for( ; i < 128; i++ )
			((uint32_t*)(scratchpad))[i]  = 0;
			

		ctx->pCtrlPayloadPtr = scratchpad;
		return 510;
	}
	return 0;
}

void HandleHidUserReportDataOut( struct _USBState * ctx, uint8_t * data, int len )
{
}

int HandleHidUserReportDataIn( struct _USBState * ctx, uint8_t * data, int len )
{
//	printf( "IN %d %d %08x %08x\n", len, ctx->USBFS_SetupReqLen, data, FSUSBCTX.ENDPOINTS[0] );
//	memset( data, 0xcc, len );
	return len;
}

void HandleHidUserReportOutComplete( struct _USBState * ctx )
{
	if( g_isConfigurePacket )
	{

		uint32_t * configs = (uint32_t*)scratchpad;
		// Note: configs[0] == 0xac (command type)

		printf( "Is Configure Packet %08x\n", configs[1] );

		int numconfigs = configs[1];
		if( numconfigs > 0) g_pwm_period = configs[2];
		if( numconfigs > 1) g_goertzel_buffer = configs[3];
		if( numconfigs > 2) g_goertzel_omega_per_sample = configs[4]; // 0.816667 of whole per step / 0.880000MHz
		if( numconfigs > 3) g_goertzel_coefficient = configs[5];
		if( numconfigs > 4) g_goertzel_coefficient_s = configs[6];
		if( numconfigs > 5) g_exactcompute = configs[7];

		// Need to reset so we don't blast by.
		g_goertzel_samples = 0;
		TIM1->ATRLR = g_pwm_period;

		g_isConfigurePacket = 0;
	}
	return;
}

