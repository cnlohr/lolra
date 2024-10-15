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

// SETUP INSTRUCTIONS:
//   (1) `make` in the optionbytes folder to configure `RESET` correctly.
//   (2) Create a tone (if using the funprog, ../ch32v003fun/minichlink/minichlink -X ECLK 1:235:189:9:3 for 27.48387097MHz
//   (2) or, for 24.387096762MHz -  ../ch32v003fun/minichlink/minichlink -X ECLK 1:150:49:8:3
/* More notes
 * Minimum sample time with DMA = fCPU / 28 (5.14MHz)
*/

// TODO:
// 1: Cleanup some code.
// 2: Leverage other ADC.
// 3: 


#include "ch32v003fun.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// Channel for ADC
#define CHANNEL 0

// For I2C, output will be on PB8/PB9 SCL/SDA
//#define ENABLE_OLED
//#define PWM_OUTPUT
int g_volume_pwm = 127; // 0 - 127 (100%) (but you can go over 100) (For when using PWM)
#define ENABLE_OLED_SCOPE
//#define PROFILING_PIN PC8

#define SAMPLETIME 1 // 0: 1.5 cycles; 1: 7.5 cycles; 2: 13.5 cycles; (0 would go fastest and is important in single-ADC mode, but 1 seems slightly better in 2-ADC mode)

#ifdef ENABLE_OLED_SCOPE
#define SH1107_128x128
#define SSD1306_RST_PIN  PA3
#define SSD1306_CS_PIN   PA4
#define SSD1306_DC_PIN   PA6
#define SSD1306_MOSI_PIN PA7
#define SSD1306_SCK_PIN  PA5
#define SSD1306_BAUD_RATE_PRESCALER SPI_BaudRatePrescaler_4
#include "ssd1306_spi.h"
#include "ssd1306.h"
#endif

#ifdef ENABLE_OLED
#define SH1107_128x128
#define SSD1306_REMAP_I2C
//#define SSD1306_I2C_IRQ
#include "ssd1306_i2c.h"
#include "ssd1306.h"
#endif

#if defined( ENABLE_OLED ) && defined( ENABLE_OLED_SCOPE )
#error Cant be SPI and I2C OLED
#endif


#include "./usb_config.h"
#include "../ch32v003fun/examples_v20x/otg_device/otgusb.h"

// Bigger buffer decreases chance of fall-through, but increases the size of each operation.
#define ADC_BUFFSIZE 512

volatile uint16_t adc_buffer[ADC_BUFFSIZE];

int32_t g_goertzel_phasor_r = 32768;
int32_t g_goertzel_phasor_i = 0;
int32_t g_attenuation_pow2 = 4;

#if 1
// Very basic setup, for tuning to 880AM
int g_pwm_period = (60-1);
int g_exactcompute = (1);
int g_goertzel_buffer = (1024);
int32_t g_goertzel_omega_per_sample = 873460290; // 0.183333 of whole per step / -8.720000MHz
int32_t g_goertzel_coefficient = 873460290;
int32_t g_goertzel_coefficient_s = 1961823932;
int32_t g_goertzel_advance_r = -3425;
int32_t g_goertzel_advance_i = 32588;
#endif

int intensity_average = 1;

#define LOG_GOERTZEL_LIST 512
int32_t qibaselogs[LOG_GOERTZEL_LIST];
volatile int     qibaselogs_head;

void SetupADC()
{
	// XXX TODO -look into PGA
	// XXX TODO - Look into tag-teaming the ADCs

	// PDA is analog input chl CHANNEL
	GPIOA->CFGLR &= ~(0xf<<(4*CHANNEL));	// CNF = 00: Analog, MODE = 00: Input
	
	// ADC CLK is chained off of APB2.

	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2;
	RCC->APB2PRSTR &= ~( RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 );

	// ADCCLK = 12 MHz => RCC_ADCPRE divide by 4
	RCC->CFGR0 &= ~RCC_ADCPRE;  // Clear out the bis in case they were set
	RCC->CFGR0 |= RCC_ADCPRE_DIV2; // Fastest possible (divide-by-2) NOTE: This is OUTSIDE the specified value in the datasheet.

	// Set up single conversion on chl 7
	ADC1->RSQR3 = CHANNEL;	// 0-9 for 8 ext inputs and two internals  Set to 7 for PA7
	ADC2->RSQR3 = CHANNEL;	// 0-9 for 8 ext inputs and two internals  Set to 7 for PA7

	ADC1->ISQR = CHANNEL; // Mirror in case we switch to injection mode.
	ADC2->ISQR = CHANNEL;

	// Not using injection group.

	// Sampling time for channels. Careful: This has PID tuning implications.
	// Note that with 3 and 3,the full loop (and injection) runs at 138kHz.
	ADC1->SAMPTR2 = (SAMPLETIME<<(3*CHANNEL));  // (3*channel)
	ADC2->SAMPTR2 = (SAMPLETIME<<(3*CHANNEL));  // (3*channel)

	// Turn on ADC and set rule group to sw trig
	// 0 = Use TRGO event for Timer 1 to fire ADC rule.
	ADC1->CTLR2 = ADC_ADON | ADC_EXTTRIG | ADC_DMA; 
	ADC2->CTLR2 = ADC_ADON | ADC_EXTTRIG | ADC_EXTSEL_1;// | ADC_DMA; 
		// For EXTTRIG, EXTSEL (none) = 0 = TIM1CC1 /
		// For JEXTTRIG, EXTSEL = 0 = TIM1 TRGO  (Or ADC_JEXTSEL_0 => CH4)

	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	ADC2->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	while(ADC2->CTLR2 & ADC_RSTCAL);

	// Calibrate ADC
	ADC1->CTLR2 |= ADC_CAL;
	ADC2->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);
	while(ADC2->CTLR2 & ADC_CAL);

	// ADC_SCAN: Allow scanning.
	ADC2->CTLR1 = ADC_SCAN;
	ADC1->CTLR1 = 
		//ADC_DUALMOD_0 | ADC_DUALMOD_3 | // Alternate Trigger Mode (Can't use with DMA)
		ADC_SCAN;
		//ADC_Pga_16 | ADC_BUFEN ;
		//ADC_Pga_64 |  ADC_BUFEN;

	// Turn on DMA
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	//DMA1_Channel1 is for ADC
	DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
	DMA1_Channel1->MADDR = (uint32_t)adc_buffer;
	DMA1_Channel1->CNTR  = ADC_BUFFSIZE/2;
	DMA1_Channel1->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_VeryHigh |
		DMA_MemoryDataSize_Word |
		DMA_PeripheralDataSize_Word |
		DMA_MemoryInc_Enable |
		DMA_Mode_Circular |
		DMA_DIR_PeripheralSRC;

	NVIC_EnableIRQ( DMA1_Channel1_IRQn );
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN | DMA_IT_TC | DMA_IT_HT; // Transmission Complete + Half Empty Interrupts. 
	
	// Enable continuous conversion and DMA
	ADC1->CTLR2 |= ADC_DMA; // | ADC_CONT;
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

	TIM1->CCER |= TIM_CC1E | TIM_CC4E | TIM_CC3E;
	TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;
	TIM1->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1 | TIM_OC4M_2 | TIM_OC4M_1;
	TIM1->CH1CVR = 1; // In case we are using rule triggering
	TIM1->CH3CVR = 1; // In case we are using rule (alternate) triggering
	TIM1->CH4CVR = 1; // In case we are using injection triggering

	// Setup TRGO to trigger for ADC injection group
	TIM1->CTLR2 = TIM_MMS_1;

	// Enable TIM1 outputs
	TIM1->BDTR = TIM_MOE;
	TIM1->CTLR1 = TIM_CEN;
}

#ifdef ENABLE_OLED_SCOPE

#ifdef PLOTGUARD
// Command-mode, Set X, Disable Timer, Set Y, Enable Timer
// There doesn't seem to be a way of truly pausing the scanout.
// GENERAL NOTE: This doesn't actually seem to help reduce streaking.
uint8_t cmdxy[] = { 0x00, 0xd3, 0x30, 0xd5, 0xff, 0xdc, 0x30, 0xd5, 0xf0 };
#else
// Only set xy plot, and make sure clock is cranked.
uint8_t cmdxy[] = { 0x00, 0xd3, 0x30, 0xdc, 0x30, 0xd5, 0xf0  };
#endif

void config_turbo_scope()
{
	DMA1_Channel3->PADDR = (uint32_t)&SPI1->DATAR;
	DMA1_Channel3->MADDR = (uint32_t)cmdxy;
	DMA1_Channel3->CNTR  = sizeof(cmdxy);
	DMA1_Channel3->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_Low |
		DMA_MemoryDataSize_Byte |
		DMA_PeripheralDataSize_Byte |
		DMA_MemoryInc_Enable |
		DMA_Mode_Normal |
		DMA_DIR_PeripheralDST;

	// Turn on DMA channel 3 (For SPI output)
	DMA1_Channel3->CFGR |= DMA_CFGR1_EN;

	// All display controls from here on out are made using DC = 0.
	funDigitalWrite( SSD1306_DC_PIN, FUN_LOW );

	SPI1->CTLR2 |= SPI_CTLR2_TXDMAEN;
}

static void PlotPoint( int x, int y )
{
	funDigitalWrite( SSD1306_CS_PIN, FUN_HIGH );
#ifdef PLOTGUARD
	cmdxy[2] = x; cmdxy[6] = y;
#else
	cmdxy[2] = x; cmdxy[4] = y;
#endif
	DMA1_Channel3->CNTR  = sizeof(cmdxy);
	funDigitalWrite( SSD1306_CS_PIN, FUN_LOW );
	DMA1_Channel3->CFGR |= DMA_CFGR1_EN;
}
#endif

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

		int tpl = ADC_BUFFSIZE - DMA1_Channel1->CNTR*2; 
		// Warning, sometimes this is DMA1_Channel1->CNTR == to the base, or == 0 (i.e. might be 256, if top is 255)

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
			// Also, this is the current limiting factor for the maximum samplerate.
			// We can't go above 7.2MSPS and keep up here when main CPU is @ 144MHz.

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
				int32_t rr = (((int64_t)(g_goertzel_coefficient  ) * (int64_t)zp<<1)>>(32+g_attenuation_pow2)) - (zp2>>g_attenuation_pow2);
				int32_t ri = (((int64_t)(g_goertzel_coefficient_s) * (int64_t)zp<<1)>>(32+g_attenuation_pow2));

				// Advanced the current goertzel advance 
				// phasor = phasor * advance;
				//  real = real * real - imag * imag;
				//  imag = real * imag + real * imag;
				// Sometimes you would bias the output here so that when truncating down you don't perpetually decay.
				// But experimentally, it didn't make a difference.
				int32_t temp = (g_goertzel_phasor_r * g_goertzel_advance_i + g_goertzel_phasor_i * g_goertzel_advance_r) >> 15;
				g_goertzel_phasor_r = (g_goertzel_phasor_r * g_goertzel_advance_r - g_goertzel_phasor_i * g_goertzel_advance_i) >> 15;
				g_goertzel_phasor_i = temp;

				// Fixup phasor over time to prevent it from dacaying.
				#define ABS(x) (((x)<0)?-(x):(x))
				int s_phasor = g_goertzel_phasor_r * g_goertzel_phasor_r + g_goertzel_phasor_i * g_goertzel_phasor_i;
				int intensity_phasor = (ABS(g_goertzel_phasor_r) + ABS(g_goertzel_phasor_i)) * 26100 / 32768 + 1; // Found experimentally (Also try to avoid divide-by-zero.
				intensity_phasor = (intensity_phasor + s_phasor/intensity_phasor)/2;
				intensity_phasor = (intensity_phasor + s_phasor/intensity_phasor)/2;
				if( intensity_phasor < 32760 )
				{
					// It is decaying, this is equivelent to f = f * 1.000244141
					g_goertzel_phasor_r += g_goertzel_phasor_r >> 12;
					g_goertzel_phasor_i += g_goertzel_phasor_i >> 12;
				}

				// Now, rotate rr, ri by that phasor.
				// To get it in line >> 15, but we also want to divide by 8 (>>3) because that makes the rest of the math easier.
				temp = (g_goertzel_phasor_r * ri + g_goertzel_phasor_i * rr) >> (15); 
				rr = (g_goertzel_phasor_r * rr - g_goertzel_phasor_i * ri) >> (15);
				ri = temp;

				// rr, ri are now in the correct frame of reference.  Continue computing.

				qibaselogs[qibaselogs_head] = ((uint16_t)rr) | (((uint16_t)ri)<<16);
				qibaselogs_head = ( qibaselogs_head + 1 ) & ((LOG_GOERTZEL_LIST)-1);

				int s = rr * rr + ri * ri;
				//int intensity = 1<<( ( 32 - __builtin_clz(s) )/2);
				#define ABS(x) (((x)<0)?-(x):(x))
				int intensity = (ABS(rr) + ABS(ri)) * 26100 / 32768; // Found experimentally (Also try to avoid divide-by-zero.
				//if( intensity == 0 )
				//	intensity = 1;
				intensity++;
				intensity = (intensity + s/intensity)/2;
				intensity = (intensity + s/intensity)/2;

				// intensity = rr * rr + ri * ri

				// This performs a low-pass IIR without exploding intensity_average.
				intensity_average = intensity_average - (intensity_average>>12) + (intensity>>6);
				if( ((int32_t)intensity_average) >= 1<<23 ) intensity_average = (1<<23)-1;
				if( ((int32_t)intensity_average) < 2048 ) intensity_average = 2048;

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

#ifdef ENABLE_OLED_SCOPE
				int rrplot = rr * 1536 / (intensity_average);
				int riplot = ri * 1536 / (intensity_average);
				rrplot += 64;
				riplot += 64;
				if( rrplot < 1 ) rrplot = 1;
				if( riplot < 1 ) riplot = 1;
				if( rrplot > 126 ) rrplot = 126;
				if( riplot > 126 ) riplot = 126;
				PlotPoint( rrplot, riplot );
#endif

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

uint8_t i2c_send_buffer[16];
void setup_i2c_dma(void)
{
	// Turn on DMA
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	//DMA1_Channel6 is for I2C1TX
	DMA1_Channel6->PADDR = (uint32_t)&I2C1->DATAR;
	DMA1_Channel6->MADDR = (uint32_t)&i2c_send_buffer;
	DMA1_Channel6->CNTR  = 0;
	DMA1_Channel6->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_Low |
		DMA_MemoryDataSize_Byte |
		DMA_PeripheralDataSize_Byte |
		DMA_MemoryInc_Enable |
		DMA_Mode_Normal |
		DMA_DIR_PeripheralDST | 
		0;
	I2C1->CTLR2 = I2C_CTLR2_DMAEN | 0b111100;

	DMA1_Channel6->CFGR |= DMA_CFGR1_EN;

	NVIC_DisableIRQ(I2C1_EV_IRQn);
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

#if defined( ENABLE_OLED )
	ssd1306_i2c_setup();
	ssd1306_i2c_init();

	if( ssd1306_init() )
		printf( "Failed to initialize OLED\n" );
	else
		printf( "Initialized OLED\n" );

	ssd1306_setbuf(1);
	ssd1306_refresh();
#endif

#ifdef ENABLE_OLED_SCOPE
	int i;

	ssd1306_spi_init();
	ssd1306_rst();

	if( ssd1306_init() )
		printf( "Failed to initialize OLED\n" );
	else
		printf( "Initialized OLED\n" );

	ssd1306_setbuf(0);

	// Setup a diagonal to allow for vector mode.
	for( i = 0; i < 128; i++ )
	{
		ssd1306_drawPixel( i, i, 1 );
		//ssd1306_drawPixel( i+1, i, 1 );
	}

	// Not sure why, need to do it a few times to make it stick?
	ssd1306_refresh();
	ssd1306_refresh();


	uint8_t force_two_row_mode[] = {
		0xa8, 0, // Set MUX ratio (Actually # of lines to scan) (But it's this + 1)  You can make this 1 for wider.
	};
	ssd1306_pkt_send(force_two_row_mode, sizeof( force_two_row_mode ) , 1);

	uint8_t force_max_speed[] = {
		0xd5, 0xf0
	};
	ssd1306_pkt_send(force_max_speed, sizeof( force_max_speed ) , 1);

	config_turbo_scope();
#if 0 // Test streaking
	int rframe = 0;
	while(1)
	{
		PlotPoint( rframe & 0xff, rframe>>8 );
		rframe++;
		//Delay_Ms(1);
	}
#endif

#endif

#ifdef PROFILING_PIN
	funPinMode( PROFILING_PIN, GPIO_CFGLR_OUT_50Mhz_PP );
#endif

	SetupTimer1();

	USBOTGSetup();


	while(1){
#if 0
		// To draw a sinewave...
		int adcz = adc_buffer[0];
		for( k = 0; k < 128; k++ )
		{
			int y = adc_buffer[k]-adcz + 64;
			if( y < 0 ) y = 0;
			if( y > 127 ) y = 127;
			ssd1306_drawPixel( k, y, 1 );
		}
#endif

		// Only display half of the list so the other half could
		// be updated by the ISR.

#ifdef ENABLE_OLED
		int pxa = 0;
		int glread = qibaselogs_head;

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
			
			ssd1306_drawPixel( rr, ri, 1 );
		}

		ssd1306_refresh();

		ssd1306_setbuf(0);
#endif

		// Do nothing.
		Delay_Ms(17);
	}
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

		int intensity_send = intensity_average;

		if( intensity_send >= (1<<24) )
			intensity_send = (1<<24)-1;
		((uint32_t*)scratchpad)[0] = (((uint32_t)intensity_send)<<8) | samps_to_send;
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

		//printf( "Is Configure Packet %08x\n", configs[1] );

		int numconfigs = configs[1];
		if( numconfigs > 0) g_pwm_period = configs[2];
		if( numconfigs > 1) g_goertzel_buffer = configs[3];
		if( numconfigs > 2) g_goertzel_omega_per_sample = configs[4]; // 0.816667 of whole per step / 0.880000MHz
		if( numconfigs > 3) g_goertzel_coefficient = configs[5];
		if( numconfigs > 4) g_goertzel_coefficient_s = configs[6];
		if( numconfigs > 5) g_exactcompute = configs[7];
		if( numconfigs > 6) g_goertzel_advance_r = configs[8];
		if( numconfigs > 7) g_goertzel_advance_i = configs[9];
		if( numconfigs > 8) 
		{
			int adc_buffer = configs[10];
			if( adc_buffer )
			{
				// Consider using PGA.
				//ADC_Pga_16 | ADC_SCAN | ADC_BUFEN ;
				//ADC_Pga_64 | ADC_SCAN;
				ADC1->CTLR1 |= ADC_BUFEN;// | ADC_Pga_4; // Adding PGA causes wild oscillation.
				ADC1->CTLR2 |= ADC_BUFEN;// | ADC_Pga_4;
			}
			else
			{
				ADC1->CTLR1 &= (~ADC_BUFEN);
				ADC2->CTLR1 &= (~ADC_BUFEN);
			}
		}
		if( numconfigs > 9) g_attenuation_pow2 = configs[11];

		// Need to reset so we don't blast by.
		g_goertzel_samples = 0;
		TIM1->ATRLR = g_pwm_period;

		TIM1->CH1CVR = 1;
		TIM1->CH3CVR = TIM1->ATRLR/2+1;


		g_isConfigurePacket = 0;
	}
	return;
}

