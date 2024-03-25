/*
MIT License

Copyright (c) 2024 Charles Lohr "CNLohr"

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// TEST 1: 125kHz with -.028                   675
// TEST 2: 500kHz with 138 kHz correction.     534??
// TEST 3: 500kHz with 0 kHz correction. FAIL
// TEST 4: 125kHz with -.028 5V                1210

//END OF RED ANTENNA

// TEST 5: 125kHz with -.027 3.3V, shorter antenna?   829

// TEST 6: 125k ESP 80 GREY antenna ???/???  850 / 

#include <stdint.h>

// Transmit  a 500kHz width LoRa
//#define KHZ500

// Expects a 24MHz external clock.
//#define USE_EXTERNAL_CLOCK

#ifdef KHZ500

	const double tune_mhz = -0.000;
	const double center_frequency = 904.6 + tune_mhz;
	const double bw = .5;
	#ifdef USE_EXTERNAL_CLOCK
	#define SF_NUMBER 9
	#else
	#define SF_NUMBER 10
	#endif
	#define SF_SYMBOL_TIME 0.00000025

#else // 125kHz channel

	const double bw = 0.125;

	#ifdef USE_EXTERNAL_CLOCK
	const double tune_mhz = 0.00;
	#define SF_NUMBER 7
	#else
	const double tune_mhz = -0.028; //XXX ??? Per device tuning?  Or area the load caps just wrong?
	#define SF_NUMBER 8
	#endif

	const double center_frequency = 904.5 + tune_mhz;

#endif

#define SPI_DIV 1


#ifdef USE_EXTERNAL_CLOCK
const double sample_rate = (24*9/2)/SPI_DIV; //216MHz PLL
#else
const double sample_rate = (72)/SPI_DIV;
#endif

#define ATTRIBUTE_FOR_DATA "__attribute__((section(\".text\")))"
#define USE_16_BITS

const uint32_t memory_offset = 0x00000;

#include "../../lib/rf_data_gen.h"

int main()
{
	return gen_buffer_files();
}
