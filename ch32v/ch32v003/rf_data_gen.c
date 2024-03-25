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

#include <stdint.h>


//#define KHZ500

#ifdef KHZ500

// Tested, works - be sure to set your radio to be able to receive this.
#define SF_NUMBER 10
const double center_frequency = 904.6;
const double bw = .5;
#define SF_SYMBOL_TIME 0.00000025
#else

// Tested, works.
const double center_frequency = 904.5;
#define SF_NUMBER 8
const double bw = .125;
#endif

const uint32_t memory_offset = 0x00000;
#define SPI_DIV 2
const double sample_rate = (48.0/2)/SPI_DIV;


#define ATTRIBUTE_FOR_DATA "__attribute__((section(\".text\"),aligned(32)))"
#define USE_16_BITS

#include "../../lib/rf_data_gen.h"

int main()
{
	return gen_buffer_files();
}
