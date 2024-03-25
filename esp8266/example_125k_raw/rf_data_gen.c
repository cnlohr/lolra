/**

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

**/

#include <stdint.h>

const double center_frequency = 904.5;
const double bw = .125;
const uint32_t memory_offset = 0x20000;
#define SF_NUMBER 9
#define SF_SYMBOL_TIME 0.000001
#define MEM_MAX_BYTES 68000

#if SF_NUMBER < 7
#warning SF6 still does not work :(
#endif

#define SPI_DIV 1

// Funny modes:
/// 80MHz, SF8,  SPI_DIV 5 @903.9/904.1 produces hilarious mirror images around 904.0

#if MAIN_MHZ == 80
const double sample_rate = 1040.0/13.0/SPI_DIV;

#elif MAIN_MHZ == 115
const double sample_rate = 1040.0/9.0/SPI_DIV;

#elif MAIN_MHZ == 52
const double sample_rate = 1040.0/20.0/SPI_DIV;

#elif MAIN_MHZ == 173
const double sample_rate = 1040.0/6.0/SPI_DIV;

#else
#error Unknown Clock Rate
#endif


#include "../../lib/rf_data_gen.h"

int main()
{
	return gen_buffer_files();
}
