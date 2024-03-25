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

// This file is for generated a 310MHz CW Tone that lines up with itself so
// It can be repeated many times in-phase.


#include <stdint.h>
#include <stdio.h>
#include <math.h>

const double rf_frequency = 310;
//const double spread = 0.000001;
const double spread = 0.0;

#define SPI_DIV 1

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

#define MAX_WORDS_DMA 511
#define NUMBITS 32

int main()
{
	int i;
	FILE * fcba = fopen( "samples.h", "w" );
	double phase = 0;
	uint32_t samplect = 0;
	uint32_t words = 0;
	uint32_t sample_word = 0;
	int32_t best_words = -1;
	double best_phase = 1000;
	double current_frequency = rf_frequency;
	for( i = 0; i < MAX_WORDS_DMA*NUMBITS; i++ )
	{
		phase += 3.1415926 * 2 * current_frequency / sample_rate;
		current_frequency += spread;
		while( phase > 3.1415926 * 2.0 ) phase -= 3.1415926 * 2.0;
		samplect++;
		if( samplect == NUMBITS )
		{
			words++;
			printf( "Phase Delta at %d: %f\n", words, phase );
			if( words > 64 ) // Minimum number of words
			{
				double phasediff = ( phase > 3.1415926 ) ? (3.1415926*2.0 - phase) : phase;
				if( phasediff < best_phase )
				{
					best_phase = phasediff;
					best_words = words;
				}
			}
			samplect = 0;
		}
	}

	uint32_t num_words = best_words;

	printf( "Selected %d words\n", num_words );
	samplect = 0;
	words = 0;
	sample_word = 0;
	phase = 0;
	current_frequency = rf_frequency;

	fprintf( fcba, "uint32_t samples[%d] = { \n\t", num_words );
	for( i = 0; i < num_words*NUMBITS; i++ )
	{
		phase += 3.1415926 * 2 * current_frequency / sample_rate;
		while( phase > 3.1415926 * 2.0 ) phase -= 3.1415926 * 2.0;
		current_frequency += spread;

		// This is the magic line - this is what decides if you are emitting a `1` bit or a `0` bit.
		int bit = sin( phase ) > 0.0; // HINT: if you want to mix multiple signals, add a DC offset here.
		sample_word |= !!bit;
		samplect++;
		if( samplect == NUMBITS )
		{
#ifdef USE_16_BITS
			fprintf( fcba, "0x%04x,%s", sample_word, ((words & 0xf) != 0xf)?" ":"\n\t" );
#else
			fprintf( fcba, "0x%08x,%s", sample_word, ((words & 0xf) != 0xf)?" ":"\n\t" );
#endif
			//fwrite( &sample_word, 1, NUMBITS/8, fd );
			words++;
			sample_word = 0;
			samplect = 0;
		}
		sample_word <<= 1;
	}
	fprintf( fcba, "};\n" );
	fprintf( fcba, "#define DMA_WORDS %d\n", num_words );
	fprintf( fcba, "#define SPI_DIV %d\n", SPI_DIV );
	fprintf( fcba, "#define NUMBITS %d\n", NUMBITS );
	fprintf( fcba, "#define SAMPLE_RATE %d\n", (int)(sample_rate*1000000) );
}

