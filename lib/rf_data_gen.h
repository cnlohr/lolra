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

*/

// This file is code, but intended to be included into a .c file that
// does #define's for a specific fitness.  Below is an example .c file
// that this could be included into to generate the bit table.  It is
// from an ESP8266 example, but other specifics can be specified on a
// per-processor basis.

/*

const double center_frequency = 904.1;
const double bw = .125;
const uint32_t memory_offset = 0x20000;
#define SF_NUMBER 7

#if SF_NUMBER < 7
#warning SF6 still does not work :(
#endif

#define SPI_DIV 1

// Funny modes:
/// 80MHz, SF8,  SPI_DIV 5 @903.9/904.1 produces hilarious mirror images around 904.0

#if MAIN_MHZ == 80
const double sample_rate = 1040.0/13.0/SPI_DIV;
#if ( SF_NUMBER > 8 ) 
#error Not enough ram for chirp table
#endif

#elif MAIN_MHZ == 115
const double sample_rate = 1040.0/9.0/SPI_DIV;
#if ( SF_NUMBER > 8 ) 
#error Not enough ram for chirp table
#endif

#elif MAIN_MHZ == 52
const double sample_rate = 1040.0/20.0/SPI_DIV;
#if ( SF_NUMBER > 9 ) 
#error Not enough ram for chirp table
#endif

#elif MAIN_MHZ == 173
const double sample_rate = 1040.0/6.0/SPI_DIV;
#if (  SF_NUMBER > 7 ) 
#error Not enough ram for chirp table
#endif

#else
#error Unknown Clock Rate
#endif


EXTRA OPTIONS
#define USE_16_BITS
*/

#include <stdio.h>
#include <stdint.h>
#include <math.h>


const double chirp_begin = center_frequency-bw/2;
const double chirp_end = center_frequency+bw/2;

#ifndef MEM_MAX_BYTES
#define MEM_MAX_BYTES 2000000
#endif

#ifndef SF_SYMBOL_TIME
#define SF_SYMBOL_TIME 0.000001
#endif

const double chirp_length_seconds = (8<<SF_NUMBER) * SF_SYMBOL_TIME;
const double sampletotal = ( chirp_length_seconds * sample_rate * 1000000 );

int words = 0;
int words_nominal = 0;
uint32_t bleedover = 0; // Will be tailored based on the needed buffers.

FILE * fcba;
FILE * fd;
FILE * fCBI;


#ifdef USE_16_BITS
#define NUMBITS 16
#else
#define NUMBITS 32
#endif

static uint32_t flipBits( uint32_t w, int len )
{
	int i;
	uint32_t ret = 0;
	for( i = 0; i < len; i++ )
	{
		if( w & (1<<(len-1)) )
			ret |= 1;
		ret <<= 1;
		w <<= 1;
	}
	return ret;
}

void GenChirp( double fStart,  double fEnd )
{
	uint32_t sample_word = 0;
	int samplect = 0;
	double phase = 0.0001;
	int samples = 0;
	int ic;
	{
		for( samples = 0; ; samples++ )
		{
			double placeInSamples = samples / sampletotal;
			if( placeInSamples >= 1 )
			{
				// When going off the top frequency, start over at the bottom.
				placeInSamples -= 1;
			}

			//////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////

			// These are the key lines here.  This is what actually decides the current
			// frequency and figures out what the given bit in a position should be.  
			// I use 0.1 as an offset because it seems to have some beneficial behaviors
			// but 0.0 also seems to work really well.  You can artifically reduce the 
			// amplituide of the output signal by changing the offset in the comparison
			// on the last line of this section.
			
			double current_f = ( placeInSamples ) * ( fEnd - fStart ) + fStart;
			phase += 3.1415926 * 2.0 * current_f / sample_rate;
			int bit = sin( phase ) > 0.1; // HINT: if you want to mix multiple signals, add a DC offset here.

			//////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////

			sample_word |= !!bit;
			samplect++;
			if( samplect == NUMBITS )
			{
#ifdef USE_16_BITS
			//	sample_word = ((sample_word>>8)&0xff) | ((sample_word<<8)&0xff00);
			//	sample_word = flipBits( sample_word, 16 );
				fprintf( fcba, "0x%04x,%c", sample_word, ((words & 0xf) != 0xf)?' ':'\n' );
#else
				fprintf( fcba, "0x%08x,%c", sample_word, ((words & 0xf) != 0xf)?' ':'\n' );
#endif
				fwrite( &sample_word, 1, NUMBITS/8, fd );
				words++;
				sample_word = 0;
				samplect = 0;
				if( samples < sampletotal )
					words_nominal = words;

				if( samples >= sampletotal + bleedover*32 )
				{
					break;
				}
			}
			sample_word <<= 1;
		}
	}
}

#ifndef ATTRIBUTE_FOR_DATA
#define ATTRIBUTE_FOR_DATA
#endif

int gen_buffer_files()
{
	fcba = fopen( "chirpbuff.h", "w" );
	fd = fopen( "chirpbuff.dat", "w" );
	fCBI = fopen( "chirpbuffinfo.h", "w" );

#ifdef USE_16_BITS
	fprintf( fcba, "const uint16_t chirpbuff[] " ATTRIBUTE_FOR_DATA " = {\n" );
#else
	fprintf( fcba, "const uint32_t chirpbuff[] " ATTRIBUTE_FOR_DATA " = {\n" );
#endif



	int quarter_chirp_length_bits = (int)(sampletotal/4.0+0.5);
	int factor = 0;
	int is_perfect_divisor = 0;
	int i;

	// DMA size words, when multiplied out should be SMALLER than the
	// overall length of our message, so that we can trim off the last few
	// words sometimes, to average out to the right bitrate.

	// So, we need to pick a number that when multiplied out, ends up
	// slightly larger than IDEAL_CHIRP_LENGTH_BITS.  But not too much so.

	// First try to find a perfect divisor, if we can find a perfect divisor,
	// then no weird fixup stuff needs to be done.

	int seeking_perfect_divisor = 1;
	bleedover = 0;

retry_without_seek:

	printf( "Searching for a divisor for %d\n", quarter_chirp_length_bits );
	for( i = 511; i > 100; i-- )
	{
		int nr_to_div = (quarter_chirp_length_bits / 32 + 32) / i;
		int num_bits_default = i * 32 * nr_to_div;
		int leftover = num_bits_default - quarter_chirp_length_bits;
		//fprintf( stderr, "%d %d %d [%d] --> %d\n", i, nr_to_div, num_bits_default, quarter_chirp_length_bits, leftover );
		if( leftover < 32*nr_to_div &&
				( seeking_perfect_divisor ? (leftover == 0) : (leftover >= 0)) )
		{
			// Make sure we aren't more than 1 32-bit word too large
			fprintf( stderr, "Found %s divisor: %d (%d - %d = %d)\n",
				(leftover == 0)?"PERFECT" : "OK",
				i,
				num_bits_default,
				quarter_chirp_length_bits,
				num_bits_default - quarter_chirp_length_bits );
			factor = i;

			//XXX TODO: I think this bleedover calc is wrong. Should investigate?
			bleedover = num_bits_default/32;
			is_perfect_divisor = leftover == 0;
			break;
		}
	}

	if( seeking_perfect_divisor && !factor )
	{
		seeking_perfect_divisor = 0;
		goto retry_without_seek;
	}

	if( !factor )
	{
		fprintf( stderr, "Error: No factor of %d found, you may have to do something clever in rf_data_gen.c\n", quarter_chirp_length_bits );
		return -9;
	}

	// For a given word, it is shifted out MSB (Bit and byte) first
	GenChirp( chirp_begin, chirp_end );
	int sample_word_median = words;
	int quarter_chirp_length = ((words_nominal+2)/4);
	int reverse_start = words;
	words = 0;
	GenChirp( chirp_end, chirp_begin );

	fprintf( fcba, "};\n" );
	fclose( fcba );
	fclose( fd );
	
#ifdef USE_16_BITS
	int bytes_total = 2*(words+sample_word_median);
	fprintf( stderr, "Wrote out %d uint16_t's.\n", words + sample_word_median );
#else
	int bytes_total = 4*(words+sample_word_median);
	fprintf( stderr, "Wrote out %d uint32_t's.\n", words + sample_word_median );
#endif

	if( bytes_total > MEM_MAX_BYTES )
	{
		fprintf( stderr, "ERROR: Your table is too big (trying to write out %d bytes, please adjust SF/BW/clock rate settings)\n", bytes_total );
	}

	fprintf( fCBI, "#define CHIRPLENGTH_WORDS (%d)\n", words_nominal );
	fprintf( fCBI, "#define MEMORY_START_OFFSET_BYTES (0x%08x)\n", memory_offset );
	fprintf( fCBI, "#define REVERSE_START_OFFSET_BYTES (0x%08x)\n", memory_offset + reverse_start * NUMBITS/8 );
	fprintf( fCBI, "#define QUARTER_CHIRP_LENGTH_WORDS (%d)\n",  (int)(quarter_chirp_length) );
	fprintf( fCBI, "#define IDEAL_QUARTER_CHIRP_LENGTH_BITS (%d)\n", quarter_chirp_length_bits );
	fprintf( fCBI, "#define CHIRPLENGTH_WORDS_WITH_PADDING (%d)\n", sample_word_median );
	fprintf( fCBI, "#define STRIPE_BLEEDOVER_WORDS (%d)\n", bleedover );
	fprintf( fCBI, "#define TARGET_SAMPLE_COUNT_BITS (%d)\n", (int)sampletotal );
	fprintf( fCBI, "#define DMA_SIZE_WORDS (%d)\n", factor );
	fprintf( fCBI, "#define NUM_DMAS_PER_QUARTER_CHIRP (%d)\n", quarter_chirp_length/factor+((is_perfect_divisor)?0:1) );
	fprintf( fCBI, "#define SF_NUMBER %d\n", SF_NUMBER );
	fprintf( fCBI, "#define SPI_DIV %d\n", SPI_DIV );

	if( factor & 1 )
		fprintf( fCBI, "#define DMA_SIZE_WORDS_DIVISIBLE_BY_TWO 0\n" );
	else
		fprintf( fCBI, "#define DMA_SIZE_WORDS_DIVISIBLE_BY_TWO 1\n" );

	if( factor & 3 )
		fprintf( fCBI, "#define DMA_SIZE_WORDS_DIVISIBLE_BY_FOUR 0\n" );
	else
		fprintf( fCBI, "#define DMA_SIZE_WORDS_DIVISIBLE_BY_FOUR 1\n" );


#ifdef USE_EXTERNAL_CLOCK
	fprintf( fCBI, "#define USE_EXTERNAL_CLOCK\n" );
#endif
	if( is_perfect_divisor )
		fprintf( fCBI, "#define FOUND_PERFECT_DIVISOR\n" );
	fclose( fCBI );
}
