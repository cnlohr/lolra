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

// This was my first test, it still contains a number of TESTSTRAP flags.
// Teststrap was used when I compiled and ran this code on a host PC.
// In general, don't use this, code is just left around in case I run into
// any really weird bugs.
#ifdef TESTSTRAP

#include <stdint.h>
#include <stdio.h>
#include "chirpbuff.h"
#include <stdlib.h>

#define uint32 uint32_t

#else

#include "esp8266_auxrom.h"
#include "eagle_soc.h"
#include "nosdk8266.h"
#include "esp8266_rom.h"

// TODO: Use float number (related to 8) to fix the drift
#define call_delay_us(time) { asm volatile("mov.n a2, %0\n_call0 delay4clk" : : "r"(time * (MAIN_MHZ / 8)) : "a2" ); }

#include "ets_sys.h"
#include "pin_mux_register.h"

#endif

#include "slc_register.h"
#include "dmastuff.h"
#include "chirpbuffinfo.h"
#include "LoRa-SDR-Code.h"

#define DMABUFFERDEPTH 3
//These contol the speed at which the bus comms.
#define WS_I2S_BCK SPI_DIV  //Can't be less than 1.
#define WS_I2S_DIV 1

#ifndef TESTSTRAP
#include "esp8266_i2s_setup.h"
#endif

uint32_t chirpbuffUP[CHIRPLENGTH_WORDS_WITH_PADDING];
uint32_t chirpbuffDOWN[CHIRPLENGTH_WORDS_WITH_PADDING];
uint32_t dummy[DMA_SIZE_WORDS];

volatile int fxcycle;
int etx;

// Limit because of RAM usage, but this should be able to hold a practical limit of a 255 byte LoRa packet.
#define MAX_SYMBOLS 532


// Our table is bespoke for the specific SF.
#define CHIPSSPREAD CHIRPLENGTH_WORDS// QUARTER_CHIRP_LENGTH_WORDS (TODO: Use the quater value elsewhere in the code)
#define MARK_FROM_SF0 (1<<SF_NUMBER) // SF7

#define PREAMBLE_CHIRPS 10
#define CODEWORD_LENGTH 2

uint32_t quadsetcount;
int32_t quadsets[MAX_SYMBOLS*4+PREAMBLE_CHIRPS*4+9+CODEWORD_LENGTH*4];

int32_t * AddChirp( int32_t * qso, int offset, int verneer )
{
	offset = offset * CHIPSSPREAD / (MARK_FROM_SF0);
	offset += verneer;
	*(qso++) = (CHIPSSPREAD * 0 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	*(qso++) = (CHIPSSPREAD * 1 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	*(qso++) = (CHIPSSPREAD * 2 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	*(qso++) = (CHIPSSPREAD * 3 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	return qso;
}


volatile int quadsetplace = -1;

int runningcount_bits = 0;

void slc_isr(void * v) {

	uint32_t * sendbuff = 0;
	uint32_t sendlen = 0;
	struct sdio_queue *finishedDesc;
//	slc_intr_status = READ_PERI_REG(SLC_INT_STATUS); -> We should check to make sure we are SLC_RX_EOF_INT_ST, but we are only getting one interrupt.
#ifdef TESTSTRAP
	struct sdio_queue tmp;
	finishedDesc = &tmp;
#else
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
	finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);
#endif

	etx++;

	if( quadsetplace < 0 )
	{
		goto dump0;
	}

	// LoRa symbols are in quarters of a chirp.
	if( fxcycle>= NUM_DMAS_PER_QUARTER_CHIRP )
	{
		fxcycle = 0;
		quadsetplace++;
		if( quadsetplace >= quadsetcount ) goto dump0;
	}

	int symbol = quadsets[quadsetplace];

	// Select down- or up-chirp.
	if( symbol < 0 )
	{
		int word = fxcycle * DMA_SIZE_WORDS - symbol - 1;
		if( word >= CHIPSSPREAD ) word -= CHIPSSPREAD;
		word++;
		sendbuff = (chirpbuffDOWN + word);
	}
	else
	{
		int word = fxcycle * DMA_SIZE_WORDS + symbol;
		if( word >= CHIPSSPREAD ) word -= CHIPSSPREAD;
		sendbuff = (chirpbuffUP + word);
	}

#ifndef FOUND_PERFECT_DIVISOR
	// Sometimes we do the full length, of all of the needed DMAs
	// Sometimes we overshoot the time window, so we peel off 4 bytes.
	//
	// Very few combinations of clock rate, divisor, etc can produce
	// perfect divisors. Most notably 52MHz, /2 SF9 can produce a perfect
	// divisor.  While this is very tidy and beautiful that the 
	// words would align perfectly, the actual difference it makes on
	// LoRa's ability to receive the message is minimal.
	//
	// Additionally, 80MHz /2 SF7 can produce a perfect divisor.
	int running_bits_after = runningcount_bits + DMA_SIZE_WORDS*32;
	int overflow = running_bits_after - IDEAL_QUARTER_CHIRP_LENGTH_BITS;
	if( overflow >= 0 )
	{
		int overflow_amount = overflow / 32;
		int overflow_remainder = overflow % 32;
		sendlen = DMA_SIZE_WORDS*4 - 4*overflow_amount;
		runningcount_bits = overflow_remainder;

		// XXX TODO: Why can't I put the logic for advancing the group in here?
	}
	else
	{
		sendlen = DMA_SIZE_WORDS*4;
		runningcount_bits = running_bits_after;
	}
#else
	sendlen = DMA_SIZE_WORDS*4;
#endif

#ifdef TESTSTRAP
	static FILE * fappendlog;
	if( !fappendlog ) fappendlog = fopen( "fappendlog.csv", "w" );
	{
		if( symbol < 0 )
			fprintf( fappendlog, "2, %d, %d\n", (int)(CHIRPLENGTH_WORDS - (sendbuff - chirpbuffDOWN) - 1), sendlen );
		else
			fprintf( fappendlog, "1, %d, %d\n", (int)(sendbuff - chirpbuffUP), sendlen );
	}
#else
	finishedDesc->buf_ptr = (uint32_t)sendbuff;
	finishedDesc->datalen = sendlen;
#endif

	fxcycle++;
	return;


dump0:
#ifdef TESTSTRAP
	printf( "Hit dummy %d %d\n", quadsetplace, quadsetcount );
	exit( 0 );
#else
	// This location just always reads as zeroes.
	finishedDesc->buf_ptr = (uint32_t)dummy;
	quadsetplace = -1;
#endif
	return;
}


#ifdef TESTSTRAP

void SPIRead( uint32_t pos, uint32_t * buff, int len )
{
	memcpy( buff, (pos - 0x00020000) + (uint8_t*)chirpbuff, len );
}

void nosdk8266_init()
{
}


void testi2s_init( uint32_t * default_address )
{
}

#endif

int main()
{
	// We store the bit pattern at flash:0x20000, so we don't have to constantly
	// re-write it when working on code.
	SPIRead( MEMORY_START_OFFSET_BYTES, chirpbuffUP, sizeof( chirpbuffUP ) );
	SPIRead( REVERSE_START_OFFSET_BYTES, chirpbuffDOWN, sizeof( chirpbuffDOWN ) );
	memset( dummy, 0, sizeof( dummy ) );

	// Don't crank up clock speed til we're done with flash.
	nosdk8266_init();


	int i = 0;
	fxcycle = 0;
	etx = 0;


#ifndef TESTSTRAP
	// Configure GPIO5 (TX) and GPIO2 (LED)
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,FUNC_GPIO5);
	PIN_DIR_OUTPUT = _BV(2); //Enable GPIO2 light off.

	// Run the I2S bus at 1040/6 = 173.333 MHz.
	// It looks like, at least on my part, if I try running
	// hotter it can get to 1040/5.1 but not all the way to
	// 5 so it's unstable there.
#endif
	testi2s_init( dummy );

	int frame = 0;
	uint16_t lora_symbols[MAX_SYMBOLS];
	int lora_symbols_count;

	while(1) {
		//12x this speed.
		frame++;
#ifndef TESTSTRAP
		PIN_OUT_SET = _BV(2); //Turn GPIO2 light off.
		//call_delay_us(1000000);
		printf("ETX: %d %08x\n", fxcycle, chirpbuffUP[10] );
		PIN_OUT_CLEAR = _BV(2); //Turn GPIO2 light off.
		call_delay_us(500000);
#endif
		call_delay_us(2000000);

		// Just some random data.
		uint8_t payload_in[259] = { 0xbb, 0xcc, 0xde, 0x55, 0x22,}; 
		int payload_in_size = 6;

		static int msgno = 0;
		payload_in[4] = msgno++;

		lora_symbols_count = 0;
		int r = CreateMessageFromPayload( lora_symbols, &lora_symbols_count, MAX_SYMBOLS, SF_NUMBER, 4, payload_in, payload_in_size );

		if( r < 0 )
		{
			printf( "Failed to generate message (%d)\n", r );
			// Failed
			continue;
		}

		int j;

		quadsetcount = 0;
		int32_t * qso = quadsets;
		for( j = 0; j < PREAMBLE_CHIRPS; j++ )
		{
			qso = AddChirp( qso, 0, 0 );
		}

		uint8_t syncword = 0x43;

	#if SF_NUMBER == 6
		#define CODEWORD_SHIFT 2
	#else
		#define CODEWORD_SHIFT 3
	#endif

		if( CODEWORD_LENGTH > 0 )
			qso = AddChirp( qso,  ( ( syncword & 0xf ) << CODEWORD_SHIFT ), 0 );
		if( CODEWORD_LENGTH > 1 )
			qso = AddChirp( qso, ( ( ( syncword & 0xf0 ) >> 4 ) << CODEWORD_SHIFT ), 0);

		*(qso++) = -(CHIPSSPREAD * 0 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 1 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 2 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 3 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 0 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 1 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 2 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 3 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 0 / 4 )-1;

		if( SF_NUMBER <= 6 )
		{
			// Two additional upchirps with SF6 https://github.com/tapparelj/gr-lora_sdr/issues/74#issuecomment-1891569580
			for( j = 0; j < 2; j++ )
				qso = AddChirp( qso, 0, 0 );
		}

		for( j = 0; j < lora_symbols_count; j++ )
		{
			int ofs = lora_symbols[j];
			//ofs = ofs ^ ((MARK_FROM_SF6<<6) -1);
			//ofs &= (MARK_FROM_SF6<<6) -1;
			qso = AddChirp( qso, ofs, 0 );
			printf( "%02x ", ofs );
		}
		printf( "\n" );

		runningcount_bits = 0;

		// This tells the interrupt we have data.
		quadsetcount = qso - quadsets + 0;
		printf( "--- %d [%d] %d\n", lora_symbols_count, quadsetcount, CHIPSSPREAD/4 );
		quadsetplace = 0;
#ifdef TESTSTRAP
		while(1)
		{
			slc_isr( 0 );
		}
#endif
	}

}

