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

// Transmit LoRaWAN Packets from an ESP8266 - be sure to fill in your
// payload_key, network_skey and devaddress

#include "esp8266_auxrom.h"
#include "eagle_soc.h"
#include "nosdk8266.h"
#include "esp8266_rom.h"
#include "ets_sys.h"
#include "pin_mux_register.h"
#include "chirpbuffinfo.h"
#include "LoRa-SDR-Code.h"


#define DMABUFFERDEPTH 3
//These contol the speed at which the bus comms.
#define WS_I2S_BCK SPI_DIV  //Can't be less than 1.
#define WS_I2S_DIV 1

#include "esp8266_i2s_setup.h"

#include "lorawan_simple.h"


uint32_t chirpbuffUP[CHIRPLENGTH_WORDS_WITH_PADDING];
uint32_t chirpbuffDOWN[CHIRPLENGTH_WORDS_WITH_PADDING];
uint32_t dummy[DMA_SIZE_WORDS];

volatile int fxcycle;
int etx;

// Practical limit because of RAM usage.  But this should be able to hold a full-sized (255-byte) LoRa packet.
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

// TODO: Use float number (related to 8) to fix the drift
#define call_delay_us(time) { asm volatile("mov.n a2, %0\n_call0 delay4clk" : : "r"(time * (MAIN_MHZ / 8)) : "a2" ); }

void slc_isr(void * v) {

	uint32_t * sendbuff = 0;
	uint32_t sendlen = 0;
	struct sdio_queue *finishedDesc;

	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
	finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);

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

	finishedDesc->buf_ptr = (uint32_t)sendbuff;
	finishedDesc->datalen = sendlen;

	fxcycle++;
	return;


dump0:
	// This location just always reads as zeroes.
	finishedDesc->buf_ptr = (uint32_t)dummy;
	quadsetplace = -1;
	return;
}

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
	// Configure GPIO5 (TX) and GPIO2 (LED)
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,FUNC_GPIO5);
	PIN_DIR_OUTPUT = _BV(2); //Enable GPIO2 light off.

	// Run the I2S bus at 1040/6 = 173.333 MHz.
	// It looks like, at least on my part, if I try running
	// hotter it can get to 1040/5.1 but not all the way to
	// 5 so it's unstable there.

	testi2s_init( dummy );

	int frame = 0;
	uint16_t lora_symbols[MAX_SYMBOLS];
	int lora_symbols_count;

	frame = 0;

	while(1) {
		//12x this speed.
		frame++;

		PIN_OUT_SET = _BV(2); //Turn GPIO2 light off.
		printf("ETX: %d %08x\n", fxcycle, chirpbuffUP[10] );
		PIN_OUT_CLEAR = _BV(2); //Turn GPIO2 light on.
		call_delay_us(5000000);

		// Send a message with LoraWan.  Formatted specifically for thethings.network.
		uint8_t inner_payload_raw[20] = { 'T', 'e', 's', 't', '0', '0', '0', '0', ' ', ' ', ' ', ' ', ' ', ' ', ' ' };
		inner_payload_raw[4] = ((frame/1000)%10)+'0';
		inner_payload_raw[5] = ((frame/100)%10)+'0';
		inner_payload_raw[6] = ((frame/10)%10)+'0';
		inner_payload_raw[7] = (frame%10)+'0';
		int inner_payload_len = 20;

		// Get these from your thethings.network console.
		// GENERAL NOTE: When addinga new device, go to General Settings -> Network Layer -> Advanced MAC Settings -> Resets frame counters [check]
		static const uint8_t payload_key[AES_BLOCKLEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // AppSKey Big Endian
		static const uint8_t network_skey[AES_BLOCKLEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // NwkSKey Big Endian
		static const uint8_t devaddress[4] = { 0x00, 0x00, 0x00, 0x00 }; // Device address Little Endian LSB (Written backwards from Device address default view)
#warning Please set payload_key, network_skey and devaddress

		// Just some random data.
		uint8_t raw_payload_with_b0[259+8] = { };
		uint8_t * raw_payload = raw_payload_with_b0 + 16;
		uint8_t * pl = raw_payload;
		int raw_payload_size = 0;

		pl += GenerateLoRaWANPacket( raw_payload_with_b0, inner_payload_raw, inner_payload_len, payload_key, network_skey, devaddress, frame);

		raw_payload_size = pl - raw_payload;

		printf( "Length: %d\n", raw_payload_size );

		for( i = 0; i < raw_payload_size; i++ )
			printf( "%02x ", raw_payload[i] );

		printf( "\n" );

		lora_symbols_count = 0;
		int r = CreateMessageFromPayload( lora_symbols, &lora_symbols_count, MAX_SYMBOLS, SF_NUMBER, 4, raw_payload, raw_payload_size );

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

		#define CODEWORD_SHIFT 3

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
		printf( "--- %d [%d] %d frame %08x\n", lora_symbols_count, quadsetcount, CHIPSSPREAD/4, frame );
		quadsetplace = 0;
	}

}

