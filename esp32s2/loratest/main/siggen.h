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


#ifndef _SIGGEN_H
#define _SIGGEN_H

#include <string.h>
#include "../../../lib/LoRa-SDR-Code.h"

#define SF_NUMBER 10

// This is MEGA overkill.  No way would we ever use them all.
#define MAX_SYMBOLS 2070

// https://electronics.stackexchange.com/questions/278192/understanding-the-relationship-between-lora-chips-chirps-symbols-and-bits
// Has some good hints

// At 125kHz
// https://medium.com/@prajzler/what-is-lora-the-fundamentals-79a5bb3e6dec
// With SF5, You get 3906 chips/second "Symbol Rate" (Included to help with math division)
// With SF7, You get 976 chips/second   / 5469 bits/sec

// 7 bits per symbol (I think)
// 7 * 4/5 = 5.6 data bits per symbol.
// https://wirelesspi.com/understanding-lora-phy-long-range-physical-layer/ says 7 for SF7

#define MARK_FROM_SF0 (1<<SF_NUMBER)

// Determined experimentally, but this is the amount you have to divide the chip by to
// Fully use the 125000 Hz channel Bandwidth.
//#define DESPREAD (50*MARK_FROM_SF6)

#define CHIPRATE 8 // chirp length for SF0, in us
#define CHIPSSPREAD ((uint32_t)(240ULL*MARK_FROM_SF0*CHIPRATE))

#define PREAMBLE_CHIRPS 10
#define CODEWORD_LENGTH 2
int     symbols_len = 1;
uint16_t symbols[MAX_SYMBOLS];

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

static void SigSetupTest()
{
	memset( symbols, 0, sizeof( symbols ) );

	uint8_t payload_in[258] = { 0x40/*0x48*/, 0xcc/*0x45*/, 0xde, 0x55, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22}; 
	int payload_in_size = 6;
	static uint8_t uctr;
	payload_in[4] = uctr++;
	int _rdd = 0; // 1 = 4/5, 4 = 4/8 Coding Rate

	int r = CreateMessageFromPayload( symbols, &symbols_len, MAX_SYMBOLS, SF_NUMBER, 4, payload_in, payload_in_size );

	if( r < 0 )
	{
		printf( "Error generating stream: %d\n", r );
		quadsetcount = 0;
		return;
	}

	int j;
	//for( j = 0; j < symbols_len; j++ )
	//	symbols[j] = 255 - symbols[j];

	quadsetcount = 0;
	int32_t * qso = quadsets;
	for( j = 0; j < PREAMBLE_CHIRPS; j++ )
	{
		qso = AddChirp( qso, 0, 0 );
	}

	uint8_t syncword = 0x43;

#if SF_NUMBER <= 6
	#define CODEWORD_SHIFT 2 // XXX TODO: No idea what this would do here! XXX This is probably wrong.
#elif SF_NUMBER >= 11
	#define CODEWORD_SHIFT 5 // XXX TODO: Unknown for SF11, SF12 Might be 3?
#else
	#define CODEWORD_SHIFT 3
#endif

	if( CODEWORD_LENGTH > 0 )
		qso = AddChirp( qso,  ( ( syncword & 0xf ) << CODEWORD_SHIFT ), 0 );
	if( CODEWORD_LENGTH > 1 )
		qso = AddChirp( qso, ( ( ( syncword & 0xf0 ) >> 4 ) << CODEWORD_SHIFT ), 0 );


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
		{
			qso = AddChirp( qso, 0, 0 );
		}
	}

	for( j = 0; j < symbols_len; j++ )
	{
		int ofs = symbols[j];
		//ofs = ofs ^ ((MARK_FROM_SF6<<6) -1);
		//ofs &= (MARK_FROM_SF6<<6) -1;
		qso = AddChirp( qso, ofs, 0 );
	}
	
	quadsetcount = qso - quadsets;
//	printf( "--- %d %d %d\n", symbols_len, quadsetcount, CHIPSSPREAD/4 );
}

static int32_t SigGen( uint32_t Frame240MHz, uint32_t codeTarg )
{
	// TODO: Get some of these encode things going: https://github.com/myriadrf/LoRa-SDR/blob/master/LoRaCodes.hpp

	// frame = 0...240000000
	uint32_t sectionQuarterNumber = Frame240MHz / (CHIPSSPREAD/4);
	if( sectionQuarterNumber >= quadsetcount )
		return -codeTarg;

	int32_t quadValue = quadsets[sectionQuarterNumber];
	uint32_t placeInQuad = Frame240MHz % (CHIPSSPREAD/4);

	if( quadValue >= 0 )
		return ( ( quadValue + placeInQuad ) % CHIPSSPREAD); // Up-Chirp
	else
		return ( ( quadValue - placeInQuad + CHIPSSPREAD ) % CHIPSSPREAD ); // Down-Chirp

#if 0
	// Let's say 1ms per sweep.
	int32_t sectionQuarterNumber = Frame240MHz / (CHIPSSPREAD/4);
	uint32_t placeInSweep = Frame240MHz % CHIPSSPREAD;
	// 2400 edge-to-edge = 
	if( sectionQuarterNumber < 0 ) return -codeTarg;

	sectionQuarterNumber -= PREAMBLE_CHIRPS*4;

	// Preamble Start
	if( sectionQuarterNumber < 0 )
	{
		return ((placeInSweep /*+ 240000/2*/) % CHIPSSPREAD) / DESPREAD;			
	}

	// Last 2 codes here are for the sync word.

#define SYNC_WORD
#ifdef SYNC_WORD
	// https://static1.squarespace.com/static/54cecce7e4b054df1848b5f9/t/57489e6e07eaa0105215dc6c/1464376943218/Reversing-Lora-Knight.pdf
	// Says that this does not exist.  but, it does seem to exist in some of their waterfalls. 
	sectionQuarterNumber -= 4*2;
	// Two symbols
	if( sectionQuarterNumber < 0 )
	{
		int32_t  chirp = (8+sectionQuarterNumber)/4;
		uint32_t SYNCWORD = (((0x34)>>(4-chirp*4)) & 0xf)<<3; //0x34 for some vendors?
		int32_t  offset = SYNCWORD * CHIPSSPREAD / (MARK_FROM_SF7*128);
		return (( placeInSweep + offset) % CHIPSSPREAD) / DESPREAD;			
	}
#endif
/*

	EXTRA NOTES: lora-grc says to look for sync word 18
*/



	sectionQuarterNumber -= 9;

	if( sectionQuarterNumber < 0 )
	{
		// Down-Sweeps for Sync
		return ((CHIPSSPREAD-placeInSweep/*+240000/2*/) % CHIPSSPREAD) / DESPREAD;				
	}

	uint32_t chirp = (sectionQuarterNumber)/4;
	if( chirp < symbols_len )
	{
		placeInSweep += (CHIPSSPREAD*3/4);
		uint32_t offset = ( symbols[chirp] + 0 ) * CHIPSSPREAD / (MARK_FROM_SF7*128);
		fplv = ((placeInSweep + offset) % CHIPSSPREAD) / DESPREAD;
		return fplv;
	}
	else
	{
		return -codeTarg;
		//return -1;
	}
#endif
}

/*
  A real semtech SX1276 will produce 16 symbols at SF8 if
  Given 2 byte payload, CRC explicit header.
  Given 3 bytes of payload, that goes up to 24 symbols.

	[HEADER] [HEADER*0.5] [HEADER] **An extra byte** [PAYLOAD] [PAYLOAD] [PAYLOAD] [CRC] [CRC] << Does not fit.

	Payload sizes:
		-5 would be 8       (Considering CRC would be -3 bytes)
		-4 would be 8
		-3 would be 8
		-2 would be 8
		-1 would be 16
		0 would be 16
		1 byte: 16 symbols. (Include CRC so 3 bytes)
		2 byte: 16 symbols. (Include CRC so 4 bytes)
			// theoreteically, 2 bytes  + 2 crc should fit nicely into 16 symbols (or 8 bytes) BUT 3+2 (5) does not.

		3 byte: 24 symbols. (Include CRC so 5 bytes)
		4 byte: 24 symbols
		5 byte: 24 symbols.
		6 byte: 24 symbols.
		7 byte: 32 symbols.
		8 byte: 32 symbols.
		9 byte: 32 symbols.
		10 byte: 32 symbols.
		11 byte: 40 symbols.

	That means there's 

*/

#endif

