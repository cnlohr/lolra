// NOTE: https://github.com/cnlohr/lorasoft/lib/LoRa-SDR-Code.h - IS UPSTREAM VERSION

/* This code is lifted from LoRa-SDR. I found this copyright at the top
 of one of their files:

// Copyright (c) 2016-2016 Lime Microsystems
// Copyright (c) 2016-2016 Arne Hennig
// SPDX-License-Identifier: BSL-1.0
// from https://github.com/myriadrf/LoRa-SDR/blob/master/LoRaEncoder.cpp

Hopefully that covers some of the other stuff.

For any portions of this file not covered under the above licensing terms, it
they shall be governed by the MIT License

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


#ifndef _LORAENCODER_H
#define _LORAENCODER_H


// From LoRaCodes.hpp

#include <string.h>

/***********************************************************************
 * Defines
 **********************************************************************/
#define HEADER_RDD          4
#define N_HEADER_SYMBOLS    (HEADER_RDD + 4)


/***********************************************************************
 * Round functions
 **********************************************************************/
static unsigned roundUp(unsigned num, unsigned factor)
{
    return ((num + factor - 1) / factor) * factor;
}

/***********************************************************************
 * Simple 8-bit checksum routine
 **********************************************************************/
static uint8_t checksum8(const uint8_t *p, const size_t len)
{
    uint8_t acc = 0;
    for (size_t i = 0; i < len; i++)
    {
        acc = (acc >> 1) + ((acc & 0x1) << 7); //rotate
        acc += p[i]; //add
    }
    return acc;
}

static uint8_t headerChecksum(const uint8_t *h) {
	int a0 = (h[0] >> 4) & 0x1;
	int a1 = (h[0] >> 5) & 0x1;
	int a2 = (h[0] >> 6) & 0x1;
	int a3 = (h[0] >> 7) & 0x1;

	int b0 = (h[0] >> 0) & 0x1;
	int b1 = (h[0] >> 1) & 0x1;
	int b2 = (h[0] >> 2) & 0x1;
	int b3 = (h[0] >> 3) & 0x1;

	int c0 = (h[1] >> 0) & 0x1;
	int c1 = (h[1] >> 1) & 0x1;
	int c2 = (h[1] >> 2) & 0x1;
	int c3 = (h[1] >> 3) & 0x1;

	uint8_t res;
	res = (a0 ^ a1 ^ a2 ^ a3) << 4;
	res |= (a3 ^ b1 ^ b2 ^ b3 ^ c0) << 3;
	res |= (a2 ^ b0 ^ b3 ^ c1 ^ c3) << 2;
	res |= (a1 ^ b0 ^ b2 ^ c0 ^ c1 ^ c2) << 1;
	res |= a0 ^ b1 ^ c0 ^ c1 ^ c2 ^ c3;
	
	return res;
}

static uint16_t crc16sx(uint16_t crc, const uint16_t poly) {
	for (int i = 0; i < 8; i++) {
		if (crc & 0x8000) {
			crc = (crc << 1) ^ poly;
		}
		else {
			crc <<= 1;
		}
	}
	return crc;
}

static uint8_t xsum8(uint8_t t) {
	t ^= t >> 4;
	t ^= t >> 2;
	t ^= t >> 1;
	return (t & 1);
}

/***********************************************************************
 *  CRC reverse engineered from Sx1272 data stream.
 *  Modified CCITT crc with masking of the output with an 8bit lfsr
 **********************************************************************/
static uint16_t sx1272DataChecksum(const uint8_t *data, int length) {
	uint16_t res = 0;
	uint8_t v = 0xff;
	uint16_t crc = 0;
	for (int i = 0; i < length; i++) {
		crc = crc16sx(res, 0x1021);
		v = xsum8(v & 0xB8) | (v << 1);
		res = crc ^ data[i];
	}
	res ^= v; 
	v = xsum8(v & 0xB8) | (v << 1);
	res ^= v << 8;
	return res;
}


/***********************************************************************
 *  http://www.semtech.com/images/datasheet/AN1200.18_AG.pdf
 **********************************************************************/
static void SX1232RadioComputeWhitening( uint8_t *buffer, uint16_t bufferSize )
{
    uint8_t WhiteningKeyMSB; // Global variable so the value is kept after starting the
    uint8_t WhiteningKeyLSB; // de-whitening process
    WhiteningKeyMSB = 0x01; // Init value for the LFSR, these values should be initialize only
    WhiteningKeyLSB = 0xFF; // at the start of a whitening or a de-whitening process
    // *buffer is a char pointer indicating the data to be whiten / de-whiten
    // buffersize is the number of char to be whiten / de-whiten
    // >> The whitened / de-whitened data are directly placed into the pointer
    uint8_t i = 0;
    uint16_t j = 0;
    uint8_t WhiteningKeyMSBPrevious = 0; // 9th bit of the LFSR
    for( j = 0; j < bufferSize; j++ )     // byte counter
    {
        buffer[j] ^= WhiteningKeyLSB;   // XOR between the data and the whitening key
        for( i = 0; i < 8; i++ )    // 8-bit shift between each byte
        {
            WhiteningKeyMSBPrevious = WhiteningKeyMSB;
            WhiteningKeyMSB = ( WhiteningKeyLSB & 0x01 ) ^ ( ( WhiteningKeyLSB >> 5 ) & 0x01 );
            WhiteningKeyLSB= ( ( WhiteningKeyLSB >> 1 ) & 0xFF ) | ( ( WhiteningKeyMSBPrevious << 7 ) & 0x80 );
        }
    }
}


/***********************************************************************
 *  Whitening generator reverse engineered from Sx1272 data stream.
 *  Each bit of a codeword is combined with the output from a different position in the whitening sequence.
 **********************************************************************/
static void Sx1272ComputeWhitening(uint8_t *buffer, uint16_t bufferSize, const int bitOfs, const int RDD) {
	static const int ofs0[8] = {6,4,2,0,-112,-114,-302,-34 };	// offset into sequence for each bit
	static const int ofs1[5] = {6,4,2,0,-360 };					// different offsets used for single parity mode (1 == RDD)
	static const int whiten_len = 510;							// length of whitening sequence
	static const uint64_t whiten_seq[8] = {						// whitening sequence
		0x0102291EA751AAFFL,0xD24B050A8D643A17L,0x5B279B671120B8F4L,0x032B37B9F6FB55A2L,
		0x994E0F87E95E2D16L,0x7CBCFC7631984C26L,0x281C8E4F0DAEF7F9L,0x1741886EB7733B15L
	};
	const int *ofs = (1 == RDD) ? ofs1 : ofs0;
	int i, j;
	for (j = 0; j < bufferSize; j++) {
		uint8_t x = 0;
		for (i = 0; i < 4 + RDD; i++) {
			int t = (ofs[i] + j + bitOfs + whiten_len) % whiten_len;
			if (whiten_seq[t >> 6] & ((uint64_t)1 << (t & 0x3F))) {
				x |= 1 << i;
			}
		}
		buffer[j] ^= x;
	}	
}

/***********************************************************************
 *  Whitening generator reverse engineered from Sx1272 data stream.
 *  Same as above but using the actual interleaved LFSRs.
 **********************************************************************/
static void Sx1272ComputeWhiteningLfsr(uint8_t *buffer, uint16_t bufferSize, const int bitOfs, const size_t RDD) {
    static const uint64_t seed1[2] = {0x6572D100E85C2EFF,0xE85C2EFFFFFFFFFF};   // lfsr start values
    static const uint64_t seed2[2] = {0x05121100F8ECFEEF,0xF8ECFEEFEFEFEFEF};   // lfsr start values for single parity mode (1 == RDD)
    const uint8_t m = 0xff >> (4 - RDD);
    uint64_t r[2] = {(1 == RDD)?seed2[0]:seed1[0],(1 == RDD)?seed2[1]:seed1[1]};
    int i,j;
    for (i = 0; i < bitOfs;i++){
        r[i & 1] = (r[i & 1] >> 8) | (((r[i & 1] >> 32) ^ (r[i & 1] >> 24) ^ (r[i & 1] >> 16) ^ r[i & 1]) << 56);   // poly: 0x1D
    }
    for (j = 0; j < bufferSize; j++,i++) {
        buffer[j] ^= r[i & 1] & m;
        r[i & 1] = (r[i & 1] >> 8) | (((r[i & 1] >> 32) ^ (r[i & 1] >> 24) ^ (r[i & 1] >> 16) ^ r[i & 1]) << 56);
    }	
}

/***********************************************************************
 *  https://en.wikipedia.org/wiki/Gray_code
 **********************************************************************/

/*
 * This function converts an unsigned binary
 * number to reflected binary Gray code.
 *
 * The operator >> is shift right. The operator ^ is exclusive or.
 */
static unsigned short binaryToGray16(unsigned short num)
{
    return num ^ (num >> 1);
}

/*
 * A more efficient version, for Gray codes of 16 or fewer bits.
 */
static unsigned short grayToBinary16(unsigned short num)
{
    num = num ^ (num >> 8);
    num = num ^ (num >> 4);
    num = num ^ (num >> 2);
    num = num ^ (num >> 1);
    return num;
}

/***********************************************************************
 * Encode a 4 bit word into a 8 bits with parity
 * Non standard version used in sx1272.
 * https://en.wikipedia.org/wiki/Hamming_code
 **********************************************************************/
static unsigned char encodeHamming84sx(const unsigned char x)
{
    int d0 = (x >> 0) & 0x1;
    int d1 = (x >> 1) & 0x1;
    int d2 = (x >> 2) & 0x1;
    int d3 = (x >> 3) & 0x1;
    
    int b = x & 0xf;
    b |= (d0 ^ d1 ^ d2) << 4;
    b |= (d1 ^ d2 ^ d3) << 5;
    b |= (d0 ^ d1 ^ d3) << 6;
    b |= (d0 ^ d2 ^ d3) << 7;
    return b;
}

/***********************************************************************
 * Decode 8 bits into a 4 bit word with single bit correction.
 * Non standard version used in sx1272.
 * Set error to true when a parity error was detected
 * Set bad to true when the result could not be corrected
 **********************************************************************/
static unsigned char decodeHamming84sx(const unsigned char b, int * error, int * bad)
{
    int b0 = (b >> 0) & 0x1;
    int b1 = (b >> 1) & 0x1;
    int b2 = (b >> 2) & 0x1;
    int b3 = (b >> 3) & 0x1;
    int b4 = (b >> 4) & 0x1;
    int b5 = (b >> 5) & 0x1;
    int b6 = (b >> 6) & 0x1;
    int b7 = (b >> 7) & 0x1;
    
    int p0 = (b0 ^ b1 ^ b2 ^ b4);
    int p1 = (b1 ^ b2 ^ b3 ^ b5);
    int p2 = (b0 ^ b1 ^ b3 ^ b6);
    int p3 = (b0 ^ b2 ^ b3 ^ b7);
    
    int parity = (p0 << 0) | (p1 << 1) | (p2 << 2) | (p3 << 3);
    if (parity != 0) *error = 1;
    switch (parity & 0xf)
    {
        case 0xD: return (b ^ 1) & 0xf;
        case 0x7: return (b ^ 2) & 0xf;
        case 0xB: return (b ^ 4) & 0xf;
        case 0xE: return (b ^ 8) & 0xf;
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x4:
        case 0x8: return b & 0xf;
        default: *bad = 1; return b & 0xf;
    }
}

/***********************************************************************
 * Encode a 4 bit word into a 7 bits with parity.
 * Non standard version used in sx1272.
 **********************************************************************/
static unsigned char encodeHamming74sx(const unsigned char x)
{
    int d0 = (x >> 0) & 0x1;
    int d1 = (x >> 1) & 0x1;
    int d2 = (x >> 2) & 0x1;
    int d3 = (x >> 3) & 0x1;
    
    unsigned char b = x & 0xf;
    b |= (d0 ^ d1 ^ d2) << 4;
    b |= (d1 ^ d2 ^ d3) << 5;
    b |= (d0 ^ d1 ^ d3) << 6;
    return b;
}

/***********************************************************************
 * Decode 7 bits into a 4 bit word with single bit correction.
 * Non standard version used in sx1272.
 * Set error to true when a parity error was detected
 **********************************************************************/
static unsigned char decodeHamming74sx(const unsigned char b, int * error)
{
    int b0 = (b >> 0) & 0x1;
    int b1 = (b >> 1) & 0x1;
    int b2 = (b >> 2) & 0x1;
    int b3 = (b >> 3) & 0x1;
    int b4 = (b >> 4) & 0x1;
    int b5 = (b >> 5) & 0x1;
    int b6 = (b >> 6) & 0x1;
    
    int p0 = (b0 ^ b1 ^ b2 ^ b4);
    int p1 = (b1 ^ b2 ^ b3 ^ b5);
    int p2 = (b0 ^ b1 ^ b3 ^ b6);
    
    int parity = (p0 << 0) | (p1 << 1) | (p2 << 2);
    if (parity != 0) *error = 1;
    switch (parity)
    {
        case 0x5: return (b ^ 1) & 0xf;
        case 0x7: return (b ^ 2) & 0xf;
        case 0x3: return (b ^ 4) & 0xf;
        case 0x6: return (b ^ 8) & 0xf;
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x4: return b & 0xF;
    }
    return b & 0xf;
}

/***********************************************************************
 * Check parity for 5/4 code.
 * return true if parity is valid.
 **********************************************************************/
static unsigned char checkParity54(const unsigned char b, int *error) {
	int x = b ^ (b >> 2);
	x = x ^ (x >> 1) ^ (b >> 4);
	if (x & 1) *error = 1;
	return b & 0xf;
}

static unsigned char encodeParity54(const unsigned char b) {
	int x = b ^ (b >> 2);
	x = x ^ (x >> 1);
	return (b & 0xf) | ((x << 4) & 0x10);
}

/***********************************************************************
* Check parity for 6/4 code.
* return true if parity is valid.
**********************************************************************/
static unsigned char checkParity64(const unsigned char b, int *error) {
	int x = b ^ (b >> 1) ^ (b >> 2);
	int y = x ^ b ^ (b >> 3);
	
	x ^= b >> 4;
	y ^= b >> 5;
	if ((x | y) & 1) *error = 1;
	return b & 0xf;
}

static unsigned char encodeParity64(const unsigned char b) {
	int x = b ^ (b >> 1) ^ (b >> 2);
	int y = x ^ b ^ (b >> 3);
	return ((x & 1) << 4) | ((y & 1) << 5) | (b & 0xf);
}

/***********************************************************************
 * Diagonal interleaver + deinterleaver
 **********************************************************************/
static void diagonalInterleaveSx(const uint8_t *codewords, const size_t numCodewords, uint16_t *symbols, const size_t PPM, const size_t RDD){
	for (size_t x = 0; x < numCodewords / PPM; x++)	{
		const size_t cwOff = x*PPM;
		const size_t symOff = x*(4 + RDD);
		for (size_t k = 0; k < 4 + RDD; k++){
			uint16_t s = symbols[symOff + k];
			for (size_t m = 0; m < PPM; m++){
				const size_t i = (m + k + PPM) % PPM;
				const int bit = (codewords[cwOff + i] >> k) & 0x1;
				s |= (bit << m);
			}
			symbols[symOff + k] = s;
		}
	}
}

static void diagonalDeterleaveSx(const uint16_t *symbols, const size_t numSymbols, uint8_t *codewords, const size_t PPM, const size_t RDD)
{
	for (size_t x = 0; x < numSymbols / (4 + RDD); x++)
	{
		const size_t cwOff = x*PPM;
		const size_t symOff = x*(4 + RDD);
		for (size_t k = 0; k < 4 + RDD; k++)
		{
			for (size_t m = 0; m < PPM; m++)
			{
				const size_t i = (m + k) % PPM;
				const int bit = (symbols[symOff + k] >> m) & 0x1;
				codewords[cwOff + i] |= (bit << k);
			}
		}
	}
}

static void diagonalDeterleaveSx2(const uint16_t *symbols, const size_t numSymbols, uint8_t *codewords, const size_t PPM, const size_t RDD)
{
	size_t nb = RDD + 4;
	for (size_t x = 0; x < numSymbols / nb; x++) {
		const size_t cwOff = x*PPM;
		const size_t symOff = x*nb;
		for (size_t m = 0; m < PPM; m++) {
			size_t i = m;
			int sym = symbols[symOff + m];
			for (size_t k = 0; k < PPM; k++, sym >>= 1) {
				codewords[cwOff + i] |= (sym & 1) << m;
				if (++i == PPM) i = 0;
			}
		}
	}
}



static void encodeFec(uint8_t  * codewords, const size_t RDD, size_t * cOfs, size_t * dOfs, const uint8_t *bytes, const size_t count)
{
	if (RDD == 0) for (size_t i = 0; i < count; i++, (*dOfs)++) {
		if ((*dOfs) & 1)
			codewords[(*cOfs)++] = bytes[(*dOfs) >> 1] >> 4;
		else
			codewords[(*cOfs)++] = bytes[(*dOfs) >> 1] & 0xf;
	} else if (RDD == 1) for (size_t i = 0; i < count; i++, (*dOfs)++) {
		if ((*dOfs) & 1)
			codewords[(*cOfs)++] = encodeParity54(bytes[(*dOfs) >> 1] >> 4);
		else
			codewords[(*cOfs)++] = encodeParity54(bytes[(*dOfs) >> 1] & 0xf);
	} else if (RDD == 2) for (size_t i = 0; i < count; i++, (*dOfs)++) {
		if ((*dOfs) & 1)
			codewords[(*cOfs)++] = encodeParity64(bytes[(*dOfs) >> 1] >> 4);
		else
			codewords[(*cOfs)++] = encodeParity64(bytes[(*dOfs) >> 1] & 0xf);
	} else if (RDD == 3) for (size_t i = 0; i < count; i++, (*dOfs)++) {
		if ((*dOfs) & 1)
			codewords[(*cOfs)++] = encodeHamming74sx(bytes[(*dOfs) >> 1] >> 4);
		else
			codewords[(*cOfs)++] = encodeHamming74sx(bytes[(*dOfs) >> 1] & 0xf);
	} else if (RDD == 4) for (size_t i = 0; i < count; i++, (*dOfs)++) {
		if ((*dOfs) & 1)
			codewords[(*cOfs)++] = encodeHamming84sx(bytes[(*dOfs) >> 1] >> 4);
		else
			codewords[(*cOfs)++] = encodeHamming84sx(bytes[(*dOfs) >> 1] & 0xf);
	}
}

// This function has been HEAVILY modified from the original.
// Note that there are references to SF6, 11 and 12, but they do not currently work.

static int CreateMessageFromPayload( uint16_t * symbols, int * symbol_out_count, int max_symbols, int _sf, int _rdd, uint8_t * payload_plus_two_extra_crc_bytes, int payload_length )
{
	//static int uctr = 0;

	int _whitening = 1; // Enable whitening
	int _crc = 1; // Enable CRC.

	size_t PPM = _sf;

	int _explicit = 1;

	// TODO: https://dl.acm.org/doi/fullHtml/10.1145/3546869#sec-9 -- what about the corner cases for SF6, etc.

	// SF6 does NOT WORK
	int nHeaderCodewords = 0;
	if( _sf == 6 ) nHeaderCodewords = 6;
	if( _sf == 7 ) nHeaderCodewords = 5; // CORRECT VALIDATED
	if( _sf == 8 ) nHeaderCodewords = 6; // CORRECT VALIDATED
	if( _sf == 9 ) nHeaderCodewords = 7; // CORRECT VALIDATED
	if( _sf == 10 ) nHeaderCodewords = 8; // CORRECT VALIDATED
	if( _sf == 11 ) nHeaderCodewords = 9;  // ???? Probably Wrong
	if( _sf == 12 ) nHeaderCodewords = 10;  // ???? Probably Wrong

	int header_ppm_shift_up_by = 2;
	// SF6 still isn't working. 
	// header does not reduce SF on SF <= 6 https://github.com/tapparelj/gr-lora_sdr/compare/master...feature/sf5_6_sx126x#diff-1821161335c7f28236eb88e3b8a3c84cce6997861cd847e87fbc9e270d338a37R68
	// Indirectly, to note, for SF > 6, the header is implicitly LDRO (the bottom 2 bits are ignored)
	if( _sf < 7 ) header_ppm_shift_up_by = 0;

	int extra_codewords_due_to_header_padding = ( _sf == 7 || _sf == 6 /* CHECKME */ ) ? 1 : 0;

	int header_ppm = ( _sf - header_ppm_shift_up_by );
	int data_ppm = _sf;

	// XXX TODO: Investigate: I thought SF12 had an LDRO mode which made the PPM only 10.
	// TODO: Compare to https://github.com/jkadbear/LoRaPHY/blob/master/LoRaPHY.m

	const size_t numCodewords = roundUp( ( payload_length + 2 * _crc ) * 2 + (_explicit ? nHeaderCodewords:0) + extra_codewords_due_to_header_padding, PPM);
	const size_t numSymbols = N_HEADER_SYMBOLS + (numCodewords / PPM - 1) * (4 + _rdd);		// header is always coded with 8/4
	uint8_t codewords[numCodewords];
	memset( codewords, 0, sizeof( codewords ) );

	if( numSymbols >= max_symbols )
	{
		//uprintf( "Error: Too many symbols to fit (%d/%d)\n", numSymbols, max_symbols );
		return -1;
	}

	memset( symbols, 0, numSymbols * sizeof( symbols[0] ) );

	size_t cOfs = 0;
	size_t dOfs = 0;

	//std::vector<uint8_t> codewords(numCodewords);
	if (_crc)
	{
		uint16_t crc = sx1272DataChecksum( payload_plus_two_extra_crc_bytes, payload_length );
		payload_plus_two_extra_crc_bytes[payload_length] = crc & 0xff;
		payload_plus_two_extra_crc_bytes[payload_length+1] = (crc >> 8) & 0xff;
	}

	// Why does this disagree? https://www.Carloalbertoboano.Com/Documents/Yang22emu.Pdf  
	if (_explicit) {
		uint8_t hdr[3];
		hdr[0] = payload_length;
		hdr[1] = (_crc ? 1 : 0) | (_rdd << 1);
		//static int k;
		hdr[2] = 
			//k++;
			headerChecksum(hdr);

		codewords[cOfs++] = encodeHamming84sx(hdr[0] >> 4);
		codewords[cOfs++] = encodeHamming84sx(hdr[0] & 0xf);	// length
		codewords[cOfs++] = encodeHamming84sx(hdr[1] & 0xf);	// crc / fec info
		codewords[cOfs++] = encodeHamming84sx(hdr[2] >> 4);		// checksum
		codewords[cOfs++] = encodeHamming84sx(hdr[2] & 0xf);

		if( extra_codewords_due_to_header_padding > 1 )
		{
			codewords[cOfs++] = 0;
		}
		if( extra_codewords_due_to_header_padding > 2 )
		{
			codewords[cOfs++] = 0;
		}
		if( extra_codewords_due_to_header_padding > 3 )
		{
			codewords[cOfs++] = 0;
		}
	}

	size_t cOfs1 = cOfs;

	// Header is encoded at 8/4 (4)
	encodeFec( codewords, 4, &cOfs, &dOfs, payload_plus_two_extra_crc_bytes, PPM - cOfs );

	// Whitening for the data that lives inside the header block.
	if( _whitening )
	{
		Sx1272ComputeWhitening(codewords + cOfs1, PPM - cOfs1, 0, HEADER_RDD);
	}

	if (numCodewords > PPM) {
		size_t cOfs2 = cOfs;

		encodeFec(codewords, _rdd, &cOfs, &dOfs, payload_plus_two_extra_crc_bytes, numCodewords-PPM);

		if (_whitening) {
			Sx1272ComputeWhitening(codewords + cOfs2, numCodewords - PPM, PPM - cOfs1, _rdd);
		}
	}

	//interleave the codewords into symbols
	int symbols_size = numSymbols;

	// TRICKY: Header is forced to a particularly slow mode.
	diagonalInterleaveSx(codewords, header_ppm, symbols, header_ppm, HEADER_RDD);

	int i;

	if (numCodewords > header_ppm) {
		diagonalInterleaveSx(codewords + nHeaderCodewords, numCodewords-nHeaderCodewords, symbols+N_HEADER_SYMBOLS, data_ppm, _rdd);
	}

	//gray decode, when SF > PPM, pad out LSBs
	uint16_t sym;
	for( i = 0; i < symbols_size; i++ )
	{
		//int is_header = (i < 8);
		sym = symbols[i];
		sym = grayToBinary16(sym);
		sym <<= (_sf - PPM);
		symbols[i] = sym; // OR +1
	}

	// The first header symbols are all shifted by 2.
	// XXX TODO Look here for SF6 stuff.
	for( i = 0; i < N_HEADER_SYMBOLS; i++ )
	{
		symbols[i] <<= header_ppm_shift_up_by;
	}

	*symbol_out_count = symbols_size;
	return 0;
}

#endif

