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

#ifndef _LORAWAN_SIMPLE_H
#define _LORAWAN_SIMPLE_H

#include <string.h>
#include <unistd.h>

// Configuration needed for aes-cbc-cmac.h
#define TINY_AES_C_IMPLEMENTATION

#define TINY_AES_ECB 1
#include "tiny-AES-c.h"
#include "aes-cbc-cmac.h"

static int GenerateLoRaWANPacket( uint8_t * raw_payload_with_b0, const uint8_t * inner_payload_raw, int inner_payload_len, const uint8_t * payload_key, const uint8_t * network_skey, const uint8_t * devaddress, int frame )
{
	int i;
	uint8_t * raw_payload = raw_payload_with_b0 + 16;
	uint8_t * pl = raw_payload;

	struct AES_ctx payload_key_ctx;
	AES_init_ctx( &payload_key_ctx, payload_key );

	// MAC header
	// 010 Unconfirmed data uplink.
	// 000 RFU
	//  00 Major Version
	*(pl++) = ( 0b01000000 );


	// Frame header
	// Device address (4 bytes) 
	memcpy( pl, devaddress, 4 );
	pl += 4;

	// Frame Control
	// MSB
	//  0 = ADR (cannot do rate adaptation)
	//  0 = Not confirming ADR.
	//  0 = ACK
	//  0 = Class B (we are not a class B device because we can't schedule downlinks)
	// 0000 = fopts length (no fopts)
	// LSB
	*(pl++) = 0b00000000;

	// Frame Count
	*(pl++) = frame;
	*(pl++) = frame>>8;

	// Frame Port
	*(pl++) = 1; // Port (1-223 are application specific)


	int padded_pad_length = ( inner_payload_len + AES_BLOCKLEN - 1 ) & (~(AES_BLOCKLEN-1));
	int nr_blocks = padded_pad_length / AES_BLOCKLEN;
	uint8_t pad[padded_pad_length];
	for( i = 0; i < nr_blocks; i++ )
	{
		// Add CMAC. Construct b0
		uint8_t * Ai = &pad[i*AES_BLOCKLEN];
		*(Ai++) = 0x01;
		*(Ai++) = 0;
		*(Ai++) = 0;
		*(Ai++) = 0;
		*(Ai++) = 0;

		*(Ai++) = 0; // Uplink = 0

		*(Ai++) = devaddress[0];
		*(Ai++) = devaddress[1];
		*(Ai++) = devaddress[2];
		*(Ai++) = devaddress[3];

		*(Ai++) = (frame>>0) & 0xff;
		*(Ai++) = (frame>>8) & 0xff;
		*(Ai++) = (frame>>16) & 0xff;
		*(Ai++) = (frame>>24) & 0xff;
		*(Ai++) = 0;

		*(Ai++) = i+1;
	}
	for( i = 0; i < nr_blocks; i++ )
	{
		AES_ECB_encrypt( &payload_key_ctx, &pad[i*AES_BLOCKLEN] );
	}

//	uint8_t * start_of_payload = pl;

	for( i = 0; i < inner_payload_len; i++ )
	{
		*(pl++) = pad[i] ^ inner_payload_raw[i];
	}

	int to_cmac_size = pl - raw_payload_with_b0;
	int length_of_frame_without_b0 = pl - raw_payload;

	uint8_t * b0 = raw_payload_with_b0;
	*(b0++) = 0x49;
	*(b0++) = 0x00;
	*(b0++) = 0x00;
	*(b0++) = 0x00;
	*(b0++) = 0x00;
	*(b0++) = 0x00;
	
	*(b0++) = devaddress[0];
	*(b0++) = devaddress[1];
	*(b0++) = devaddress[2];
	*(b0++) = devaddress[3];

	*(b0++) = (frame>>0) & 0xff;
	*(b0++) = (frame>>8) & 0xff;
	*(b0++) = (frame>>16) & 0xff;
	*(b0++) = (frame>>24) & 0xff;
	*(b0++) = 0x00;
	*(b0++) = length_of_frame_without_b0;

	uint8_t mac[AES_BLOCKLEN];
	//printf( "To CMAC: %d\n", to_cmac_size );
	//for ( i = 0; i <  to_cmac_size; i++ )
	//{
	//	printf( "%02x ", raw_payload_with_b0[i] );
	//}
	//printf( "\n" );

	AES_CMAC( network_skey, raw_payload_with_b0, to_cmac_size, mac );

	*(pl++) = mac[0];
	*(pl++) = mac[1];
	*(pl++) = mac[2];
	*(pl++) = mac[3];

	//printf( "MAC %02x%02x%02x%02x  %02x%02x%02x%02x\n", mac[0], mac[1], mac[2], mac[3], mac[12], mac[13], mac[14], mac[15] );

	return pl - raw_payload_with_b0 - 16;
}

#endif
