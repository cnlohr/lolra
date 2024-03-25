/* Clean implementation of RFC4493 AES-CMAC in C, Public Domain by cnlohr
 WARNING: This is not cryptographically hardened.
    DO NOT USE THIS IN AN ENVIRONMENT WHERE SECURITY MATTERS
    THIS IMPLEMENTATION IS HIGHLY SUCCEPTABLE TO TIMING ATTACKS
    THIS IMPLEMENTATION DOES NOT USE HARDENED MEMORY.

 You may license this code under the unlicense, MIT-x11 or NewBSD licenses.

	This is free and unencumbered software released into the public domain.

	Anyone is free to copy, modify, publish, use, compile, sell, or
	distribute this software, either in source code form or as a compiled
	binary, for any purpose, commercial or non-commercial, and by any
	means.

	In jurisdictions that recognize copyright laws, the author or authors
	of this software dedicate any and all copyright interest in the
	software to the public domain. We make this dedication for the benefit
	of the public at large and to the detriment of our heirs and
	successors. We intend this dedication to be an overt act of
	relinquishment in perpetuity of all present and future rights to this
	software under copyright law.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
	MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
	IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
	ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
	OTHER DEALINGS IN THE SOFTWARE.
	For more information, please refer to <http://unlicense.org/>

 Assumes a tiny-AES-c implementation for AES_init_ctx and AES_ECB_encrypt.
    #define TINY_AES_ECB 1
    #include "tiny-AES-c.h"
*/

#include <string.h>

static void BlockLeftShift( uint8_t * blockout, const uint8_t * blockin )
{
	int i;
	int carry = 0;
	for( i = 15; i >= 0; i-- )
	{
		uint8_t bi = blockin[i];
		blockout[i] = (bi << 1) | carry;
		carry = !!(bi & 0x80);
	}
}

static void BlockXor( uint8_t * out, const uint8_t * in, const uint8_t * inb )
{
	int i;
	for( i = 0; i < 16; i++ )
		out[i] = in[i] ^ inb[i];
}


static void AES_CMAC( const uint8_t *key, const uint8_t *input, int length,
	uint8_t *mac )
{
	struct AES_ctx K;
	AES_init_ctx( &K, key);

	int i;
	// Based on Figure 2.2, GenerateSubKey
	/*
	   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	   +                    Algorithm Generate_Subkey                      +
	   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	   +                                                                   +
	   +   Input    : K (128-bit key)                                      +
	   +   Output   : K1 (128-bit first subkey)                            +
	   +              K2 (128-bit second subkey)                           +
	   +-------------------------------------------------------------------+
	   +                                                                   +
	   +   Constants: const_Zero is 0x00000000000000000000000000000000     +
	   +              const_Rb   is 0x00000000000000000000000000000087     +
	   +   Variables: L          for output of AES-128 applied to 0^128    +
	   +                                                                   +
	   +   Step 1.  L := AES-128(K, const_Zero);                           +
	   +   Step 2.  if MSB(L) is equal to 0                                +
	   +            then    K1 := L << 1;                                  +
	   +            else    K1 := (L << 1) XOR const_Rb;                   +
	   +   Step 3.  if MSB(K1) is equal to 0                               +
	   +            then    K2 := K1 << 1;                                 +
	   +            else    K2 := (K1 << 1) XOR const_Rb;                  +
	   +   Step 4.  return K1, K2;                                         +
	   +                                                                   +
	   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	*/
	const uint8_t const_Rb[AES_BLOCKLEN] = { 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87 };
	uint8_t K1[AES_BLOCKLEN];
	uint8_t K2[AES_BLOCKLEN];
	{
		uint8_t L[AES_BLOCKLEN] = { 0 };
		AES_ECB_encrypt( &K, L );	

		BlockLeftShift( K1, L );
		if( L[0] & 0x80 )
			BlockXor( K1, K1, const_Rb );

		BlockLeftShift( K2, K1 );
		if( K1[0] & 0x80 )
			BlockXor( K2, K2, const_Rb );
	}

	// Based on figure 2.3, AES-CMAC
	/*
	   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	   +                   Algorithm AES-CMAC                              +
	   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	   +                                                                   +
	   +   Input    : K    ( 128-bit key )                                 +
	   +            : M    ( message to be authenticated )                 +
	   +            : len  ( length of the message in octets )             +
	   +   Output   : T    ( message authentication code )                 +
	   +                                                                   +
	   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	   +   Constants: const_Zero is 0x00000000000000000000000000000000     +
	   +              const_Bsize is 16                                    +
	   +                                                                   +
	   +   Variables: K1, K2 for 128-bit subkeys                           +
	   +              M_i is the i-th block (i=1..ceil(len/const_Bsize))   +
	   +              M_last is the last block xor-ed with K1 or K2        +
	   +              n      for number of blocks to be processed          +
	   +              r      for number of octets of last block            +
	   +              flag   for denoting if last block is complete or not +
	   +                                                                   +
	   +   Step 1.  (K1,K2) := Generate_Subkey(K);                         +
	   +   Step 2.  n := ceil(len/const_Bsize);                            +
	   +   Step 3.  if n = 0                                               +
	   +            then                                                   +
	   +                 n := 1;                                           +
	   +                 flag := false;                                    +
	   +            else                                                   +
	   +                 if len mod const_Bsize is 0                       +
	   +                 then flag := true;                                +
	   +                 else flag := false;                               +
	   +                                                                   +
	   +   Step 4.  if flag is true                                        +
	   +            then M_last := M_n XOR K1;                             +
	   +            else M_last := padding(M_n) XOR K2;                    +
	   +   Step 5.  X := const_Zero;                                       +
	   +   Step 6.  for i := 1 to n-1 do                                   +
	   +                begin                                              +
	   +                  Y := X XOR M_i;                                  +
	   +                  X := AES-128(K,Y);                               +
	   +                end                                                +
	   +            Y := M_last XOR X;                                     +
	   +            T := AES-128(K,Y);                                     +
	   +   Step 7.  return T;                                              +
	   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	*/
	uint8_t M_last[AES_BLOCKLEN];

	int remainder = length % AES_BLOCKLEN;
	int flag = !(remainder); // If evenly divisible, set to true.
	int n = ( length + AES_BLOCKLEN - 1 ) / AES_BLOCKLEN;
	if( n == 0 )
	{
		n = 1;
		flag = 0; // Unless 0 size payload.
	}

	const uint8_t * in_last = &input[(n-1)*AES_BLOCKLEN];
	if( flag )
	{
		BlockXor( M_last, in_last, K1 );
	}
	else
	{
		memcpy( M_last, in_last, remainder );
		memset( M_last + remainder + 1, 0, AES_BLOCKLEN - remainder - 1 );
		M_last[remainder] = 0x80;
		BlockXor( M_last, M_last, K2 );
	}

	uint8_t X[AES_BLOCKLEN] = { 0 };
	for( i = 0; i < n-1; i++ )
	{
		BlockXor( X, X, &input[i*AES_BLOCKLEN] );
		AES_ECB_encrypt( &K, X );
	}

	BlockXor( mac, M_last, X );
	AES_ECB_encrypt( &K, mac );
}



