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

#include "ch32v003fun.h"
#include <stdio.h>
#include <string.h>
#include "chirpbuff.h"
#include "chirpbuffinfo.h"

#include "../../lib/LoRa-SDR-Code.h"

// NOTE: The SPI engine in the 203 isn't perfect, and I wasn't able to get the I2S engine working.

// Optionally send LoRaWAN messages.  Be sure to copy your keys from the things network.
#define LORAWAN

#ifdef LORAWAN
#include "lorawan_simple.h"
		static const uint8_t payload_key[AES_BLOCKLEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // AppSKey Big Endian
		static const uint8_t network_skey[AES_BLOCKLEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // NwkSKey Big Endian
		static const uint8_t devaddress[4] = { 0x00, 0x00, 0x00, 0x00 }; // Device address Little Endian LSB (Written backwards from Device address default view)
#endif


#define SENDBUFF_WORDS (DMA_SIZE_WORDS*2)
uint16_t sendbuff[SENDBUFF_WORDS];

#if !defined( FOUND_PERFECT_DIVISOR )
#error Code only written for perfect division right now.
#endif

// Bits are shifted out MSBit first, then to LSBit


#define MAX_BYTES 60
#define MAX_SYMBOLS (MAX_BYTES*2+16)

// Our table is bespoke for the specific SF.
#define CHIPSSPREAD CHIRPLENGTH_WORDS// QUARTER_CHIRP_LENGTH_WORDS (TODO: Use the quater value elsewhere in the code)
#define MARK_FROM_SF0 (1<<SF_NUMBER) // SF7

#define PREAMBLE_CHIRPS 10
#define CODEWORD_LENGTH 2

uint32_t quadsetcount;
int16_t quadsets[MAX_SYMBOLS*4+PREAMBLE_CHIRPS*4+9+CODEWORD_LENGTH*4];
int runningcount_bits = 0;
volatile int fxcycle;
volatile int quadsetplace = -1;

int16_t * AddChirp( int16_t * qso, int offset, int verneer )
{
	offset = offset * CHIPSSPREAD / (MARK_FROM_SF0);
	offset += verneer;
	*(qso++) = (CHIPSSPREAD * 0 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	*(qso++) = (CHIPSSPREAD * 1 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	*(qso++) = (CHIPSSPREAD * 2 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	*(qso++) = (CHIPSSPREAD * 3 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	return qso;
}

// This IRQ is called peridocally to fill the buffer that is going to the DMA.
void DMA1_Channel5_IRQHandler( void ) __attribute__((interrupt)) __attribute__((section(".srodata")));
void DMA1_Channel5_IRQHandler( void ) 
{
	//GPIOD->BSHR = 1;	 // Turn on GPIOD0 for profiling

	// Backup flags.
	volatile int intfr = DMA1->INTFR;
	do
	{
		// Clear all possible flags.
		DMA1->INTFCR = DMA1_IT_GL5;


		int place = DMA1_Channel5->CNTR;
		// CNTR says that there are THIS MANY bytes left in the transfer.
		// So a high CNTR value indicates a very early place in the buffer.

		uint16_t * sb = 0;

		//if( intfr & DMA1_IT_HT5 )
		if( place < SENDBUFF_WORDS/2 )
		{
			sb = sendbuff + SENDBUFF_WORDS/2;
		}
		//else if( intfr & DMA1_IT_TC5 )
		else
		{
			sb = sendbuff;
		}

		if( sb )
		{
			if( quadsetplace < 0 )
			{
				// Abort Send
				memset( sb, 0, SENDBUFF_WORDS*2/2 );  //in bytes, but only half the buffer.
				DMA1_Channel5->CFGR &= ~DMA_Mode_Circular; // disable transfer.
				goto complete;
			}

			if( 0 ) // Generate test tone
			{
				int remain = SENDBUFF_WORDS/2;
				do
				{
					uint16_t v = chirpbuff[quadsetplace++];
					if( quadsetplace >= QUARTER_CHIRP_LENGTH_WORDS*4 ) quadsetplace = 0;
					*(sb++) = v;
				} while( --remain );
			}
			else
			{
				fxcycle += DMA_SIZE_WORDS;
				if( fxcycle == NUM_DMAS_PER_QUARTER_CHIRP*DMA_SIZE_WORDS )
				{
					fxcycle = 0;
					// Advance to next quarter word.
					quadsetplace++;

					if( quadsetplace > quadsetcount )
					{
						quadsetplace = -1;
						memset( sb, 0, SENDBUFF_WORDS*2/2 );
						goto complete;
					}
				}
				
				int symbol = quadsets[quadsetplace]; // Actually 0...CHIRPLENGTHWORDS

				const uint16_t * tsb = 0;

				// Select down- or up-chirp.
				if( symbol < 0 )
				{
					int word = fxcycle - symbol - 1;
					if( word >= CHIRPLENGTH_WORDS ) word -= CHIRPLENGTH_WORDS;
					word++;
					tsb = (&chirpbuff[word+REVERSE_START_OFFSET_BYTES/2]);
				}
				else
				{
					int word = fxcycle + symbol;
					if( word >= CHIRPLENGTH_WORDS ) word -= CHIRPLENGTH_WORDS;
					tsb = (&chirpbuff[word]);
				}

				int cpy = DMA_SIZE_WORDS;

				// Very fast half-word-copy.
				// Make sure the data is aligned to groups-of-two bytes.
				if( cpy & 1 )
				{
					*(sb++) = *(tsb++);
					cpy--;
				}

				cpy /= 2; // Doubled up per loop

				asm volatile("\
					1:\
					lh a3, 0(%[from])\n\
					lh a4, 2(%[from])\n\
					c.addi %[from], 4\n\
					c.addi %[cpy], -1\n\
					sh a3, 0(%[to])\n\
					sh a4, 2(%[to])\n\
					c.addi %[to], 4\n\
					c.bnez %[cpy], 1b\n\
					" : [cpy]"+r"(cpy), [from]"+r"(tsb), [to]"+r"(sb) : : "a3", "a4", "memory");

//				while( cpy-- ) { *(sb++) = *(tsb++); } // Doesn't help.  GCC Still tries to use memcpy.
				//memcpy( sb, tsb, DMA_SIZE_WORDS * 2 );
			}
		}

complete:
		intfr = DMA1->INTFR;
	} while( intfr );

	//GPIOD->BSHR = 1<<16; // Turn off GPIOD0 for profiling
}


int main()
{
	SystemInit();

	funGpioInitAll();

	// Force HCLK to be nodiv. ch32vfun sets it to be a reasonbly high div.
	RCC->CFGR0 &= ~RCC_PPRE1_DIV16;

// MCO for testing.
//	funPinMode( PA8, GPIO_CFGLR_OUT_50Mhz_AF_PP );	RCC->CFGR0 |= RCC_CFGR0_MCO_PLL;

	printf( "Switching to HSE\n" );
	Delay_Ms( 100 );

	// Disable clock security system.
	RCC->CTLR &= ~RCC_CSSON;

#ifdef USE_EXTERNAL_CLOCK
	// No crystal - use clock.
	RCC->CTLR |= RCC_HSEBYP;
#endif

	// Enable external crystal
	RCC->CTLR |= RCC_HSEON;

	// Set System Clock Source to be 0.
	RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | 0;

	// Disable PLL
	RCC->CTLR &= ~RCC_PLLON;


#ifdef USE_EXTERNAL_CLOCK
	// Set PLL to 9x not 18x
	RCC->CFGR0 = ( RCC->CFGR0 & ~RCC_PLLMULL18 ) | RCC_PLLMULL9;
#endif

	// Switch to HSE
	RCC->CFGR0 |= RCC_PLLSRC;

	// Enable PLL
	RCC->CTLR |= RCC_PLLON;

	// Wait for HSE to become ready.
	while( !( RCC->CTLR & RCC_HSERDY) );
	while( !( RCC->CTLR & RCC_PLLRDY) );
	RCC->CFGR0 |= RCC_SW_1; //  Switch system clock to PLL
	printf( "HSE Switched\n" );
	Delay_Ms( 10 );
	RCC->CTLR &= ~RCC_HSION;
	Delay_Ms( 10 );
	printf( "HSI Off [%08lx %08lx]\n", RCC->CTLR, RCC->CFGR0 );

//	funPinMode( PB12, GPIO_CFGLR_OUT_50Mhz_AF_PP ); // NSS
//	funPinMode( PB13, GPIO_CFGLR_OUT_50Mhz_AF_PP ); // SCK
//	funPinMode( PB14, GPIO_CFGLR_OUT_50Mhz_AF_PP ); // MISO
	funPinMode( PB15, GPIO_CFGLR_OUT_50Mhz_AF_PP ); // MOSI

	RCC->APB1PRSTR = RCC_SPI2RST;
	RCC->APB1PRSTR = 0;
	RCC->APB1PCENR |= RCC_APB1Periph_SPI2;
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	// Configure SPI 
	SPI2->CTLR1 = 
		SPI_NSS_Soft | SPI_CPHA_1Edge | SPI_CPOL_Low | SPI_DataSize_16b |
		SPI_Mode_Master | SPI_Direction_1Line_Tx |
		0 |
		0<<3; // Divisior = 0

	// If using DMA may need this.
	SPI2->CTLR2 = SPI_CTLR2_TXDMAEN;


	SPI2->HSCR = 1; // High-speed enable.

	SPI2->CTLR1 |= CTLR1_SPE_Set;
	//SPI2->DATAR = 0x55aa; // Set SPI line Low.

	//DMA1_Channel5 is for SPI2TX
	DMA1_Channel5->PADDR = (uint32_t)&SPI2->DATAR;
	DMA1_Channel5->MADDR = (uint32_t)sendbuff;
	DMA1_Channel5->CNTR  = 0;// sizeof( bufferset )/2; // Number of unique copies.  (Don't start, yet!)
	DMA1_Channel5->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_VeryHigh |
		DMA_MemoryDataSize_HalfWord |
		DMA_PeripheralDataSize_HalfWord |
		DMA_MemoryInc_Enable |
		DMA_Mode_Normal | // OR DMA_Mode_Circular or DMA_Mode_Normal
		DMA_DIR_PeripheralDST |
		DMA_IT_TC | DMA_IT_HT; // Transmission Complete + Half Empty Interrupts. 

	NVIC_EnableIRQ( DMA1_Channel5_IRQn );
	DMA1_Channel5->CFGR |= DMA_CFGR1_EN;


	memset( sendbuff, 0x00, sizeof( sendbuff ) );

	// Enter critical section.
	DMA1_Channel5->CNTR  = 0;
	DMA1_Channel5->MADDR = (uint32_t)sendbuff;
	DMA1_Channel5->CNTR = SENDBUFF_WORDS; // Number of unique uint16_t entries.
	DMA1_Channel5->CFGR |= DMA_Mode_Circular;

	uint16_t lora_symbols[MAX_SYMBOLS];
	int lora_symbols_count;

	while(1)
	{

		Delay_Ms( 1000 );


#ifdef LORAWAN
		Delay_Ms( 1000 );
		static uint32_t frame = 0;

		// Send a message with LoraWan.  Formatted specifically for thethings.network.
		uint8_t inner_payload_raw[24];
		int inner_payload_len = snprintf( (char*)inner_payload_raw, 24, "meow%lu      ", frame%10 );
		inner_payload_len = 5;

		// Just some random data.
		uint8_t raw_payload_with_b0[259+8] = { };
		uint8_t * payload_in = raw_payload_with_b0 + 16;
		uint8_t * pl = payload_in;
		int payload_in_size = 0;

		pl += GenerateLoRaWANPacket( raw_payload_with_b0, inner_payload_raw, inner_payload_len, payload_key, network_skey, devaddress, frame++);

		payload_in_size = pl - payload_in;

		lora_symbols_count = 0;
#else

		// Just some random data.
		uint8_t payload_in[MAX_BYTES] = { 0xbb, 0xcc, 0xde, 0x55, 0x22,}; 
		int payload_in_size = 6;

		static int msgno = 0;
		payload_in[4] = msgno++;


#endif

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
		int16_t * qso = quadsets;
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
			printf( "%03x ", ofs );
		}
		printf( "\n" );

		runningcount_bits = 0;

		// This tells the interrupt we have data.
		quadsetcount = qso - quadsets + 0;
		printf( "--- %d [%d] %d\n", (int)lora_symbols_count, (int)quadsetcount, CHIPSSPREAD/4 );

		quadsetplace = 0;

		DMA1_Channel5->CFGR |= DMA_Mode_Circular;
	}
}

