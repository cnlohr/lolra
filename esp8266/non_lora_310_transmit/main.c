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

#include "esp8266_auxrom.h"
#include "eagle_soc.h"
#include "nosdk8266.h"
#include "esp8266_rom.h"
#include <string.h>
#include "ets_sys.h"
#include "pin_mux_register.h"
#include "slc_register.h"
#include "dmastuff.h"
#include "samples.h"

//#define CONTINUOUS_TONE

// The speed at which bits are shifted out - this is done with trial and error.  Try one speed
// and figure out how it aligns to your target system.  The units are too messy for me to be
// troubled with thinking of.

#define BIT_RATE 245
#define num_replays (( SAMPLE_RATE / (DMA_WORDS*NUMBITS) ) / BIT_RATE)

uint8_t output_buffer[] = {
	// OOK, map to time_limits_for_bits, 0 is short pulse, 1 is long pulse, 2 is no-signal.
	// You will need to manually use your 310MHz device and record the pulse timing
	// To fill out this array with thecorrect code for your opener.
	0, 1, 0, 0, 0, 1, 1, 2, 2, 2, 2, 1, 0, 1, 0, 1, 0, 0, 1, 1,
};
uint32_t time_limits_for_bits[3] = { num_replays/8, num_replays/2, 0 };
uint32_t dummy[DMA_WORDS];

void testi2s_init();

//These contol the speed at which the bus comms.
#define DMABUFFERDEPTH 3
#define WS_I2S_BCK SPI_DIV  //Can't be less than 1.
#define WS_I2S_DIV 1
#define call_delay_us(time) { asm volatile("mov.n a2, %0\n_call0 delay4clk" : : "r"(time * (MAIN_MHZ / 8)) : "a2" ); }

//I2S DMA buffer descriptors
static struct sdio_queue i2sBufDescTX[DMABUFFERDEPTH] __attribute__((aligned(128)));;

volatile int output_buffer_place = 1000000;
volatile int count_out_this_bit = 0;
volatile int etx = 0;

void slc_isr(void * v) {

	uint32_t * sendbuff = 0;

	struct sdio_queue *finishedDesc;
//	slc_intr_status = READ_PERI_REG(SLC_INT_STATUS); -> We should check to make sure we are SLC_RX_EOF_INT_ST, but we are only getting one interrupt.

	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
	finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);

#ifdef CONTINUOUS_TONE
	return;
#endif

	etx++;

	if( output_buffer_place >= sizeof( output_buffer ) / sizeof( output_buffer[0] )  )
	{
		goto dump0;
	}

	count_out_this_bit++;

	if( count_out_this_bit >= num_replays )
	{
		count_out_this_bit = 0;
		output_buffer_place++;

		if( output_buffer_place >= sizeof( output_buffer ) / sizeof( output_buffer[0] )  )
		{
			goto dump0;
		}
	}

	int symbol = output_buffer[output_buffer_place];

	if( count_out_this_bit < time_limits_for_bits[symbol] )
	{
		sendbuff = samples;
	}
	else
	{
		sendbuff = dummy;
	}

	finishedDesc->buf_ptr = (uint32_t)sendbuff;
	finishedDesc->datalen = DMA_WORDS*4;

	return;
dump0:
	finishedDesc->buf_ptr = (uint32_t)dummy;
	finishedDesc->datalen = DMA_WORDS*4;
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


void testi2s_init()
{
}

#else

//Initialize I2S subsystem for DMA circular buffer use
void testi2s_init() {
	int x, y;
	//Bits are shifted out

	//Initialize DMA buffer descriptors in such a way that they will form a circular
	//buffer.

	for (x=0; x<DMABUFFERDEPTH; x++) {
		i2sBufDescTX[x].owner=1;
		i2sBufDescTX[x].eof=1;  // Trigger interrupt on packet complete.
		i2sBufDescTX[x].sub_sof=0;
		i2sBufDescTX[x].datalen=DMA_WORDS*4;
		i2sBufDescTX[x].blocksize=4;
#ifdef CONTINUOUS_TONE
		i2sBufDescTX[x].buf_ptr= ((uint32_t)samples);
#else
		i2sBufDescTX[x].buf_ptr= ((uint32_t)dummy);
#endif
		i2sBufDescTX[x].unused=0;
		i2sBufDescTX[x].next_link_ptr=(int)((x<(DMABUFFERDEPTH-1))?(&i2sBufDescTX[x+1]):(&i2sBufDescTX[0]));
	}

	//Reset DMA )
	//SLC_TX_LOOP_TEST = IF this isn't set, SO will occasionally get unrecoverable errors when you underflow.
	//Originally this little tidbit was found at https://github.com/pvvx/esp8266web/blob/master/info/libs/bios/sip_slc.c
	//
	//I have not tried without SLC_AHBM_RST | SLC_AHBM_FIFO_RST.  I just assume they are useful?
	SET_PERI_REG_MASK(SLC_CONF0, SLC_TX_LOOP_TEST |SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST | SLC_AHBM_FIFO_RST);
	CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST | SLC_AHBM_FIFO_RST);

	//Clear DMA int flags
	SET_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);
	CLEAR_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);

	//Enable and configure DMA
	CLEAR_PERI_REG_MASK(SLC_CONF0, (SLC_MODE<<SLC_MODE_S));
	SET_PERI_REG_MASK(SLC_CONF0,(1<<SLC_MODE_S));
	
	// We have to do this, otherwise, when the end of a "RX" packet is hit, it will skip outputting a few random frames.
	SET_PERI_REG_MASK(SLC_RX_DSCR_CONF,SLC_INFOR_NO_REPLACE|SLC_TOKEN_NO_REPLACE);
	CLEAR_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_RX_FILL_EN|SLC_RX_EOF_MODE | SLC_RX_FILL_MODE);
	
	CLEAR_PERI_REG_MASK(SLC_RX_LINK,SLC_RXLINK_DESCADDR_MASK);
	SET_PERI_REG_MASK(SLC_RX_LINK, ((uint32)&i2sBufDescTX[0]) & SLC_RXLINK_DESCADDR_MASK);

	//Attach the DMA interrupt
	ets_isr_attach(ETS_SLC_INUM, slc_isr, 0);
	WRITE_PERI_REG(SLC_INT_ENA,  SLC_RX_EOF_INT_ENA );		// Not including SLC_RX_UDF_INT_ENA

	//clear any interrupt flags that are set
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
	///enable DMA intr in cpu
	ets_isr_unmask(1<<ETS_SLC_INUM);

	//Start transmission
	SET_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);


	//Init pins to i2s functions
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_I2SO_DATA); // GPIO3
//	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_I2SO_WS);   // GPIO2
//	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_I2SO_BCK);   // GPIO15

	//Enable clock to i2s subsystem
	i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 1);

	//Reset I2S subsystem
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);

	CLEAR_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN|(I2S_I2S_RX_FIFO_MOD<<I2S_I2S_RX_FIFO_MOD_S));
	SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN);
	WRITE_PERI_REG(I2SRXEOF_NUM, DMA_WORDS*4);

	CLEAR_PERI_REG_MASK(I2SCONF_CHAN, (I2S_RX_CHAN_MOD<<I2S_RX_CHAN_MOD_S));

	CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|I2S_RECE_SLAVE_MOD|
						(I2S_BITS_MOD<<I2S_BITS_MOD_S)|
						(I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
                                    	(I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
	
	SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						((WS_I2S_BCK&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						((WS_I2S_DIV&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S) );

	//Start transmission
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START|I2S_I2S_RX_START);
}

#endif

int main()
{
	// We store the bit pattern at flash:0x20000, so we don't have to constantly
	// re-write it when working on code.
	memset( dummy, 0, sizeof( dummy ) );

	int frame = 0;
	output_buffer_place = 1000000;
	count_out_this_bit = 0;
	etx = 0;

	// Don't crank up clock speed til we're done with flash.
	nosdk8266_init();

	testi2s_init();

#ifdef CONTINUOUS_TONE
	while(1);
#endif


	while(1) {
		//12x this speed.
		frame++;

		PIN_OUT_SET = _BV(2); //Turn GPIO2 light off.
		//call_delay_us(1000000);
		printf("ETX: %d %d %d %d ( %d / %d ) / %d \n", etx, output_buffer_place, count_out_this_bit, num_replays, SAMPLE_RATE, BIT_RATE, DMA_WORDS );
		PIN_OUT_CLEAR = _BV(2); //Turn GPIO2 light off.

		output_buffer_place = 0;
		count_out_this_bit = 0;
		call_delay_us(5000000);
	}

}

