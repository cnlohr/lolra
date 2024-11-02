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

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "hal/gpio_types.h"
#include "esp_log.h"
#include "soc/efuse_reg.h"
#include "soc/soc.h"
#include "soc/system_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_sig_map.h"


#include "soc/io_mux_reg.h"
#include "soc/dedic_gpio_reg.h"

// Funtenna will output on GPIO1 and 14.  Otherwise it will be 17 and 18
//#define FUNTENNA

int global_i = 100;


static inline uint32_t getCycleCount()
{
    uint32_t ccount;
    asm volatile("rsr %0,ccount":"=a" (ccount));
    return ccount;
}


#define IO_MUX_REG(x) XIO_MUX_REG(x)
#define XIO_MUX_REG(x) IO_MUX_GPIO##x##_REG

#define GPIO_NUM(x) XGPIO_NUM(x)
#define XGPIO_NUM(x) GPIO_NUM_##x


#ifdef FUNTENNA

#define RF1_PIN 1
#define RF2_PIN 14
#else
#define RF1_PIN 17
#define RF2_PIN 18
#endif



#include "hal/gpio_types.h"
#include "driver/gpio.h"
#include "rom/gpio.h"
#include "soc/i2s_reg.h"
#include "soc/periph_defs.h"
#include "rom/lldesc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "soc/regi2c_apll.h"
//#include "components/hal/esp32s2/include/hal/regi2c_ctrl_ll.h
#include "hal/regi2c_ctrl_ll.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/regi2c_ctrl.h"
#include "hal/clk_tree_ll.h"




#define I2C_RTC_WIFI_CLK_EN (SYSCON_WIFI_CLK_EN_REG)

#define I2C_RTC_CLK_GATE_EN    (BIT(18))
#define I2C_RTC_CLK_GATE_EN_M  (BIT(18))
#define I2C_RTC_CLK_GATE_EN_V  0x1
#define I2C_RTC_CLK_GATE_EN_S  18

#define I2C_RTC_CONFIG0  0x6000e048

#define I2C_RTC_MAGIC_CTRL 0x00001FFF
#define I2C_RTC_MAGIC_CTRL_M  ((I2C_RTC_MAGIC_CTRL_V)<<(I2C_RTC_MAGIC_CTRL_S))
#define I2C_RTC_MAGIC_CTRL_V  0x1FFF
#define I2C_RTC_MAGIC_CTRL_S  4

#define I2C_RTC_CONFIG1  0x6000e044

#define I2C_RTC_BOD_MASK (BIT(22))
#define I2C_RTC_BOD_MASK_M  (BIT(22))
#define I2C_RTC_BOD_MASK_V  0x1
#define I2C_RTC_BOD_MASK_S  22

#define I2C_RTC_SAR_MASK (BIT(18))
#define I2C_RTC_SAR_MASK_M  (BIT(18))
#define I2C_RTC_SAR_MASK_V  0x1
#define I2C_RTC_SAR_MASK_S  18

#define I2C_RTC_BBPLL_MASK (BIT(17))
#define I2C_RTC_BBPLL_MASK_M  (BIT(17))
#define I2C_RTC_BBPLL_MASK_V  0x1
#define I2C_RTC_BBPLL_MASK_S  17

#define I2C_RTC_APLL_MASK (BIT(14))
#define I2C_RTC_APLL_MASK_M  (BIT(14))
#define I2C_RTC_APLL_MASK_V  0x1
#define I2C_RTC_APLL_MASK_S  14

#define I2C_RTC_ALL_MASK 0x00007FFF
#define I2C_RTC_ALL_MASK_M  ((I2C_RTC_ALL_MASK_V)<<(I2C_RTC_ALL_MASK_S))
#define I2C_RTC_ALL_MASK_V  0x7FFF
#define I2C_RTC_ALL_MASK_S  8

#define I2C_RTC_CONFIG2  0x6000e000

#define I2C_RTC_BUSY (BIT(25))
#define I2C_RTC_BUSY_M  (BIT(25))
#define I2C_RTC_BUSY_V  0x1
#define I2C_RTC_BUSY_S  25

#define I2C_RTC_WR_CNTL (BIT(24))
#define I2C_RTC_WR_CNTL_M  (BIT(24))
#define I2C_RTC_WR_CNTL_V  0x1
#define I2C_RTC_WR_CNTL_S  24

#define I2C_RTC_DATA 0x000000FF
#define I2C_RTC_DATA_M  ((I2C_RTC_DATA_V)<<(I2C_RTC_DATA_S))
#define I2C_RTC_DATA_V  0xFF
#define I2C_RTC_DATA_S  16

#define I2C_RTC_ADDR 0x000000FF
#define I2C_RTC_ADDR_M  ((I2C_RTC_ADDR_V)<<(I2C_RTC_ADDR_S))
#define I2C_RTC_ADDR_V  0xFF
#define I2C_RTC_ADDR_S  8

#define I2C_RTC_SLAVE_ID 0x000000FF
#define I2C_RTC_SLAVE_ID_M  ((I2C_RTC_SLAVE_ID_V)<<(I2C_RTC_SLAVE_ID_S))
#define I2C_RTC_SLAVE_ID_V  0xFF
#define I2C_RTC_SLAVE_ID_S  0

#define I2C_RTC_MAGIC_DEFAULT (0x1c40)

#define I2C_BOD     0x61
#define I2C_BBPLL   0x66
#define I2C_SAR_ADC 0X69
#define I2C_APLL    0X6D


int lastend = 0;

// This generates a slow sweep over 17.8 seconds of exactly one full SDM0 iteration
// this is to measure how good dithering can be.

const float fRadiator = 28.08; // In MHz 28.08MHz = in RTTY section of 10 Meters HAM band.
const float fHarmonic = 1.0;
const float fXTAL = 40;

// Set to 2 for highest possible frequencies (up to 69MHz) Lower to get more control at lower frequencies.  This is the part in parenthesis (ODIV+2)
// Higher numbers will give you more control. But, there is a limit. 350<40 * (SDM2 + SDM1/(2^8) + SDM0/(2^16) + 4)<500
// Set to 3 for up to 46MHz
// Set to 4 for up to 34MHz
// set to 5 for up to 27MHz (well 27MHzish, you can go a tad higher)
const float nPLLDivisorD2 = 5;




// Configures APLL = 480 / 4 = 120
// 40 * (SDM2 + SDM1/(2^8) + SDM0/(2^16) + 4) / ( 2 * (ODIV+2) );
// Datasheet recommends that numerator does not exceed 500MHz.
void IRAM_ATTR local_rtc_clk_apll_enable(bool enable, uint32_t sdm0, uint32_t sdm1, uint32_t sdm2, uint32_t o_div)
{
	REG_SET_FIELD(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PD, enable ? 0 : 1);
	REG_SET_FIELD(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PU, enable ? 1 : 0);

	if (enable) {
		REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM2, sdm2);
		REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM0, sdm0);
		REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM1, sdm1);
		REGI2C_WRITE(I2C_APLL, I2C_APLL_SDM_STOP, CLK_LL_APLL_SDM_STOP_VAL_1);
		REGI2C_WRITE(I2C_APLL, I2C_APLL_SDM_STOP, CLK_LL_APLL_SDM_STOP_VAL_2_REV1);
		REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_OR_OUTPUT_DIV, o_div );
	}


	// Settings determined experimentally.
	REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_OC_TSCHGP, 0 ); // 0 or 1
	REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_EN_FAST_CAL, 1 ); // 0 or 1
	REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_OC_DHREF_SEL, 3 ); // 0..3
	REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_OC_DLREF_SEL, 2 ); // 0..3
	REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_SDM_DITHER, 1 ); // 0 or 1
	REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_OC_DVDD, 25 ); // 0 .. 31

}


void IRAM_ATTR regi2c_write_reg_raw_local(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t data)
{
    uint32_t temp = ((block & I2C_RTC_SLAVE_ID_V) << I2C_RTC_SLAVE_ID_S)
                    | ((reg_add & I2C_RTC_ADDR_V) << I2C_RTC_ADDR_S)
                    | ((0x1 & I2C_RTC_WR_CNTL_V) << I2C_RTC_WR_CNTL_S)
                    | (((uint32_t)data & I2C_RTC_DATA_V) << I2C_RTC_DATA_S);
    while (REG_GET_BIT(I2C_RTC_CONFIG2, I2C_RTC_BUSY));
    REG_WRITE(I2C_RTC_CONFIG2, temp);
}


void apll_quick_update( uint32_t sdm )
{
	uint8_t sdm2 = sdm>>16;
	uint8_t sdm1 = (sdm>>8)&0xff;
	uint8_t sdm0 = (sdm>>0)&0xff;
	static int last_sdm_0 = -1;
	static int last_sdm_1 = -1;
	static int last_sdm_2 = -1;

	if( sdm2 != last_sdm_2 )
		regi2c_write_reg_raw_local(I2C_APLL, I2C_APLL_HOSTID, I2C_APLL_DSDM2, sdm2);
	if( sdm0 != last_sdm_0 ) 
		regi2c_write_reg_raw_local(I2C_APLL, I2C_APLL_HOSTID, I2C_APLL_DSDM0, sdm0);
	if( sdm1 != last_sdm_1 )
		regi2c_write_reg_raw_local(I2C_APLL, I2C_APLL_HOSTID, I2C_APLL_DSDM1, sdm1);

	last_sdm_2 = sdm2;
	last_sdm_1 = sdm1;
	last_sdm_0 = sdm0;
}



void sandbox_main()
{
	printf( "sandbox_main()\n" );



	DPORT_SET_PERI_REG_MASK( DPORT_CPU_PERI_CLK_EN_REG, DPORT_CLK_EN_DEDICATED_GPIO );
	DPORT_CLEAR_PERI_REG_MASK( DPORT_CPU_PERI_RST_EN_REG, DPORT_RST_EN_DEDICATED_GPIO);

	// Setup GPIO39 to be the PRO ALONE output (For LEDs)
	REG_WRITE( GPIO_OUT1_W1TC_REG, 1<<(39-32) );
	REG_WRITE( GPIO_ENABLE1_W1TS_REG, 1<<(39-32) );
	REG_WRITE( IO_MUX_GPIO39_REG, 2<<FUN_DRV_S );
	REG_WRITE( GPIO_FUNC39_OUT_SEL_CFG_REG, PRO_ALONEGPIO_OUT0_IDX );
	REG_WRITE( DEDIC_GPIO_OUT_CPU_REG, 0x01 ); // Enable CPU instruction output


	// Output clock on P2.

	// Maximize the drive strength.
	gpio_set_drive_capability( GPIO_NUM(RF1_PIN), GPIO_DRIVE_CAP_3 );
	gpio_set_drive_capability( GPIO_NUM(RF2_PIN), GPIO_DRIVE_CAP_3 );

	// Use the IO matrix to create the inverse of TX on pin 17.
	gpio_matrix_out( GPIO_NUM(RF1_PIN), CLK_I2S_MUX_IDX, 1, 0 );
	gpio_matrix_out( GPIO_NUM(RF2_PIN), CLK_I2S_MUX_IDX, 0, 0 );

	periph_module_enable(PERIPH_I2S0_MODULE);

	int use_apll = 1;

	// Start with a default tone.
	int sdm0 = 100;
	int sdm1 = 230;
	int sdm2 = 8;
	int odiv = nPLLDivisorD2-2;

	local_rtc_clk_apll_enable( use_apll, sdm0, sdm1, sdm2, odiv );

	if( use_apll )
	{
		WRITE_PERI_REG( I2S_CLKM_CONF_REG(0), (1<<I2S_CLK_SEL_S) | (1<<I2S_CLK_EN_S) | (0<<I2S_CLKM_DIV_A_S) | (0<<I2S_CLKM_DIV_B_S) | (1<<I2S_CLKM_DIV_NUM_S) );
	}
	else
	{
		// fI2S = fCLK / ( N + B/A )
		// DIV_NUM = N
		// Note I2S_CLKM_DIV_NUM minimum = 2 by datasheet.  Less than that and it will ignoreeee you.
		WRITE_PERI_REG( I2S_CLKM_CONF_REG(0), (2<<I2S_CLK_SEL_S) | (1<<I2S_CLK_EN_S) | (0<<I2S_CLKM_DIV_A_S) | (0<<I2S_CLKM_DIV_B_S) | (1<<I2S_CLKM_DIV_NUM_S) );  // Minimum reduction, 2:1
	}

	lastend = getCycleCount();
}



#define DisableISR()            do { XTOS_SET_INTLEVEL(XCHAL_EXCM_LEVEL); portbenchmarkINTERRUPT_DISABLE(); } while (0)
#define EnableISR()             do { portbenchmarkINTERRUPT_RESTORE(0); XTOS_SET_INTLEVEL(0); } while (0)


uint32_t frame = 0;
void sandbox_tick()
{


	const float fTarg = (fRadiator)/fHarmonic;
	const float fAPLL = fTarg * 2;
	const uint32_t sdmBaseTarget = ( fAPLL * (nPLLDivisorD2*2) / fXTAL - 4 ) * 65536 + 0.5; // ~649134


	{
		int iterct = 0;
		int dither = 0;

		gpio_matrix_out( GPIO_NUM(RF1_PIN), CLK_I2S_MUX_IDX, 1, 0 );
		gpio_matrix_out( GPIO_NUM(RF2_PIN), CLK_I2S_MUX_IDX, 0, 0 );

		//DisableISR();

		while(1)
		{
			int fplv = 0;
			frame = (getCycleCount());
			fplv = 0;
			int microtone = ((frame & 0xfff00000 ) >> 20);
			dither = (((iterct)&0xffff)<microtone)?1:0;
			uint32_t sdm = sdmBaseTarget + fplv + dither;//( fplv ) / sdmDivisor + dither;
			apll_quick_update( sdm );
			iterct++;
		}

		gpio_matrix_out( GPIO_NUM(RF1_PIN), CLK_I2S_MUX_IDX, 1, 1 );
		gpio_matrix_out( GPIO_NUM(RF2_PIN), CLK_I2S_MUX_IDX, 0, 1 );

		printf( "Iter: %d\n", iterct );
		lastend = getCycleCount();
	}

//	vTaskDelay( 1 );
}

struct SandboxStruct
{
	void (*fnIdle)();
};
struct SandboxStruct sandbox_mode =
{
	.fnIdle = sandbox_tick,
};
