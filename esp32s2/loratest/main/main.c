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

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h" // or "esp_efuse_custom_table.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "soc/dedic_gpio_reg.h"
#include "driver/gpio.h"
#include "soc/soc.h"
#include "soc/system_reg.h"
#include "soc/usb_reg.h"
#include "ulp_riscv.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "rom/gpio.h"
#include "soc/rtc.h"

#define SOC_DPORT_USB_BASE 0x60080000

struct SandboxStruct
{
	void (*fnIdle)();
};


struct SandboxStruct * g_SandboxStruct;


void esp_sleep_enable_timer_wakeup();

volatile void * keep_symbols[] = { 0, vTaskDelay, ulp_riscv_halt,
	ulp_riscv_timer_resume, ulp_riscv_timer_stop, ulp_riscv_load_binary,
	ulp_riscv_run, ulp_riscv_config_and_run, esp_sleep_enable_timer_wakeup,
	ulp_set_wakeup_period, rtc_gpio_init, rtc_gpio_set_direction,
	rtc_gpio_set_level, gpio_config, gpio_matrix_out, gpio_matrix_in,
	rtc_clk_cpu_freq_get_config, rtc_clk_cpu_freq_set_config_fast,
	rtc_clk_apb_freq_get };

extern struct SandboxStruct sandbox_mode;

void app_main(void)
{
	printf("Hello world! Keep table at %p\n", &keep_symbols );

	g_SandboxStruct = &sandbox_mode;

	// esp_efuse_set_rom_log_scheme(ESP_EFUSE_ROM_LOG_ALWAYS_OFF);
	esp_efuse_set_rom_log_scheme(ESP_EFUSE_ROM_LOG_ALWAYS_ON);
        
	printf("Minimum free heap size: %d bytes\n", (int)esp_get_minimum_free_heap_size());

	void sandbox_main();

	sandbox_main();

	do
	{
		if( g_SandboxStruct && g_SandboxStruct->fnIdle ) { g_SandboxStruct->fnIdle(); }
		esp_task_wdt_reset();
		taskYIELD();
	} while( 1 );

//	printf("Restarting now.\n");
//	fflush(stdout);
//	esp_restart();
}
