// This file is under the standard MIT license, CC0 or Public domain.  you choose.
// CNLohr <>< 2024
// It just does a normal PWM output from a ch32v203

#include "ch32v003fun.h"
#include <stdio.h>

#define LEDPIN PD6

int main()
{	
	SystemInit();

	Delay_Ms( 100 );


	funGpioInitAll();
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	// PD4 = T2C1 on 003, PA0 on the 203.
	funPinMode( PA0, GPIO_CFGLR_OUT_50Mhz_AF_PP );
	funPinMode( PA1, GPIO_CFGLR_OUT_50Mhz_AF_PP );
	funPinMode( LEDPIN, GPIO_CFGLR_OUT_50Mhz_PP );

	// Reset TIM2 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	TIM2->PSC = 0x0000; // Prescalar
	TIM2->ATRLR = 4; // loop = fclk / (atrlr+1)
	TIM2->CHCTLR1 = TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE | TIM_OC1FE;
	TIM2->CTLR1 = TIM_ARPE;
	TIM2->CCER = TIM_CC1E | TIM_CC1P | TIM_CC1NE;
	TIM2->SWEVGR = TIM_UG;

	// Enable TIM2
	TIM2->CTLR1 |= TIM_CEN;
	TIM2->CH1CVR = TIM2->ATRLR/2+1;

	printf( "Setup\n" );
	while(1)
	{
		TIM2->CH1CVR = 2;
		TIM2->CCER = TIM_CC1E | TIM_CC1P;
		funDigitalWrite( LEDPIN, 1 );
		Delay_Us( 20000 );
		TIM2->CCER = TIM_CC1E;
		TIM2->CH1CVR = 2;
		funDigitalWrite( LEDPIN, 0 );
		Delay_Us( 20000 );
	}
}
