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

#define LEDPIN PD6

int main()
{	
	SystemInit();

	Delay_Ms( 100 );

	funGpioInitAll();
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	// PD4 = T2C1
	funPinMode( PD4, GPIO_CFGLR_OUT_50Mhz_AF_PP );
	funPinMode( LEDPIN, GPIO_CFGLR_OUT_50Mhz_PP );

	// Reset TIM2 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	TIM2->PSC = 0x0000; // Prescalar
	TIM2->ATRLR = 1; // Max
	TIM2->CHCTLR1 = TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE;
	TIM2->CTLR1 = TIM_ARPE;
	TIM2->CCER = TIM_CC1E | TIM_CC1P | TIM_CC1NP;
	TIM2->SWEVGR = TIM_UG;

	// Enable TIM2
	TIM2->CTLR1 |= TIM_CEN;
	TIM2->CH1CVR = 1;

	while(1)
	{
		TIM2->CCER = TIM_CC1E | TIM_CC1P | TIM_CC1NP;
		funDigitalWrite( LEDPIN, 1 );
		Delay_Ms( 250 );
		TIM2->CCER = TIM_CC1E | TIM_CC1P;
		funDigitalWrite( LEDPIN, 0 );
		Delay_Ms( 250 );
	}
}
