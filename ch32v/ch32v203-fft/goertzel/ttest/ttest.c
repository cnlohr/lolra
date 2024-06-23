#include <stdio.h>
#include <math.h>
#include <stdint.h>

uint32_t g_goertzel_omega_per_sample;
uint32_t g_goertzel_coefficient;
int32_t g_goertzel_coefficient_s;
uint32_t g_goertzel_samples;
uint32_t g_goertzel_outs;
int32_t g_goertzelp, g_goertzelp2;
int32_t g_goertzelp_store, g_goertzelp2_store;

int main()
{
	
	g_goertzel_omega_per_sample = 1.0/128 * 3.14159*2.0*65536;
	g_goertzel_coefficient = 2 * cos( g_goertzel_omega_per_sample / 65536.0 ) * 65536;
	g_goertzel_coefficient_s = 2 * sin( g_goertzel_omega_per_sample / 65536.0 ) * 65536;
	

	g_goertzelp = g_goertzel_omega_per_sample;

	int32_t goertzel_coefficient = g_goertzel_coefficient;
	int32_t goertzelp2 = g_goertzelp2;
	int32_t goertzelp = g_goertzelp;
	uint32_t goertzel_samples = g_goertzel_samples;

	uint32_t adc_tail;


	int js = 0;
	for( js = 0; js < 256/4; js++ )
	{
			int32_t t; // 1/2 of 4096, to try to keep our numbers reasonable.

			// Here is where the magic happens.
			int32_t goertzel;

			#define ITERATION(x)  \
				t = sin( (x+js*4) ) * 65536; \
				goertzel = t + ( ( (int64_t)(goertzel_coefficient) * goertzelp ) >> 16 ) - goertzelp2; \
				goertzelp2 = goertzelp; \
				goertzelp = goertzel; \
\
/*printf( "%d,%d,%d,%d\n", ( ( (int64_t)(goertzel_coefficient) * (int64_t)goertzelp ) >> 32 ) , goertzel_coefficient, goertzelp, goertzelp2 ); */ \
\
		{\
		float coeff_s = /* 2 * sin( g_goertzel_omega_per_sample/65536.0f ); */ 65536; \
		int32_t rr = ((g_goertzel_coefficient/2  * (int64_t)goertzelp)>>16) - g_goertzelp2; \
		int32_t ri = ((g_goertzel_coefficient_s/2 * (int64_t)goertzelp)>>16); \
		printf( "%3d %9d %9d %9d %9d / %9d %9d %9d\n", js, rr, ri, goertzelp, goertzelp2, goertzel_coefficient, g_goertzel_omega_per_sample, t ); \
		}


			ITERATION( 0 );
			ITERATION( 1 );
			ITERATION( 2 );
			ITERATION( 3 );

			adc_tail+=4;
			goertzel_samples+=4;
//			if( adc_tail == adc_buffer_top ) adc_tail = adc_buffer;
			if( goertzel_samples == 128 )
			{
				g_goertzelp_store = goertzelp;
				g_goertzelp2_store = goertzelp2;

				goertzelp = 0;
				goertzelp2 = 0;

				g_goertzel_outs++;
				goertzel_samples = 0;
			}
	} 

	g_goertzelp2 = goertzelp2;
	g_goertzelp = goertzelp;
	g_goertzel_samples = goertzel_samples;

}

