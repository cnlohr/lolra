#include <stdio.h>
#include <math.h>
#include <stdint.h>

uint32_t g_goertzel_omega_per_sample;
int32_t g_goertzel_coefficient;
int32_t g_goertzel_coefficient_s;
uint32_t g_goertzel_samples;
uint32_t g_goertzel_outs;
int32_t g_goertzelp, g_goertzelp2;
int32_t g_goertzelp_store, g_goertzelp2_store;

int main()
{
	//XXX XXX NOTE If you are computing the coefficients, you can plug the value in here.
	g_goertzel_omega_per_sample = (47.0/256) * 3.1415926535*2.0*(1<<29);
	g_goertzel_coefficient   = 2 * cos( g_goertzel_omega_per_sample / (double)(1<<29) ) * (1<<30);
	g_goertzel_coefficient_s = 2 * sin( g_goertzel_omega_per_sample / (double)(1<<29) ) * (1<<30);


	const double AomegaPerSample = g_goertzel_omega_per_sample/(double)(1<<29);
	const int AnumSamples = 256; // enough to go from 0 to 2pi
	double Acoeff   = 2 * cos( AomegaPerSample ) * 65536;
	double Acoeff_s = 2 * sin( AomegaPerSample ) * 65536;
	double Asprev = AomegaPerSample * 65536;
	double Asprev2 = 0;

	printf( "%d / %d / %d    %f / %f / %f\n", g_goertzel_omega_per_sample, g_goertzel_coefficient, g_goertzel_coefficient_s, Acoeff, Acoeff_s, AomegaPerSample );


	g_goertzelp = g_goertzel_omega_per_sample>>(29-16);

	int32_t goertzel_coefficient = g_goertzel_coefficient;
	int32_t goertzelp2 = g_goertzelp2;
	int32_t goertzelp = g_goertzelp;
	uint32_t goertzel_samples = g_goertzel_samples;

	uint32_t adc_tail;

	double As;

	int js = 0;
	for( js = 0; js < 260/4; js++ )
	{
			int32_t t; // 1/2 of 4096, to try to keep our numbers reasonable.

			// Here is where the magic happens.
			int32_t goertzel;

			#define ITERATION(x)  \
				t = sin(  AomegaPerSample * (x+js*4) ) * 16384*0; \
\
				/* Fixed */ \
				goertzel = t + ( ( (((int32_t)(goertzel_coefficient))) * ((((int64_t)goertzelp)<<2)) ) >> 32 ) - goertzelp2; \
				goertzelp2 = goertzelp; \
				goertzelp = goertzel; \
\
				/* Float */ \
				As = t + ( Acoeff * Asprev ) / 65536.0 - Asprev2; \
				Asprev2 = Asprev; \
				Asprev = As; \
\
/*printf( "%d,%d,%d,%d\n", ( ( (int64_t)(goertzel_coefficient) * (int64_t)goertzelp ) >> 32 ) , goertzel_coefficient, goertzelp, goertzelp2 ); */ \
\
		{\
		int32_t rr = (((int64_t)(g_goertzel_coefficient  ) * (int64_t)goertzelp<<1)>>32) - (goertzelp2); \
		int32_t ri = (((int64_t)(g_goertzel_coefficient_s) * (int64_t)goertzelp<<1)>>32); \
		/*printf( "%3d %10d %10d %10d %10d / %9d %9d %9d * ", js*4+x, rr, ri, goertzelp, goertzelp2, goertzel_coefficient, g_goertzel_omega_per_sample, t );*/ \
		/*printf( "%4d %10d %10d (%10d %10d) ", js*4+x, goertzelp, goertzelp2, goertzel_coefficient, g_goertzel_coefficient_s );*/ \
		printf( "%4d %10d %10d (%10d %10d) ", js*4+x, goertzelp, goertzelp2, rr, ri ); \
\
		float Apower = Asprev*Asprev + Asprev2*Asprev2 - (Acoeff * Asprev * Asprev2); \
		double ArR = 0.5 * Acoeff   * Asprev / 65536 - Asprev2; \
		double ArI = 0.5 * Acoeff_s * Asprev / 65536; \
		/*printf( "%14.3f, %14.3f (%14.3f %14.3f)\n", Asprev, Asprev2, Acoeff,Acoeff_s );*/ \
		printf( "%14.3f, %14.3f (%14.3f %14.3f %f)\n", Asprev, Asprev2, ArR, ArI, sqrt( ArR*ArR +ArI*ArI ) ); \
		}


			ITERATION( 0 );
			ITERATION( 1 );
			ITERATION( 2 );
			ITERATION( 3 );

			adc_tail+=4;
			goertzel_samples+=4;
//			if( adc_tail == adc_buffer_top ) adc_tail = adc_buffer;

/*			if( goertzel_samples == 128 )
			{
				g_goertzelp_store = goertzelp;
				g_goertzelp2_store = goertzelp2;

				goertzelp = 0;
				goertzelp2 = 0;

				g_goertzel_outs++;
				goertzel_samples = 0;
			}
*/
	} 


	g_goertzelp2 = goertzelp2;
	g_goertzelp = goertzelp;
	g_goertzel_samples = goertzel_samples;

	printf( "Constants:\nconst int32_t g_goertzel_omega_per_sample = %d;\nconst int32_t g_goertzel_coefficient = %d;\nconst int32_t g_goertzel_coefficient_s = %d;\n", g_goertzel_omega_per_sample, g_goertzel_coefficient, g_goertzel_coefficient_s );

}

