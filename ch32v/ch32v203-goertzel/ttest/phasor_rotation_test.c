#include <stdio.h>
#include <stdint.h>
#include <math.h>

int main()
{
	int phasor_r = 32768;
	int phasor_i = 0;
	double phasor = 0;

	double omega = 0.1;
	int omega_r = cos( omega ) * 32768;
	int omega_i = sin( omega ) * 32768;

	int i;
	for( i = 0; i < 10000; i++ )
	{
		
		int32_t temp = (phasor_r * omega_i + phasor_i * omega_r ) >> 15;
		phasor_r = (phasor_r * omega_r - phasor_i * omega_i ) >> 15;
		phasor_i = temp;
		phasor += omega;


		// Approximate sqrt(x*x+y*y)
		#define ABS(x) (((x)<0)?-(x):(x))
		int s = phasor_r * phasor_r + phasor_i * phasor_i;
		int intensity = (ABS(phasor_r) + ABS(phasor_i)) * 26100 / 32768 + 1; // Found experimentally (Also try to avoid divide-by-zero.
		intensity = (intensity + s/intensity)/2;
		intensity = (intensity + s/intensity)/2;

		if( intensity < 32763 )
		{
			phasor_r += phasor_r >> 12;
			phasor_i += phasor_i >> 12;
		}

		double fA = atan2( phasor_i, phasor_r );
		printf( "%6d %6d / %6d %6d / %d / %f %f %f\n", omega_r, omega_i, phasor_r, phasor_i, intensity, fA, phasor, fA-phasor );
		if( phasor >= 3.141592653589 ) phasor -= 3.141592653589*2;
	}

}
