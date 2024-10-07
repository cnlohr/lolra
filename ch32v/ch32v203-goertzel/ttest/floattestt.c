#include <stdio.h>
#include <math.h>

int main()
{
	const float omegaPerSample = 3.1415926*2.0 / 128; // pi / 200

	// Only divide by 128 to show two cycles.

	const int numSamples = 256; // enough to go from 0 to 2pi

	float phase = 0;
	//for( float phase = 0; phase < 3.1415926*2.0; phase += 0.01 )
	{
		float coeff = 2 * cos( omegaPerSample );
		int i;

		// TRICKY: When you want a sinewave, initialize with omegaPerSample.  This
		// is crucial.  The initial state will have massive consequences.
		float sprev = omegaPerSample;
		float sprev2 = 0;
		for( i = 0; i < numSamples; i++ )
		{
			// If you wanted to do a DFT, set SAMPLE to your incoming sample.
			float SAMPLE = 65536 * sin( phase + i * omegaPerSample );

			// Here is where the magic happens.
			float s = SAMPLE + coeff * sprev - sprev2;
			sprev2 = sprev;
			sprev = s;

		// For DFT, your power will be:
		float power = sprev*sprev + sprev2*sprev2 - (coeff * sprev * sprev2);
		//printf( "Power: %f\n", power );

		float coeff_s = 2 * sin( omegaPerSample );
		double rR = 0.5 * coeff   * sprev - sprev2;
		double rI = 0.5 * coeff_s * sprev;
		printf( "%d,%f,%f\n", i, rR, rI);

		}

	}
}
