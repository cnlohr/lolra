# How to use this

 * Display on PB8/PB9 for SCL/SDA
 * Analog is on PA7

First, use FFT, then look in the middle for the highest bin @.  This is the # of pixels from the left side of the screen you are.

Then edit ttest.c, 

```
	g_goertzel_omega_per_sample = (47.0/256) * 3.1415926535*2.0*65536;
```

where 47.0 in this case is the bin # from the FFT.

Change this, `make test`

Then copy the values:
const int32_t g_goertzel_omega_per_sample = 75599;
const int32_t g_goertzel_coefficient = 870257651;
const int32_t g_goertzel_coefficient_s = 1963246708;

into the top of adcgoertzel.c


TODO: TODO: Test HTML values.

TODO: TODO: g_goertzel_omega_per_sample lokos WRONG.


Extra notes:
 .87996033 = 880 AM, why?
