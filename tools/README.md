# LoRasoft tools

testloradec.grc uses https://github.com/tapparelj/gr-lora_sdr

and is currently configured for an airspy.

But you can configure it for whatever.

If you do a run, it also records a file to /tmp/samplelog.dat

Which if you only record for about 1 second, you can convert into an image with `magsink_to_image.c` which outputs `image.png` so you can examine the stream.

If you want to compromise the signal, making it wider to get better granularity at the cost of temporal resolution, you can change FFTSIZE (in grc) to a different power of two.  Then you can update the `int width = 512;` in `magsink_to_image.c` to convert it to the right width for examination.

