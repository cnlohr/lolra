all : complex_magsink_to_image image_color.png image.png

magsink_to_image : magsink_to_image.c
	gcc -o $@ $^ -O3

complex_magsink_to_image : complex_magsink_to_image.c
	gcc -o $@ $^ -O3 -lm

image_color.png : complex_magsink_to_image /tmp/samplelog_I.dat /tmp/samplelog_Q.dat
	./complex_magsink_to_image

image.png : magsink_to_image /tmp/samplelog.dat
	./magsink_to_image

clean : 
	rm -rf magsink_to_image complex_magsink_to_image

