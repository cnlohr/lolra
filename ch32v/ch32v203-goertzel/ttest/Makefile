all : test floattest phasor_rotation_test

phasor_rotation_testt : phasor_rotation_test.c
	gcc -o $@ $^ -lm -g

phasor_rotation_test : phasor_rotation_testt
	./phasor_rotation_testt

floattest : floattestt
	./floattestt

floattestt : floattestt.c
	gcc -o $@ $^ -lm -g


test : ttest
	./ttest

ttest : ttest.c
	gcc -o $@ $^ -lm -g

clean :
	rm -rf *.o ttest *~ floattestt
