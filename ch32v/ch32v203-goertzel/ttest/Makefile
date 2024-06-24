all : test floattest

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
