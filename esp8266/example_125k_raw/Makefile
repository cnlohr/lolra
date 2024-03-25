all : chirpbuff.dat.flash 

SUBMODULE=YES
SRCPREFIX=../nosdk8266/
MAIN_MHZ=52
SRCS=main.c
DEPS=chirpbuffinfo.h
COMPILECOUNT=$(shell cat compilecount.txt || true)
BUILDCOUNTCMD=echo 1+0$(COMPILECOUNT) | bc > compilecount.txt | true Compiled $(COMPILECOUNT) times
CFLAGS=-I../../lib -I..

include ../nosdk8266/Makefile

rf_data_gen : rf_data_gen.c
	gcc -o $@ $^ -lm -DMAIN_MHZ=$(MAIN_MHZ)

chirpbuffinfo.h : chirpbuff.dat

teststrap : main.c chirpbuff.dat
	gcc -o teststrap main.c -DTESTSTRAP -I../nosdk8266/include


chirpbuff.dat : rf_data_gen
	./rf_data_gen

chirpbuff.dat.flash : chirpbuff.dat
	$(ESPUTIL) -b 115200 flash 0x20000 chirpbuff.dat

cleanall : clean
	rm -rf chirpbuff.h rf_data_gen chirpbuff.dat chirpbuffinfo.h

