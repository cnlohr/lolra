all : flash 

SUBMODULE=YES
SRCPREFIX=../nosdk8266/
MAIN_MHZ=173
SRCS=main.c
DEPS=samples.h
COMPILECOUNT=$(shell cat compilecount.txt || true)
BUILDCOUNTCMD=echo 1+0$(COMPILECOUNT) | bc > compilecount.txt | true Compiled $(COMPILECOUNT) times
CFLAGS=-I../../lib

include ../nosdk8266/Makefile

rf_data_gen : rf_data_gen.c
	gcc -g -Og -o $@ $^ -lm -DMAIN_MHZ=$(MAIN_MHZ)

samples.h : rf_data_gen
	./rf_data_gen

#chirpbuff.dat.flash : chirpbuff.dat
#	$(ESPUTIL) -b 115200 flash 0x20000 chirpbuff.dat

cleanall : clean
	rm -rf rf_data_gen samples.h

