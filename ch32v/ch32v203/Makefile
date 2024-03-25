all : flash

TARGET:=loratest
TARGET_MCU:=CH32V203
CH32V003FUN:=../ch32v003fun/ch32v003fun

EXTRA_ELF_DEPENDENCIES:=chirpbuff.h

EXTRA_CFLAGS:=-Wno-unused-function -I../../lib

include ../ch32v003fun/ch32v003fun/ch32v003fun.mk


chirpbuff.h : rf_data_gen
	./rf_data_gen

rf_data_gen : rf_data_gen.c
	gcc -o $@ $^ -lm

flash : cv_flash
clean : cv_clean
	rm -rf rf_data_gen chirpbuff.dat chirpbuff.h chirpbuffinfo.h

