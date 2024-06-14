all : flash

TARGET:=nonlora
TARGET_MCU:=CH32V003
CH32V003FUN:=../ch32v003fun/ch32v003fun


EXTRA_CFLAGS:=-Wno-unused-function -I../../lib

include ../ch32v003fun/ch32v003fun/ch32v003fun.mk

flash : cv_flash
clean : cv_clean
	rm -rf rf_data_gen chirpbuff.dat chirpbuff.h chirpbuffinfo.h

