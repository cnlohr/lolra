all : flash

TARGET:=adcrx
TARGET_MCU:=CH32V003
CH32V003FUN:=../ch32v003fun/ch32v003fun

ADDITIONAL_C_FILES+=../rv003usb/rv003usb/rv003usb.S ../rv003usb/rv003usb/rv003usb.c
EXTRA_CFLAGS:=-I../rv003usb/lib -I../rv003usb/rv003usb -mstrict-align -Wno-unused-function

include ../ch32v003fun/ch32v003fun/ch32v003fun.mk

flash : cv_flash
clean : cv_clean
	rm -rf rf_data_gen chirpbuff.dat chirpbuff.h chirpbuffinfo.h




