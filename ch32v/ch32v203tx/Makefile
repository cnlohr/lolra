all : flash

TARGET:=ch32v203tx
TARGET_MCU:=CH32V203
CH32V003FUN:=../ch32v003fun/ch32v003fun

EXTRA_CFLAGS:=-Wno-unused-function -I../../lib

include $(CH32V003FUN)/ch32v003fun.mk

flash : cv_flash
clean : cv_clean

