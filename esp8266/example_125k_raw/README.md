# ESP8266 / ESP8285 Lora Transmitter

Connect a small length of wire to the "RX" pin on your ESP8266, then it will transmit SF7 LoRa messages at 904.1MHz.

This is designed to work at 173MHz, but, you can set `MAIN_MHZ=80` in the Makefile, and run it without any overclocking.

