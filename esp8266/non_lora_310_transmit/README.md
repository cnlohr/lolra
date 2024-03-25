# ESP8266 / ESP8285 310 MHz Transmitter

Transmits a 310 (or configurable) OOK pulse coded signal out the RX pin on the ESP8266/ESP8286.

You can also test it with an SDR and define

```c
#define CONTINUOUS_TONE
```

Frequency / word length can be configured in `rf_data_gen.c`
