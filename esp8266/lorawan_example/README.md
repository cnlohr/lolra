# ESP8266 / ESP8285 TheThings Network LoRaWAN transmitter.

Same as the others - connect a small length of wire to the "RX" pin on your ESP8266, then it will transmit SF7 LoRa messages at 904.1MHz.

But this, has the code in main() to send frames to thethings.network as valid LoRaWAN frames.  


On thethings.network,

1. Generate a new app.
2. Register a new device.
3. Select `Enter end device specifics manually`
4. Select US Plan 902-928, FSB2 (Used by TTN)
5. LoRaWAN Specification 1.0.0
6. TS001 Technical Specificaiton 1.0.0
7. "Show Advanced activation, LoRaWAN class and cluster settings"
8. Activation by Personalization
9. No addtional LoRaWAN class capabilities (class A only)
10. Use Network's default MAC settings.
11. Generate new EUI.
12. Generate new Device ID.

Add the device, it may make sense to back out to the menu so you can copy the LSB for the address, and the code for MSB for keys.

Replace the `payload_key`, `network_skey`, and `devaddress` in `main.c`


