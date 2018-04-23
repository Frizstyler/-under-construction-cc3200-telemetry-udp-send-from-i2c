# cc3200-telemetry-udp-send-from-i2c
This project incorporates an msp430g2553 MCU which calculates the input signal for a servo,and relays the 2-byte result to a CC3200 Wireless MCU via i2c after request.  The cc3200 has an AP open where one can connect and receive the UDP packets with the telemetry.  This is the cc3200 code.

Main code is the 'main' file.
