# ze08-ch2o-Arduino
A user-space driver for the Winsen ZE08-CH2O formaldehyde sensor

It has been tested on a desktop PC with a USB-serial converter and on a Raspberry Pi.

Remember that this sensor does NOT tolerate 5V in its RX input. 
If you just want to use the default, active mode, you don't even need to
connect this pin, so you can connect directly 5V, GND and Arduino's RX.

# Building

mkdir build && cd build &&
cmake .. && make

# Usage

```
build/ze08-ch2o <port> [flush interval]
```
For example:
`build/ze08-ch2o /dev/ttyUSB0 60 > ch2o.log` will only write the outputs
to stdout every 60 seconds, thus reducing flash memory wear-out.

The output is preceded by a timestamp number (the customary seconds since epoch).

# Feedback welcome

If it helps you in your project, or if you have bug reports or feature requests, 
don't hesitate to contact me.

# Links

An equivalent driver is available for Arduino/ESP8266 here: https://github.com/rsalinas/ze08-ch2o-arduino
