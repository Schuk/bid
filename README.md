# BID

This arduino sketch is intended for mantaining the temperature of a liquid.

The temperature is read from a DS18B20 and fed into a PID controller

The PID controlls a power relay, which switches a heater on and off.

The intended temperature of the liquid can be set by using a poti.
A button toggles between three setting (25W heater, 3000W heater, and manual).

The settings heavily depend on the volume you are heating. Our drum has a volume of 50l.
