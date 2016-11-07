# 433MHz receiver

This project tried to decode the signals sent by some 433MHz remote control for power switches. The remote control has the "CO/TECH" brand name and PCB markings:

```
50027.01B
FLF-130105
```

The receiver has 2 parts:
* Arduino + cheap 433MHz receiver(XY-MK-5V markings)
* PC decoding software

## Arduino

Arduino does the minimum RF processing. Since the cheap 433MHz receiver signal levels aren't enough for the digital pins, the data output is connected to an analog input pin. After some tuning for signal level(cutoff between 1 and 0) the Arduino sketch sends over serial the time duration and signal level.

## PC Software

Cheap Clas Ohlson Chineese devices seem to run a similar protocol for communication:
Consider a time unit to be around 370ms(TU).
* the 0 bit is encoded by high signal that lasts arround 1 TU and low signal of 3 TU 
* the 1 bit si encoded by a high signal that lasts around 3 TU and low signal of 1 TU

The 24bit value is repeated a few times.

Checkout rcsend.cpp for a transmitter example that emulates the remote control.
