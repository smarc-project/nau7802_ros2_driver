# AdaFruit NAU7802 QT driver for ROS2 Humble


Follow this to install system dependencies: [Python & CircuitPython \| Adafruit NAU7802 24-Bit ADC - STEMMA QT / Qwiic \| Adafruit Learning System](https://learn.adafruit.com/adafruit-nau7802-24-bit-adc-stemma-qt-qwiic/python-circuitpython)

TLDR: `pip3 install cedargrove-nau7802 circup`.

Reference: https://github.com/adafruit/CircuitPython_NAU7802

On the Jetson Orin, connected to i2c-8 pins on the 40-pin header (pin 1,3,5,9 -> 3.3v, SDA, SCL, GND) the device shows up on channel 7.
Confirm with `sudo i2cdetect -y -r 7`. 
It should look like:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- 2a -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --    
```

2a existing is what you are looking for.


