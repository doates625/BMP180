# BMP180
Class for interfacing with BMP180 I2C pressure sensor  
Written by Dan Oates (WPI Class of 2020)

### Description
The BMP180 measures temperature and pressure, and can use this data to estimate altitude. This class acts as an I2C interface with the device for Arduino and Mbed platforms.
  
Note: This class fails to accurately measure pressure (and thus altitude) on the Arduino platform for 8x oversampling.

### Dependencies
- [Platform](https://github.com/doates625/Platform.git)
- [I2CDevice](https://github.com/doates625/I2CDevice.git)
- [Struct](https://github.com/doates625/Struct.git)

### References
- [Datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf)