# BMS_MKII_ESP32_RT

## Arduino IDE setup on Ubuntu

1. Download Arduino IDE from the link:
   ```
   https://www.arduino.cc/en/software
   ```

1. In order to use `ros_lib`, follow the following guide (do not forget to have ROS installed):
   ```
   https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
   ``` 

1. Add ESP32 to board manager: File -> Preferences -> Paste the json file link in the list of additional boards
   ```
   https://dl.espressif.com/dl/package_esp32_index.json
   ```

1. Used temperature sensor library: DHT sensor library by Adafruit. Dependencies: Adafruit_Sensor.h

1. Used current sensor library: ACS712 20A by Rob Tillaart. Dependencies: none.

1. ESP32 library has a built-in version of FreeRTOS. No need to install that.

## Code Description

maybe when I am done :)
