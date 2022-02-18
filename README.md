# PowerGuru Node

ESP8266 based application for reading energy meters, sensors and controlling switches based on price, energy forecast and grid (consumption and production) information. See main project project [PowerGuru Server](https://github.com/Olli69/powerguru/)

# Interfaces
## Shelly 3EM energy meter
Reads consumed and sold energy information from the meter (http query) and calculates net net consumption for current period.

## PowerGuru
Gets state info (e.g "now spot price is low") from a PowerGuru service (http query). All spot price data and energy forecasts are preprocessed by a PowerGuru instance.

## DS18B20 temperature sensor
Currently one sensor is supported per a PowerGuru Node device.

## GPIO controlled switches
One or more low voltage switches which can controll grid voltage switches. Leave grid voltage installation to a certified professinal. 

# Current status
The software is under development (alfa testing)


# Thanks
    - https://github.com/me-no-dev/ESPAsyncWebServer
    - https://arduinojson.org/ 
