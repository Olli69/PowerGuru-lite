# PowerGuru Node
ESP8266 based application for reading energy meters, sensors and controlling switches based on price, energy forecast and grid (consumption and production) information. See main project project [PowerGuru Server](https://github.com/Olli69/powerguru/)

# Interfaces

## Energy meters

### Shelly 3EM energy meter
Reads consumed and sold energy information from the meter (http query) and calculates net net consumption for current period.

### Inverter with Fronius Solar API
Solar power production values energy (cumulated) and current power can be fetched from Fronius inverter. Tested with FRONIUS Eco 27.0-3-S .

### Inverter with SMA Modbus TCP
Solar power production values energy (cumulated) and current power can be fetched from SMA Inverter via Modbus TCP interface. Tested with STP8.0-3AV-40 (Sunny Tripower 8.0).

## PowerGuru Server
All day-ahead (spot) electricity price data and energy production forecasts are preprocessed by a PowerGuru Server instance. Powergutu Node gets state info (e.g "now spot price is low") from a PowerGuru Server service (http query). 

## DS18B20 temperature sensor
Currently one sensor is supported per a PowerGuru Node device.

## GPIO controlled switches
One or more low voltage switches which can control grid voltage switches. Leave grid voltage installation to a certified professinal. 

# Current status
The software is under development (alfa testing)

# Hardware Configuration
Example 2 channel configuration with ESP8266  based 2 channel relay module ja temperature sensor. With this configuration you can control for example 12VDC (or 24VDC) controlled AC relays or a water-based underfloor heating system (select between pre-programmed temerature levels). Waterproof temperature sensor can be attached to hot water pipeline or a water boiler, to sensor water temperature.

## Required modules/parts:
### ESP8266 microcontroller with relays
- ESP8266 (ESP-12F) Relay Module [Aliexpress](https://www.aliexpress.com/item/1005001908708140.html)
- USB to TTL converter for initial setup (this board does not have an USB connector), [e.g.](https://www.aliexpress.com/item/32529737466.html?), 
- female-female Dupont Dupont jumper lines for TTL connector, 4 pcs, [e.g.](https://www.aliexpress.com/item/1005003007413890.html)
- Optional DS3231 real time clock (RTC). Without RTC clock sync from internet or manual clock sync is needed after restart (including power breaks).

### Temperature sensor DS18B20
- DS18B20 waterproof temperature sensor, [Aliexpress](https://www.aliexpress.com/item/4000550061662.html)
- ≅ 5kΩ resistor for 1-wire pull-up resistor, between data and voltage connector
- Electric cables (3 wire) for connecting the sensor
### 12V wiring:
- 12V power supply, supplying power to ESP8266 module and pulling external relays
- Electric wires for 12V connection (two color, e.g. red and black recommended)
- female and male connecters for 12V, optional [Aliexpress](https://www.aliexpress.com/item/4000085878441.html)
- Screw terminal blocks, optional [Aliexpress](https://www.aliexpress.com/item/32939185688.html)





- 
 
 Tools:
 - Soldering iron, soldering tin
 - Tin suction gun (optional) for desoldering



# Thanks
    - https://github.com/me-no-dev/ESPAsyncWebServer  - Control UI web server
    - https://github.com/ayushsharma82/ElegantOTA  - OTA update
    - https://arduinojson.org/ - Processing web API results 