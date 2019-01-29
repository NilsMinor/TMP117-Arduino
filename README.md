# TMP117-Arduino

The TMP117 is a high-accuracy, low-power, digital temperature sensor. It can be used for devices that need to meet the NIST traceability like it is often used in medical devices.
This library exposes it's functionality using the wire library for I2C communication. The library is written in C++ to support the Arduino plattform.

## Features

- Meet ASTM E1112 and ISO 80601 requirements 
- Serve as a single chip digital alternative to a Platinum RTD (Class AA RTD)
- 100% tested on a production setup that is NIST traceable
- ±0.1°C (Maximum) From –20°C to +50°C
- ±0.15°C (Maximum) From –40°C to +70°C
- ±0.2°C (Maximum) From –40°C to +100°C
- ±0.25°C (Maximum) From –55°C to +125°C
- ±0.3°C (Maximum) From –55°C to +150°C
- Low Power Consumption 3.5-µA, 1-Hz Conversion Cycle
- Programmable temperature alert limits

    
[For more informations see the datasheet](http://www.ti.com/lit/ds/symlink/tmp117.pdf)
   
## Functionality

- [x] Read measured temperature
- [X] Read from internal EEPROM
- [x] Make configuration accessible
- [x] Implement allert function (HIGH/LOW temperature allerts) 
- [x] Read/write from/to internal EEPROM
- [x] Implement temperature offset and calibration function
- [x] Implement software callbacks if new data/temperature is available
  