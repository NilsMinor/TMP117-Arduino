# TMP117-Arduino


The TMP117 is a high-accuracy, low-power, digital temperature sensor. It can be used for devices that need to meet the NIST traceability like it is oftne used in medical devices.
This library exposes it's functionality using the wire library an I2C communication. The library is written in c++ to support the Arduino plattform.

## Functionality

- [x] Read measured temperature
- [X] Read from internal EEPROM
- [ ] Write to internal EEPROM
- [x] Implement allert function (HIGH/LOW temperature allerts)
- [ ] Implement temperature offset function
- [x] Make configuration accessible  