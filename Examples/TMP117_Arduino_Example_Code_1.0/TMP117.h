
/*
 * Arduino library for TMP117 temperature sensor. Based on TI training resources https://training.ti.com/how-interface-tmp116-tmp117-temperature-sensors-arduino 
 * 
 * datasheet http://www.ti.com/lit/ds/symlink/tmp117.pdf
 */

#ifndef _TMP117_H_
#define _TMP117_H_

#include "Wire.h"
#include "Arduino.h"

#define TMP117_REG_TEMPERATURE          0x00
#define TMP117_REG_CONFIGURATION        0x01
#define TMP117_REG_TEMP_HIGH_LIMIT      0x02
#define TMP117_REG_TEMP_LOW_LIMIT       0x03

#define TMP117_REG_EEPROM_UNLOCK        0x04
#define TMP117_REG_EEPROM1              0x05
#define TMP117_REG_EEPROM2              0x06
#define TMP117_REG_EEPROM3              0x08

#define TMP117_REG_TEMPERATURE_OFFSET   0x07
#define TMP117_REG_DEVICE_ID            0x0F



class TMP117 {
  public:
              TMP117 (uint8_t addr);
    double    getTemperature (void);
    uint16_t  getDeviceID (void);
    uint16_t  getDeviceRev (void);
    

  private:
    uint8_t   address;
    void      i2cWrite2B (uint8_t reg, uint16_t data);
    uint16_t  i2cRead2B (uint8_t reg);
    
    double    calcTemperature (uint16_t raw);
  
};




#endif
