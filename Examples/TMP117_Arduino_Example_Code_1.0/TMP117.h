
/*
 * Arduino library for TMP117 temperature sensor. Based on TI training resources https://training.ti.com/how-interface-tmp116-tmp117-temperature-sensors-arduino 
 * 
 */

#ifndef _TMP117_H_
#define _TMP117_H_

#include "Wire.h"
#include "Arduino.h"

#define TMP117_REG_TEMPERATURE          0x00


class TMP117 {
  public:
    TMP117 (uint8_t addr);
    double getTemperature (void);

  private:
    uint8_t   address;
    void      i2cWrite2B (uint8_t reg, uint16_t data);
    uint16_t  i2cRead2B (uint8_t reg);
    
    double    calcTemperature (uint16_t raw);
  
};




#endif
