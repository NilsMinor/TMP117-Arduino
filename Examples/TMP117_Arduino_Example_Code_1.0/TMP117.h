
/*
 * Arduino library for TMP117 temperature sensor. Based on TI training resources https://training.ti.com/how-interface-tmp116-tmp117-temperature-sensors-arduino 
 * 
 * datasheet http://www.ti.com/lit/ds/symlink/tmp117.pdf
 * 
 * 
 * EEPROM is ordered like a 64 bit number 
 *         X   | EEPROM3 | EEPROM2 | EEPROM1 |
 *  Bits 63:48 | 47-32  | 31: 16  |  15:0   |
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

#define TMP117_RESOLUTION               (double)0.0078125

typedef void (*allert_callback)(void);


/*  Conversion Cycle Time in CC Mode        
 *            AVG       0       1       2       3 
 *    CONV  averaging  (0)     (8)     (32)   (64)
 *      0             15.5ms  125ms   500ms    1s     C15mS5 
 *      1             125ms   125ms   500ms    1s     C125mS
 *      2             250ms   250ms   500ms    1s     C250mS
 *      3             500ms   500ms   500ms    1s     C500mS
 *      4             1s      1s      1s       1s     C1S
 *      5             4s      4s      4s       4s     C4S
 *      6             8s      8s      8s       8s     C8S
 *      7             16s     16s     16s      16s    C16S
 */
 
enum TMP117_MODE      {Thermal, Alert, Data};
enum TMP117_CMODE     {Continuous = 0, Shutdown = 1, OneShot = 3};
enum TMP117_CONVT     {C15mS5 = 0, C125mS, C250mS, C500mS, C1S, C4S, C8S, C16S};
enum TMP117_AVE       {NOAVE = 0, AVE8, AVE32, AVE64};
enum TMP117_ALERT     {NoAlert = 0, HighTempAlert, LowTempAlert};

class TMP117 {
  
  public:
              TMP117 (uint8_t addr);
    void      init ( void (*_newDataCallback) (void) );
    
    void      setAlertMode ( TMP117_MODE mode);
    void      setConvMode ( TMP117_CMODE cmode);
    void      setConvTime ( TMP117_CONVT convtime );
    void      setAveraging ( TMP117_AVE ave );
    void      setAllert (void (*allert_callback)(void), uint8_t pin);
    void      setAllertTemperature ( double lowtemp, double hightemp );
    void      setOffsetTemperature ( double offset );
    void      setTargetTemperature ( double target );
    
    double    getTemperature (void);
    uint16_t  getDeviceID (void);
    uint16_t  getDeviceRev (void);
   
    void      writeEEPROM (uint16_t data, uint8_t eeprom);
    uint16_t  readEEPROM (uint8_t eeprom);
    uint16_t  readConfig (void);

    TMP117_ALERT getAlertType ( void );

  private:
    
    uint8_t   address;
    int8_t    alert_pin;
    TMP117_ALERT alert_type;

    void      (*newDataCallback) (void); 
    void      printConfig (uint16_t reg_value);
    void      writeConfig (uint16_t config_data);
    double    calcTemperature (uint16_t raw);
    void      i2cWrite2B (uint8_t reg, uint16_t data);
    uint16_t  i2cRead2B (uint8_t reg);
    void      lockEEPROM (void);
    void      unlockEEPROM (void);
    bool      EEPROMisBusy (void);
      
};




#endif
