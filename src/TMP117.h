
/*!
 * @file    TMP117.h
 * @author  Nils Minor
 * 
 * @license  GNU GENERAL PUBLIC LICENSE (see license.txt)
 * 
 * v1.0.0   - Initial library version
 * 
 * 
 * 
 * Arduino library for TMP117 high-precision digital temperature sensor.
 *
 * Features:
 *  - meet ASTM E1112 and ISO 80601 requirements
 *  - serve as a single chip digital alternative to a Platinum RTD (Class AA RTD)
 *  - 100% tested on a production setup that is NIST traceable
 *  - ±0.1°C (Maximum) From –20°C to +50°C
 *  - ±0.15°C (Maximum) From –40°C to +70°C
 *  - ±0.2°C (Maximum) From –40°C to +100°C
 *  - ±0.25°C (Maximum) From –55°C to +125°C
 *  - ±0.3°C (Maximum) From –55°C to +150°C
 *  -Low Power Consumption 3.5-µA, 1-Hz Conversion Cycle
 *  
 *  Datasheet http://www.ti.com/lit/ds/symlink/tmp117.pdf
 *  
*/

#ifndef _TMP117_H_
#define _TMP117_H_

#include <Wire.h>
#include <Arduino.h>

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
              AVG       0       1       2       3
      CONV  averaging  (0)     (8)     (32)   (64)
        0             15.5ms  125ms   500ms    1s     C15mS5
        1             125ms   125ms   500ms    1s     C125mS
        2             250ms   250ms   500ms    1s     C250mS
        3             500ms   500ms   500ms    1s     C500mS
        4             1s      1s      1s       1s     C1S
        5             4s      4s      4s       4s     C4S
        6             8s      8s      8s       8s     C8S
        7             16s     16s     16s      16s    C16S
*/

enum class TMP117_PMODE     {THERMAL = 0, ALERT, DATA};                                 //!<  Pin mode 
enum class TMP117_CMODE     {CONTINUOUS = 0, SHUTDOWN = 1, ONESHOT = 3};                //!<  Conversion mode 
enum class TMP117_CONVT     {C15mS5 = 0, C125mS, C250mS, C500mS, C1S, C4S, C8S, C16S};  //!<  Conversion time
enum class TMP117_AVE       {NOAVE = 0, AVE8, AVE32, AVE64};                            //!<  Averaging mode
enum class TMP117_ALERT     {NOALERT = 0, HIGHALERT, LOWALERT};                         //!<  Alert type 

class TMP117 {

  public:
              TMP117 (uint8_t addr);
    void      init ( void (*_newDataCallback) (void) );
    void      update (void);
    void      softReset ( void );

    void      setAlertMode ( TMP117_PMODE mode);
    void      setAllertCallback ( void (*allert_callback)(void), uint8_t pin );
    void      setAllertTemperature ( double lowtemp, double hightemp );
    void      setConvMode ( TMP117_CMODE cmode);
    void      setConvTime ( TMP117_CONVT convtime );
    void      setAveraging ( TMP117_AVE ave );
    void      setOffsetTemperature ( double offset );
    void      setTargetTemperature ( double target );

    double    getTemperature ( void );
    uint16_t  getDeviceID ( void );
    uint16_t  getDeviceRev ( void );
    double    getOffsetTemperature ( void );
    TMP117_ALERT getAlertType ( void );
    
    void      writeEEPROM ( uint16_t data, uint8_t eeprom_nr );
    uint16_t  readEEPROM ( uint8_t eeprom_nr );
    uint16_t  readConfig ( void );
    void      printConfig ( void );
   
  private:
  
    uint8_t   address;
    int8_t    alert_pin;
    TMP117_ALERT alert_type;

    void      i2cWrite2B ( uint8_t reg, uint16_t data );
    uint16_t  i2cRead2B ( uint8_t reg );
    void      writeConfig ( uint16_t config_data );
    void      lockEEPROM ( void );
    void      unlockEEPROM ( void );
    bool      EEPROMisBusy ( void );

    void      (*newDataCallback) ( void );
    
};




#endif
