/*
 * @file    prompt_interpreter.ino
 * @author  Nils Minor
 * 
 * @brief   This example shows how to easy control the sensor using a tiny interpreter
 *          - read user command and parse parameter
 *          - read EEPROM NIST UUID
 *          - read actual temperature
 *          - show offset/calibration function
 *          
 * Connect the sensor via I2C pins to the Arduino. If you want an alert signal,           
 * you have to connect the alert output of the sensor to an interrupt pin 
 * of the Arduino.
 * 
 */
 
// Select the correct address setting
uint8_t ADDR_GND =  0x48;   // 1001000 
uint8_t ADDR_VCC =  0x49;   // 1001001
uint8_t ADDR_SDA =  0x4A;   // 1001010
uint8_t ADDR_SCL =  0x4B;   // 1001011
uint8_t ADDR =  ADDR_GND;

#include "TMP117.h"

#define ALERT_PIN               7     // low active alert pin
#define LOW_TEMPERATURE_ALERT   20    // low alert at 20°C
#define HIGH_TEMPERATURE_ALERT  28    // highalert at 28°C

bool alert_flag = false;
TMP117 tmp(ADDR);
float floatnr = 0;
void setup() {

  // Initiate wire library and serial communication
  Wire.begin();
  Serial.begin(115200);

  tmp.init ( NULL );                // no callback
  tmp.setConvMode (TMP117_CMODE::CONTINUOUS);     // contious measurement, also ONESHOT or SHUTWDOWN possible

  tmp.setConvTime (TMP117_CONVT::C15mS5);         // 1. setup C125mS+NOAVE = 15.5 mS measurement time
  tmp.setAveraging (TMP117_AVE::NOAVE);     
}

/************************* Infinite Loop Function **********************************/
void loop() {

  tmp.update();

  /*  command   parameter   description
   *    0           X           print actual temperature
   *    1           X           print EEPROM NIST UUID [E1|E2|E3]
   *    2         float         set temperature offset like "2 20.5" sets the offset to 20.5°C
   *    3         float         calibrate sensor to target temperature like "3 30.5" will calibrate sensor to 30.5°C
   * 
   */
  if (Serial.available() > 0) {
      int inByte = Serial.read();
      
      switch (inByte) {
        case '0':
          Serial.print ("Temperature : ");
          Serial.print (tmp.getTemperature());
          Serial.println (" °C");
          break;
        case '1':
          Serial.print ("EEPROM : ");
          Serial.print (tmp.readEEPROM(1),HEX);
          Serial.print (tmp.readEEPROM(2),HEX);
          Serial.println (tmp.readEEPROM(3),HEX);
          break;
        case '2':
          floatnr = Serial.parseFloat( );
          tmp.setTargetTemperature ( floatnr );
          Serial.print("Calibrate temperature to : ");
          Serial.println( floatnr );
          break;
        case '3':
          floatnr = Serial.parseFloat( );
          tmp.setOffsetTemperature ( floatnr );
          Serial.println("3");
          break;
        case '4':
          Serial.println("4");
          break;
        case '5':
          Serial.println("5");
          break;
        case '\r':
          break;
        case '\n':
          break;
        default:
          Serial.println("Wrong number");
      }
      inByte = 0;
    }
}



