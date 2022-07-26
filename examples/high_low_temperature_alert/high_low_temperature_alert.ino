/*
 * @file    high_low_temperature_alert.ino
 * @author  Nils Minor
 * 
 * @brief   This example shows high/low temperature alert
 *          - read configuration register using >update< 
 *          - use callback for new temperature 
 *          - use callback/ISR pin to get alert for high and low temperature borders
 *          
 * Connect the sensor via I2C pins to the Arduino. If you want an alert signal,           
 * you have to connect the alert output of the sensor to an interrupt pin 
 * of the Arduino.
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
void setup() {

  // Initiate wire library and serial communication
  Wire.begin();
  Serial.begin(115200);

  tmp.init ( new_temperature );     // set callback function. will be called if there is new sensor data
  tmp.setConvMode (TMP117_CMODE::CONTINUOUS);     // contious measurement, also ONESHOT or SHUTWDOWN possible


  tmp.init ( new_temperature );
  tmp.setConvTime (TMP117_CONVT::C15mS5);         // 1. setup C125mS+NOAVE = 15.5 mS measurement time
  tmp.setAveraging (TMP117_AVE::NOAVE);
  
  tmp.setAlertMode(TMP117_PMODE::ALERT);          // use THERMAL or ALERT to activate alert feature
  tmp.setAllertCallback ( temperature_allert, ALERT_PIN );
  tmp.setAllertTemperature (LOW_TEMPERATURE_ALERT, HIGH_TEMPERATURE_ALERT);      
}

/************************* Infinite Loop Function **********************************/
void loop() {

  tmp.update();

  if (alert_flag) {
    if (tmp.getAlertType () == TMP117_ALERT::HIGHALERT) {
    Serial.print("High Temperature allert : ");
    Serial.print (tmp.getTemperature());
    Serial.println (" °C");
    }
    else if (tmp.getAlertType () == TMP117_ALERT::LOWALERT) {
      Serial.print("Low Temperature allert : ");
      Serial.print (tmp.getTemperature());
      Serial.println (" °C");
    }
    else {
      alert_flag = false;
    }
  }
}

// calback for new temperature
void new_temperature ( void ) {
  if (!alert_flag) {
    Serial.print ("Temperature : ");
    Serial.print (tmp.getTemperature());
    Serial.println (" °C");
  }
}

// callback function when an temperature alert occurs
void temperature_allert (void) {
  alert_flag = true;
 //Serial.println ("Lsssssssssssss ");
  
  
}
