/*
 * @file    temperature_callback.ino
 * @author  Nils Minor
 * 
 * @brief   This example shows the more advanced usability of the driver
 *          - setup diffrent sensor modes
 *          - read configuration register using >update< 
 *          - use callback for new temperature 
 *          
 * Connect the sensor via I2C pins to the Arduino. If you want an alert signal,           
 * you have to connect the alert output of the sensor to an interrupt pin 
 * of the Arduino.
 */

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

// Select the correct address setting
uint8_t ADDR_GND =  0x48;   // 1001000 
uint8_t ADDR_VCC =  0x49;   // 1001001
uint8_t ADDR_SDA =  0x4A;   // 1001010
uint8_t ADDR_SCL =  0x4B;   // 1001011
uint8_t ADDR =  ADDR_GND;

#include "TMP117.h"

TMP117 tmp(ADDR);
void setup() {

  // Initiate wire library and serial communication
  Wire.begin();
  Serial.begin(115200);
  pinMode (LED_BUILTIN, OUTPUT); 

  tmp.init ( new_temperature );     // set callback function. will be called if there is new sensor data
  tmp.setConvMode (TMP117_CMODE::CONTINUOUS);     // contious measurement, also ONESHOT or SHUTWDOWN possible

  uint8_t setup_nr = 3;             // select an example setup to see diffrent modes
  switch (setup_nr) {
    case 1: tmp.setConvTime (TMP117_CONVT::C15mS5); // 1. setup C125mS+NOAVE = 15.5 mS measurement time
            tmp.setAveraging (TMP117_AVE::NOAVE);
            break;
    case 2: tmp.setConvTime (TMP117_CONVT::C125mS); // 2. setup C125mS+AVE8 = 125 mS measurement time
            tmp.setAveraging (TMP117_AVE::AVE8);
            break;
    case 3: tmp.setConvTime (TMP117_CONVT::C125mS); // 3. setup C500mS+AVE32 = 500 mS measurement time
            tmp.setAveraging (TMP117_AVE::AVE32);
            break;
    case 4: tmp.setConvTime (TMP117_CONVT::C4S);    // 4. setup C1S+AVE64 = 1000 mS measurement time
            tmp.setAveraging (TMP117_AVE::AVE64);
            break;
    default: setup_nr = 1;
  }  
}

/************************* Infinite Loop Function **********************************/
void loop() {

  tmp.update();   // used to update the sensor/read out configuration register
}

void new_temperature ( void ) {
  digitalWrite (LED_BUILTIN, !digitalRead(LED_BUILTIN));
  Serial.print ("Temperature : ");
  Serial.print (tmp.getTemperature());
  Serial.println (" °C");
}

