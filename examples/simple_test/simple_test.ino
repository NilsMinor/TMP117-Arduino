/*
 * @file    simple_test.ino
 * @author  Nils Minor
 * 
 * @brief   This example shows the basic functionality of the driver
 *          - setup normal mode
 *          - poll new data by reading the temperature register
 *          
 * Connect the sensor via I2C pins to the Arduino. If you want an alert signal,           
 * you have to connect the alert output of the sensor to an interrupt pin 
 * of the Arduino.
 *
 */
#include "TMP117.h"

// Select the correct address setting
uint8_t ADDR_GND =  0x48;   // 1001000 
uint8_t ADDR_VCC =  0x49;   // 1001001
uint8_t ADDR_SDA =  0x4A;   // 1001010
uint8_t ADDR_SCL =  0x4B;   // 1001011
uint8_t ADDR =  ADDR_GND;

TMP117 tmp(ADDR);

/************************* Initialization Setup Function **************************/
void setup() {
  // Initiate wire library and serial communication
  Wire.begin();           
  Serial.begin(115200);

  /* The default setup is :
   *    Conversion mode = CONTINUOUS  ---> continuous
   *    Conversion time = C125mS      -|
   *    Averaging mode  = AVE8        -|-> new data every 125mS
   *    Alert mode      = data        ---> alert pin states that new data is available
   *    
   */
  tmp.init ( NULL );    
}

/************************* Infinite Loop Function **********************************/
void loop() {

  Serial.print ("Temperature : ");
  Serial.print (tmp.getTemperature());
  Serial.println (" °C");
  delay(100);
}



