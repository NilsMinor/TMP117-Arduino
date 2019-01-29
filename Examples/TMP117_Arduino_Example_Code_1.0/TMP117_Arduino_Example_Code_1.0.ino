/*
 * @file    ???.ino
 * @author  Nils Minor
 * 
 * @brief   This example shows ...

*/
// Select the correct address setting
uint8_t ADDR_GND =  0x48;   // 1001000 
uint8_t ADDR_VCC =  0x49;   // 1001001
uint8_t ADDR_SDA =  0x4A;   // 1001010
uint8_t ADDR_SCL =  0x4B;   // 1001011
uint8_t ADDR =  ADDR_GND;
// address pin is connected to ground --> 0x48

#include "TMP117.h"

TMP117 t(ADDR);
int i = 0;

/************************* Initialization Setup Function **************************/
void setup() {

  // Initiate wire library and serial communication
  Wire.begin();
  Serial.begin(115200);
  pinMode (13, OUTPUT); 

  t.setAllertCallback ( temperature_allert, 7);
  t.setAllertTemperature (24, 28);
  t.init ( new_temperature );
}

/************************* Infinite Loop Function **********************************/
void loop() {

  t.readConfig();
  delay(100);
}

void new_temperature ( void ) {
  //digitalWrite (13, HIGH);
  Serial.print ("Temperature : ");
  Serial.print (t.getTemperature());
  Serial.println (" °C");
  delay (10);
  //digitalWrite (13, LOW);
}

void temperature_allert (void) {
 
  if (t.getAlertType () == HIGHALERT) {
    digitalWrite (13, HIGH);
    Serial.println ("High Temperature allert ");
  }
  else if (t.getAlertType () == LOWALERT) {
    digitalWrite (13, HIGH);
    Serial.println ("Low Temperature allert ");
  }
  else {
    digitalWrite (13, LOW);
  }
  
}
