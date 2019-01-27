/*
    Nils Minor

*/


// address pin is connected to ground --> 0x48

#include "TMP117.h"

TMP117 t(0x48);
/************************* Initialization Setup Function **************************/
void setup() {

  // Initiate wire library and serial communication
  Wire.begin();
  Serial.begin(115200);

  t.setAllert ( temperature_allert, 6);
  t.setAllertTemperature (24, 30);
  
}

/************************* Infinite Loop Function **********************************/
void loop() {

  Serial.println(t.getTemperature());
  t.readConfig();
  /*
    Serial.print("Device ID : ");
    Serial.print (t.getDeviceID());
    Serial.print("  Revision : ");
    Serial.println (t.getDeviceRev());
  */
  
  //Serial.println("Write to EEPROM");
  //t.writeEEPROM (0x1234, 1);
  //t.writeEEPROM (0x5678, 2);
  //t.writeEEPROM (0x9999, 3);
  //delay(1000);

  /*
  uint16_t eeprom_data [3] = {0};
  eeprom_data [0] = t.readEEPROM(1);
  eeprom_data [1] = t.readEEPROM(2);
  eeprom_data [2] = t.readEEPROM(3);

  Serial.print("Read from EEPROM : ");
  Serial.print(eeprom_data [0], HEX); Serial.print(" ");
  Serial.print(eeprom_data [1], HEX); Serial.print(" ");
  Serial.println(eeprom_data [2], HEX);*/
  
  delay(100);

}

void temperature_allert (void) {
  Serial.print ("Temperature allert : ");
  Serial.print (t.getTemperature());
  Serial.print (" °C");
}
