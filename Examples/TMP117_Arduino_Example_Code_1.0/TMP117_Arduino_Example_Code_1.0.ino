/*
    Nils Minor

*/


// address pin is connected to ground --> 0x48

#include "TMP117.h"

TMP117 t(0x48);
int i = 0;

/************************* Initialization Setup Function **************************/
void setup() {

  // Initiate wire library and serial communication
  Wire.begin();
  Serial.begin(115200);
  pinMode (13, OUTPUT); 

  t.setAllert ( temperature_allert, 7);
  t.setAllertTemperature (24, 28);
  t.init ( new_temperature );
}

/************************* Infinite Loop Function **********************************/
void loop() {

  //Serial.println(t.getTemperature());
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
  Serial.println ("ALERT");

  if (t.getAlertType () == HighTempAlert) {
    digitalWrite (13, HIGH);
    Serial.print ("High Temperature allert : ");
    Serial.print (t.getTemperature());
    Serial.println (" °C");
  }
  else if (t.getAlertType () == LowTempAlert) {
    digitalWrite (13, HIGH);
    Serial.print ("Low Temperature allert : ");
    Serial.print (t.getTemperature());
    Serial.println (" °C");
  }
  else {
    digitalWrite (13, LOW);
  }
  

}
