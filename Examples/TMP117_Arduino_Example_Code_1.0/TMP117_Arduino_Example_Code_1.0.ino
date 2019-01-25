/* 
 *  Nils Minor
 *  
 */


// address pin is connected to ground --> 0x48

 #include "TMP117.h"

 TMP117 t(0x48);
/************************* Initialization Setup Function **************************/
 void setup(){
  
  // Initiate wire library and serial communication
  Wire.begin();       
  Serial.begin(115200); 
 }

/************************* Infinite Loop Function **********************************/
 void loop(){
 
  //Serial.println(t.getTemperature());

  Serial.print("Device ID : ");
  Serial.print (t.getDeviceID());
  Serial.print("  Revision : ");
  Serial.println (t.getDeviceRev());
  delay(1000);
 }
