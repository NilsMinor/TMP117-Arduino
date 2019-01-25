/* 
 *  Nils Minor
 *  
 */


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
 
  Serial.println(t.getTemperature());

  delay(100);
 }
