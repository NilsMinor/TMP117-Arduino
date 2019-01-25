
#include "TMP117.h"


TMP117::TMP117 (uint8_t addr) {
  
  address = addr;
  //Wire.begin();
}




void TMP117::i2cWrite2B (uint8_t reg, uint16_t data){
  Wire.beginTransmission(address); 
  Wire.write(reg);
  Wire.write( (data<<8) );
  Wire.write( (data&0xff) );
  Wire.endTransmission();
  delay(10);
}

uint16_t TMP117::i2cRead2B (uint8_t reg) {
  uint8_t data[2]; 
  int16_t datac = 0;   

  Wire.beginTransmission(address); 
  Wire.write(reg); 
  Wire.endTransmission(); 

  Wire.requestFrom(address, 2); 

  if(Wire.available() <= 2){  
    data[0] = Wire.read(); 
    data[1] = Wire.read(); 
    datac = ((data[0] << 8) | data[1]); 
  }
  return datac;
}

double TMP117::calcTemperature (uint16_t raw) {
  return raw * 0.0078125;
}


double TMP117::getTemperature (void) {
  uint16_t temp = i2cRead2B( TMP117_REG_TEMPERATURE );
  return calcTemperature ( temp );
}

uint16_t  TMP117::getDeviceRev (void) {
  // read bits [15:12]
  uint16_t raw = i2cRead2B( TMP117_REG_DEVICE_ID );
  
  return ( (raw >> 12) & 0x3);
}
uint16_t  TMP117::getDeviceID (void) {
  // read bits [11:0]
  uint16_t raw = i2cRead2B( TMP117_REG_DEVICE_ID );
  
  return (raw & 0x0fff);
}
