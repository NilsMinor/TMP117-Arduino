
#include "TMP117.h"


TMP117::TMP117 (uint8_t addr) {
  
  address = addr;
  //Wire.begin();
}




void TMP117::i2cWrite2B (uint8_t reg, uint16_t data){
  Wire.beginTransmission(address); 
  Wire.write( reg );
  Wire.write( (data<<8) );
  Wire.write( (data&0xff) );
  Wire.endTransmission( );
  delay(10);
}

uint16_t TMP117::i2cRead2B (uint8_t reg) {
  uint8_t data[2] = {0}; 
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


void TMP117::writeEEPROM (uint16_t data, uint8_t eeprom) {
  if (!EEPROMisBusy()) {
    unlockEEPROM();
      switch (eeprom) {
        case 1 : i2cWrite2B ( TMP117_REG_EEPROM1, data); break;
        case 2 : i2cWrite2B ( TMP117_REG_EEPROM2, data); break;
        case 3 : i2cWrite2B ( TMP117_REG_EEPROM3, data); break;
        default: Serial.println("EEPROM value must be between 1 and 3");
      }
    lockEEPROM();
  }
  else {
    Serial.println("EEPROM is busy");
  }
}
uint16_t  TMP117::readEEPROM (uint8_t eeprom) {
  // read the 48 bit number from the EEPROM
  if (!EEPROMisBusy()) {
    uint16_t eeprom_data = 0;
    switch (eeprom) {
        case 1 : eeprom_data = i2cRead2B( TMP117_REG_EEPROM1 ); break;
        case 2 : eeprom_data = i2cRead2B( TMP117_REG_EEPROM2 ); break;
        case 3 : eeprom_data = i2cRead2B( TMP117_REG_EEPROM3 ); break;
        default: Serial.println("EEPROM value must be between 1 and 3");
      }
    return eeprom_data;
  }
  else {
    Serial.println("EEPROM is busy");
  }
}

void TMP117::lockEEPROM (void) {
  // clear bit 15
  uint16_t code = 0;
  code &= ~(1UL << 15);
  i2cWrite2B ( TMP117_REG_EEPROM_UNLOCK, code );
  delay(100);
}
void TMP117::unlockEEPROM (void) {
  // set bit 15
  uint16_t code = 0;
  code |= 1UL << 15;
  i2cWrite2B ( TMP117_REG_EEPROM_UNLOCK, code );
  delay(100);
}
bool TMP117::EEPROMisBusy (void) {
  // Bit 14 indicates the busy state of the eeprom
  uint16_t code = i2cRead2B ( TMP117_REG_EEPROM_UNLOCK );
  return false;//(bool) ((code >> 14) & 0x01);
}
