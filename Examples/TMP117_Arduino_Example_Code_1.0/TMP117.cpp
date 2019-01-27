
#include "TMP117.h"


TMP117::TMP117 (uint8_t addr) {
  
  address = addr;
  allert_pin = -1;
  //Wire.begin();
}

void TMP117::setAllert (void (*allert_callback)(void), uint8_t pin) {
  allert_pin = pin;
  pinMode(pin, INPUT_PULLUP);
  setConvMode (OneShot);
  //setMode (Data);
  
  attachInterrupt(digitalPinToInterrupt(pin), allert_callback, CHANGE); // Sets up pin 2 to trigger "alert" ISR when pin changes H->L and L->H
}

void TMP117::setAllertTemperature (double lowtemp, double hightemp) {
   // Set temperature threshold 
 const uint8_t highlimH = B00001101;   // High byte of high lim
 const uint8_t highlimL = B10000000;   // Low byte of high lim  - High 27 C
 const uint8_t lowlimH = B00001100;    // High byte of low lim
 const uint8_t lowlimL = B00000000;    // Low byte of low lim   - Low 24 C


 //uint16_t high_temp_value = ((highlimH << 8) | highlimL);
 //uint16_t low_temp_value = ((lowlimH << 8) | lowlimL);

 uint16_t high_temp_value = hightemp / TMP117_RESOLUTION;
 uint16_t low_temp_value = lowtemp / TMP117_RESOLUTION;

 i2cWrite2B (TMP117_REG_TEMP_HIGH_LIMIT , high_temp_value);
 i2cWrite2B (TMP117_REG_TEMP_LOW_LIMIT , low_temp_value);  
}

uint16_t TMP117::readConfig (void) {
  uint16_t reg_value = i2cRead2B ( TMP117_REG_CONFIGURATION );
  bool high_alert = reg_value >> 15 & 1UL;
  bool low_alert = reg_value >> 14 & 1UL;   
  bool data_ready =  reg_value >> 13 & 1UL;   
  bool eeprom_busy =  reg_value >> 12 & 1UL;   

  if (data_ready)
    Serial.println(reg_value, BIN);
  return reg_value;  
}

void TMP117::setConvMode ( TMP117_CMODE cmode) {
   uint16_t reg_value = readConfig ();
  
   if (cmode == Continuous) {      // Bit [11:10] = 00
      reg_value &= ~(1UL << 11); 
      reg_value &= ~(1UL << 10);
   } else if (cmode == Shutdown) { // Bit [11:10] = 01
      reg_value &= ~(1UL << 11);
      reg_value |= 1UL << 10;
   } else if (cmode == OneShot) {  // Bit [11:10] = 11  
      //reg_value |= 1UL << 11;
      //reg_value |= 1UL << 10;
      bitSet (reg_value , 11);
      bitSet (reg_value , 10);
   }
   
   writeConfig ( reg_value );
}
void TMP117::setConvTime ( TMP117_CONVT convtime ) {
  
}
void TMP117::setAveraging ( TMP117_AVE ave ) {
  
}

void TMP117::writeConfig (uint16_t config_data) {
  i2cWrite2B (TMP117_REG_CONFIGURATION, config_data);
}
void TMP117::setMode ( TMP117_MODE mode) {
  uint16_t reg_value = readConfig ();
  if (mode == Thermal) {
    reg_value |= 1UL << 4;    // change to thermal mode
    reg_value &= ~(1UL << 2); // set pin as alert flag
    reg_value &= ~(1UL << 3); // alert pin low active
  }
  else if (mode == Alert) {
    reg_value &= ~(1UL << 4); // change to alert mode
    reg_value &= ~(1UL << 2); // set pin as alert flag
    reg_value &= ~(1UL << 3); // alert pin low active
  } 
  else if (mode == Data) {
    reg_value |= 1UL << 2;    // set pin as data ready flag
  } 
  writeConfig ( reg_value );
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
  return raw * TMP117_RESOLUTION;
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
