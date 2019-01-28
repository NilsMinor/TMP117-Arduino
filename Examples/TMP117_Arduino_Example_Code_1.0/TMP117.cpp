
#include "TMP117.h"


TMP117::TMP117 (uint8_t addr) {
  Wire.begin();

  address = addr;
  alert_pin = -1;
  alert_type = NoAlert;
}
void TMP117::init ( void (*_newDataCallback) (void) ) {
  setConvMode (Continuous);
  setConvTime (C125mS);
  setAveraging (AVE8);
  //setMode (Data);
  newDataCallback = _newDataCallback;
}

void      TMP117::setAllert (void (*allert_callback)(void), uint8_t pin) {
  alert_pin = pin;
  pinMode(pin, INPUT_PULLUP);
   
  attachInterrupt(digitalPinToInterrupt(pin), allert_callback, CHANGE); // Sets up pin 2 to trigger "alert" ISR when pin changes H->L and L->H
}

void      TMP117::setAllertTemperature (double lowtemp, double hightemp) {
   // Set temperature threshold 
 const uint8_t highlimH = B00001101;   // High byte of high lim
 const uint8_t highlimL = B10000000;   // Low byte of high lim  - High 27 C
 const uint8_t lowlimH = B00001100;    // High byte of low lim
 const uint8_t lowlimL = B00000000;    // Low byte of low lim   - Low 24 C


 uint16_t high_temp_value = ((highlimH << 8) | highlimL);
 uint16_t low_temp_value = ((lowlimH << 8) | lowlimL);

 //uint16_t high_temp_value = hightemp / TMP117_RESOLUTION;
 //uint16_t low_temp_value = lowtemp / TMP117_RESOLUTION;

 i2cWrite2B (TMP117_REG_TEMP_HIGH_LIMIT , high_temp_value);
 i2cWrite2B (TMP117_REG_TEMP_LOW_LIMIT , low_temp_value);  
}
uint16_t  TMP117::readConfig (void) {
  uint16_t reg_value = i2cRead2B ( TMP117_REG_CONFIGURATION );
  bool high_alert = reg_value >> 15 & 1UL;
  bool low_alert = reg_value >> 14 & 1UL;   
  bool data_ready =  reg_value >> 13 & 1UL;   
  bool eeprom_busy =  reg_value >> 12 & 1UL;   

  if (data_ready && newDataCallback != NULL)
    newDataCallback ();

  if (reg_value >> 15 & 1UL) {
    alert_type = HighTempAlert;
  }
  else if (reg_value >> 14 & 1UL) {
    alert_type = LowTempAlert;
  }
  else {
    alert_type = NoAlert;
  }
  
  printConfig ( reg_value );
    
  return reg_value;  
}
TMP117_ALERT TMP117::getAlertType ( void ) {
  return alert_type;
}
void      TMP117::printConfig (uint16_t reg_value) {

  Serial.println(reg_value, BIN);

  Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  Serial.print ("HIGH alert:  ");
  Serial.println( ( reg_value >> 15) & 0b1 , BIN);
  Serial.print ("LOW alert:   ");
  Serial.println( ( reg_value >> 14) & 0b1 , BIN);
  Serial.print ("Data ready:  ");
  Serial.println( ( reg_value >> 13) & 0b1 , BIN);
  Serial.print ("EEPROM busy: ");
  Serial.println( ( reg_value >> 12) & 0b1 , BIN);
  Serial.print ("MOD[1:0]:    ");
  Serial.println( ( reg_value >> 10) & 0b11 , BIN);
  Serial.print ("CONV[2:0]:   ");
  Serial.println( ( reg_value >> 7)  & 0b111 , BIN);
  Serial.print ("AVG[1:0]:    ");
  Serial.println( ( reg_value >> 5)  & 0b11 , BIN);
  Serial.print ("T/nA:        ");
  Serial.println( ( reg_value >> 4) & 0b1 , BIN);
  Serial.print ("POL:         ");
  Serial.println( ( reg_value >> 3) & 0b1 , BIN);
  Serial.print ("DR/Alert:    ");
  Serial.println( ( reg_value >> 2) & 0b1 , BIN);
  Serial.print ("Soft_Reset:  ");
  Serial.println( ( reg_value >> 1) & 0b1 , BIN);
}
void      TMP117::setConvMode ( TMP117_CMODE cmode) {
   uint16_t reg_value = readConfig ();
   reg_value &= ~((1UL << 11) | (1UL << 10));       // clear bits
   reg_value = reg_value | ( cmode  & 0x03 ) << 10; // set bits
   
   /*if (cmode == Continuous) {      
      reg_value &= ~((1UL << 11) | (1UL << 10));        // Bit [11:10] = 00
   } else if (cmode == Shutdown) { 
      reg_value = (reg_value & ~(1 << 11)) | (1 << 10); // Bit [11:10] = 01
   } else if (cmode == OneShot) {  
      reg_value |= (1 << 11) | (1 << 10);               // Bit [11:10] = 11   
   }    */
   
   writeConfig ( reg_value );
}
void      TMP117::setConvTime ( TMP117_CONVT convtime ) {
  uint16_t reg_value = readConfig ();
  reg_value &= ~((1UL << 9) | (1UL << 8) | (1UL << 7));       // clear bits
  reg_value = reg_value | ( convtime  & 0x07 ) << 7;          // set bits
  writeConfig ( reg_value );
}
void      TMP117::setAveraging ( TMP117_AVE ave ) {
  uint16_t reg_value = readConfig ();
  reg_value &= ~((1UL << 6) | (1UL << 5) );       // clear bits
  reg_value = reg_value | ( ave & 0x03 ) << 5;          // set bits
  writeConfig ( reg_value );
}
void      TMP117::writeConfig (uint16_t config_data) {
  i2cWrite2B (TMP117_REG_CONFIGURATION, config_data);
}
void      TMP117::setMode ( TMP117_MODE mode) {
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
void      TMP117::i2cWrite2B (uint8_t reg, uint16_t data){
  Wire.beginTransmission(address); 
  Wire.write( reg );
  Wire.write( (data>>8) );
  Wire.write( (data&0xff) );
  Wire.endTransmission( );
  delay(10);
}
uint16_t  TMP117::i2cRead2B (uint8_t reg) {
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
double    TMP117::calcTemperature (uint16_t raw) {
  return raw * TMP117_RESOLUTION;
}
double    TMP117::getTemperature (void) {
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


void      TMP117::writeEEPROM (uint16_t data, uint8_t eeprom) {
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
