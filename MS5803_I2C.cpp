// I2Cdev library collection - MS5803 I2C device class
// Based on Measurement Specialties MS5803 document, 3/25/2013 (DA5803-01BA_010)
// 4/12/2014 by Ryan Neve <Ryan@PlanktosInstruments.com>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2014 Ryan Neve

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "MS5803_I2C.h"


/** Default constructor, uses default I2C address.
 * @see MPU6050_DEFAULT_ADDRESS
 */
MS5803::MS5803() {
    _dev_address = MS5803_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see MS5803_DEFAULT_ADDRESS
 * @see MS5803_ADDRESS_AD0_LOW
 * @see MS5803_ADDRESS_AD0_HIGH
 */
MS5803::MS5803(uint8_t address) {
    _dev_address = address;
}
// Because sometimes you want to set the address later.
void MS5803::setAddress(uint8_t address) {
	_dev_address = address;
}

/** Power on and prepare for general usage.
 * This will reset the device to make sure that the calibration PROM gets loaded into 
 * the internal register. It will then read the PROM
 */
void MS5803::initialize(uint8_t model) {
    switch (model) {
      case (1 ): // THese are all supported
      case (2 ):
      case (5 ):
      case (14):
      case (30):
        _model = model;
        Serial.print("Model entered ("); Serial.print(model); Serial.println(") IS supported.");
        break;
      default:
        Serial.print("Model entered ("); Serial.print(model); Serial.println(") is not valid/supported.");
        _model = 1;
        break;
    }
    Serial.print("Max ATM set to ") ; Serial.println(_model);
    reset();
    _getCalConstants();
    _debug = false;
    press_atmospheric = 1.015; //default, can be changed.
}

bool MS5803::testConnection(){
  return I2Cdev::writeBytes(_dev_address,MS5803_D2_512,0,buffer);
}

uint16_t MS5803::reset(){
  // Not sure how to send no buffer to an address.
  return I2Cdev::writeBytes(_dev_address, MS5803_RESET,0,buffer);
}

void MS5803::_getCalConstants(){
  I2Cdev::readBytes(_dev_address,MS5803_PROM_C1,2,buffer);
  _c1_SENSt1 = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(_dev_address,MS5803_PROM_C2,2,buffer);
  _c2_OFFt1 = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(_dev_address,MS5803_PROM_C3,2,buffer);
  _c3_TCS = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(_dev_address,MS5803_PROM_C4,2,buffer);
  _c4_TCO = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(_dev_address,MS5803_PROM_C5,2,buffer);
  _c5_Tref = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(_dev_address,MS5803_PROM_C6,2,buffer);
  _c6_TEMPSENS = (((uint16_t)buffer[0] << 8) + buffer[1]);
}

void MS5803::debugCalConstants(){
  Serial.print("_c1_SENSt1 ");   Serial.println(_c1_SENSt1);
  Serial.print("_c2_OFFt1 ");    Serial.println(_c2_OFFt1);
  Serial.print("_c3_TCS ");      Serial.println(_c3_TCS);
  Serial.print("_c4_TCO ");      Serial.println(_c4_TCO);
  Serial.print("_c5_Tref ");     Serial.println(_c5_Tref);
  Serial.print("_c6_TEMPSENS "); Serial.println(_c6_TEMPSENS);
}

int32_t MS5803::getCalConstant(uint8_t constant_no){
  switch ( constant_no ) {
    case 1:
      return _c1_SENSt1;
    case 2:
      return _c2_OFFt1;
    case 3:
      return _c3_TCS;
    case 4:
      return _c4_TCO;
    case 5:
      return _c5_Tref;
    case 6:
      return _c6_TEMPSENS;
  }
}

float MS5803::getTemperature(bool debug){
  _debug = debug;
  return getTemperature();
}
float MS5803::getTemperature(){
  char buf[14];
  float T2 = 0;
  float off1 = 0;
  float off2 = 0;
  float sens2 = 0;
  // Request which buffer and oversampling ratio...
  I2Cdev::writeBytes(_dev_address, MS5803_D2_512,0,buffer);
  // Get buffer & convert
  I2Cdev::readBytes(_dev_address,MS5803_ADC_READ,3,buffer,2000);
  if ( _debug ) { Serial.print("    getTemperature buffer "); Serial.print(buffer[0],HEX); Serial.write(' ' ); Serial.print(buffer[1],HEX); Serial.write(' ' ); Serial.println(buffer[2],HEX);}
  _d2_temperature = ((long)buffer[0] << 16) + ((long)buffer[1] << 8) + (long)buffer[2];
  // now some calculations
  _dT = (float)_d2_temperature - ((long)_c5_Tref * 256);
  off1 =  _dT * (float)_c6_TEMPSENS/pow(2,23); //(1<<23);
  _TEMP = off1 + 2000;
  if ( _debug ) {
    Serial.println("First order values:");
    dtostrf(_d2_temperature,13,0,buf);
    Serial.print("    _d2_temperature = "); Serial.println(buf);
    //dtostrf(_dT,13,0,buf);
    Serial.print("    _dT = "); Serial.println(_dT);
    //dtostrf(off1,13,0,buf);
    Serial.print("    offset = "); Serial.println(off1);
    Serial.print("    _TEMP = "); Serial.println(_TEMP);
  }
  // Every variant does the calculations differently, so...
  switch (_model) {
    case (1):  //MS5803-01-----------------------------------------------------------
      _OFF  = ((float)_c2_OFFt1  * pow(2,16)) + (((float)_c4_TCO * _dT)  / pow(2,7));
      _SENS = ((float)_c1_SENSt1 * pow(2,15)) + (((float)_c3_TCS * _dT)  / pow(2,8));
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = 3 * pow(_dT,2) / pow(2,31);
        off2 = 3 * pow((_TEMP - 2000.0),2);
        sens2 = 7 * pow((_TEMP - 2000.0),2) / 8;
        if ( _TEMP < 1500.0 ) sens2 += 2 * pow((_TEMP + 1500.0),2);// below 15C
      }
      else if ( _TEMP > 4500.0 ) sens2 -= pow((_TEMP - 4500.0),2)  / 8;
      break;
    case (2):  //MS5803-02-----------------------------------------------------------
      _OFF  = ((float)_c2_OFFt1  * pow(2,17)) + (((float)_c4_TCO * _dT)  / pow(2,6));
      _SENS = ((float)_c1_SENSt1 * pow(2,16)) + (((float)_c3_TCS * _dT)  / pow(2,7));
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = 3 * pow(_dT,2) / pow(2,31);
        off2 = 61 * pow((_TEMP - 2000.0),2) / 16;
        sens2 = 2 * pow((_TEMP - 2000.0),2);
        if ( _TEMP < 1500.0 ) { // below 15C
          off2  += 20 * pow((_TEMP + 1500.0),2);
          sens2 += 12 * pow((_TEMP + 1500.0),2);
        }
      }
      else if ( _TEMP > 4500.0 ) sens2 -= pow((_TEMP - 4500.0),2)  /8;
      break;
    case (5):  //MS5803-05-----------------------------------------------------------
      _OFF  = ((float)_c2_OFFt1  * pow(2,18));
      _OFF  += ((float)_c4_TCO * _dT)  / pow(2,5);
      _SENS = ((float)_c1_SENSt1 * pow(2,17)) + (((float)_c3_TCS * _dT)  / pow(2,7));
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = 3 * pow(_dT,2) / pow(2,33);
        off2 = 3 * pow((_TEMP - 2000.0),2) / 8;
        sens2 = 7 * pow((_TEMP - 2000.0),2) / 8;
        if ( _TEMP < 1500.0 ) sens2 += 3 * pow((_TEMP + 1500.0),2);// below 15C
      }
      break;
    case (14):  //MS5803-14-----------------------------------------------------------
    _OFF  = ((float)_c2_OFFt1  * pow(2,16));
    _OFF  += ((float)_c4_TCO * _dT) / pow(2,7);
      _SENS = ((float)_c1_SENSt1 * pow(2,15)) + (((float)_c3_TCS * _dT) / pow(2,8));
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = 3 * pow(_dT,2) / pow(2,33);
        off2 = 3 * pow((_TEMP - 2000.0),2) / 2;
        sens2 = 5 * pow((_TEMP - 2000.0),2) / 8;
        if ( _TEMP < 1500.0 ) {// below 15C
          off2  += 7 * pow((_TEMP + 1500.0),2);
          sens2 += 4 * pow((_TEMP + 1500.0),2);
        }
      }
      else {
        T2 = pow(_dT,2) * 7 / pow(2,37);
        off2 = 1 * pow((_TEMP - 2000.0),2) / 16;
        sens2 = 0;
      }
      break;
    case (30):  //MS5803-30-----------------------------------------------------------
      _OFF  = ((float)_c2_OFFt1  * pow(2,16)) + (((float)_c4_TCO * _dT) / pow(2,7));
      _SENS = ((float)_c1_SENSt1 * pow(2,15)) + (((float)_c3_TCS * _dT) / pow(2,8));
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = 3 * pow(_dT,2) / pow(2,33);
        off2 = 3 * pow((_TEMP - 2000.0),2) / 2;
        sens2 = 5 * pow((_TEMP - 2000.0),2) / 8;
        if ( _TEMP < 1500.0 ) {// below 15C
          off2  += 7 * pow((_TEMP + 1500.0),2);
          sens2 += 4 * pow((_TEMP + 1500.0),2);
        }
      }
      else {
        T2 = pow(_dT,2) * 7 / pow(2,37);
        off2 = 1 * pow((_TEMP - 2000.0),2) / 16;
        sens2 = 0;
      }
      break;
    default: break;
  }
  temp_C = (float)_TEMP / 100.0f; // First order
  if ( _debug ) {
    Serial.print("First order values:");
    Serial.print("     _TEMP = ");                 Serial.println(_TEMP);
    Serial.print("     Temperature (C): ");        Serial.println(temp_C);
    Serial.print("     pressure offset (_OFF): "); Serial.println(_OFF);
    Serial.print("     sensitivity (_SENS): ");    Serial.println(_SENS);
    Serial.println("Second Order Offsets:");
    Serial.print("     T2 = ");                    Serial.println(T2);
    Serial.print("     off2 = ");                  Serial.println(off2);
    Serial.print("     sens2 = ");                 Serial.println(sens2);
  }
  temp_C = temp_C - ( T2 / 100); // Second Order
  _SENS -= sens2;
  _OFF -= off2;
  if ( _debug ) {
    Serial.println("Second Order Values:");
    Serial.print("     temp_C = "); Serial.println(temp_C);
    Serial.print("     sensitivity (_SENS): ");   Serial.println(_SENS);
    Serial.print("     pressure offset (_OFF): ");  Serial.println(_OFF);
  }
  _debug = false; // Always resets
  return _TEMP;
}

    
float MS5803::getPressure(bool debug){
  _debug = debug;
  return getPressure();
}
float MS5803::getPressure(){
  char buf[14];
  // Request which buffer and oversampling ratio...
  I2Cdev::writeBytes(_dev_address, MS5803_D1_512,0,buffer);
  // Get buffer & convert
  I2Cdev::readBytes(_dev_address,MS5803_ADC_READ,3,buffer,2000);
  _d1_pressure = ((long)buffer[0] << 16) + ((long)buffer[1] << 8) + (long)buffer[2];
  dtostrf(_d1_pressure,13,0,buf);
  Serial.print("_d1_pressure = "); Serial.println(buf);
  _P = ((float)_d1_pressure * _SENS) / pow(2,21);
  _P = _P - _OFF;
  _P = _P / pow(2,15);
  press_mBar = (float)_P / 10.0;
  press_kPa = press_mBar / 10.0;
  press_gauge = press_mBar - press_atmospheric;
  press_psi = press_gauge * 14.50377;
  depth_fresh = (press_mBar/100) * 1.019716;
  if (_debug ) {
	  Serial.print("_P = "); Serial.println(_P);
    Serial.print("press_mBar = "); Serial.println(press_mBar);
    Serial.print("press_gauge = "); Serial.println(press_gauge);
    Serial.print("press_psi = "); Serial.println(press_psi);
    Serial.print("depth_fresh = "); Serial.println(depth_fresh);
  }
  return press_mBar;
}
void MS5803::debugTemperature(){
  getTemperature(true); // Set debug flag
}
