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

#ifndef _MS5803_H_
#define _MS5803_H_

#define I2CDEV_SERIAL_DEBUG

#include "I2Cdev.h"
#include <avr/pgmspace.h>

#define MS5803_ADDRESS 
#define MS5803_ADDRESS_AD0_LOW     0x77 // address pin low (GND), default for InvenSense evaluation board
#define MS5803_ADDRESS_AD0_HIGH    0x76 // address pin high (VCC)
#define MS5803_DEFAULT_ADDRESS     MS5803_ADDRESS_AD0_LOW

#define MS5803_RESET     0x1E 
#define MS5803_PROM_BASE 0xA0
#define MS5803_PROM_C1   0xA2
#define MS5803_PROM_C2   0xA4
#define MS5803_PROM_C3   0xA6
#define MS5803_PROM_C4   0xA8
#define MS5803_PROM_C5   0xAA
#define MS5803_PROM_C6   0xAC
#define MS5803_PROM_CRC  0xAE
#define MS5803_ADC_READ  0x00

#define MS5803_D1_256   0x40
#define MS5803_D1_512   0x42
#define MS5803_D1_1024  0x44
#define MS5803_D1_2048  0x46
#define MS5803_D1_4096  0x48
#define MS5803_D2_256   0x50
#define MS5803_D2_512   0x52
#define MS5803_D2_1024  0x54
#define MS5803_D2_2048  0x56
#define MS5803_D2_4096  0x58

class MS5803 {
  public:
    MS5803();
    MS5803(uint8_t address);
    
    float temp_C;
    float press_mBar;
    float press_kPa;
    float press_atmospheric;
    float press_gauge;
    float press_psi;
    float depth_fresh;
    void setAddress(uint8_t address);
	uint8_t getAddress() {return _dev_address;}
    void initialize(uint8_t model);
    bool testConnection();
    float getTemperature();
    float getTemperature(bool debug);
    float getPressure();
    float getPressure(bool debug);
    void debugCalConstants();
    int32_t getCalConstant(uint8_t constant_no);
    void debugTemperature();
    void setAtmospheric(float pressure) {press_atmospheric = pressure;}
    uint16_t reset();
    long getD1Pressure() { return _d1_pressure; }
    long getD2Temperature() { return _d2_temperature;}
    
  private:
    // Calibration Constants
    int32_t _c1_SENSt1; // Pressure Sensitivity
    int32_t _c2_OFFt1;  // Pressure Offset
    int32_t _c3_TCS;    // Temperature coefficient of pressure sensitivity
    int32_t _c4_TCO;    // Temperature coefficient of pressure offset
    int32_t _c5_Tref;   // Reference Temperature
    int32_t _c6_TEMPSENS;   // Temperature coefficient of the temperature
    // pressure and temperature data
    long _d1_pressure;    // AdcPressure - long
    long _d2_temperature; // AdcTemperature - long
    // Calculated values
    float _dT; //TempDifference
    float _TEMP;       // Actual temperature -40 to 85C with .01 resolution (divide by 100) - Temperature float
    // Temperature compensated pressure
    float _OFF;       // First Order Offset at actual temperature // Offset - float
    float _SENS;      // Sensitivity at actual temperature // Sensitivity - float
    float _P;         // Temperature compensated pressure 10...1300 mbar (divide by 100 to get mBar)
    uint8_t buffer[14]; // ByteHigh,ByteMiddle,ByteLow - byte
    uint8_t _dev_address;
    boolean _debug;
    uint8_t _model; // the suffix after ms5803. E.g 2 for MS5803-02 indicates range.
    void _getCalConstants();
    

};
#endif