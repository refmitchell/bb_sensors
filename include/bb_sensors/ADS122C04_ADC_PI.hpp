/**
   \file ADS122C04_ADC_PI.hpp
   \brief Declaration of the SFE_ADS122C04 class (and associated macro and type
          definitions) for interacting with the  ADS112C04 hardware.

  This file is based on a library written for the TI ADS122C04
  24-Bit 4-Channel 2-kSPS Delta-Sigma ADC With I2C Interface.
  The implementation (ADS122C04_ADC_PI.cpp) has been modified for
  the 16-bit cousin of the ADS122C04 (namely, the ADS112C04).

  If you need to understand this code, then I recommed going through the
  datasheet linked below in parallel.

  Original library written by: Paul Clark (PaulZC)
  Date: May 4th 2020

  Modified by: Robert Mitchell (refmitchell)
  Date: 15/02/2022

  Reason for modification:
  Native operation on RPi using Linux i2c headers (as opposed to Arduino exclusive
  Wire.h) and adaptation for the 16-bit ADS112C04 platform. Modifications to this
  header are minimal.

  Based on: https://github.com/sparkfun/SparkFun_ADS122C04_ADC_Arduino_Library
  (written for the 24-bit ADS122C04 for use with an Arduino).

  Also see the TI datasheet:
  https://www.ti.com/product/ADS112C04
  https://www.ti.com/lit/ds/symlink/ads112c04.pdf

  Software made available under the MIT license (included in the source code).
*/

/*
  The MIT License (MIT)

  Copyright (c) 2020 SparkFun Electronics

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef ADS122C04_PI_LIBRARY_H
#define ADS122C04_PI_LIBRARY_H

#include <cstdint>
#include <string>
#include <chrono>


// Single Conversion Timeout (millis)
// The maximum time we will wait for DRDY to go valid for a single conversion
#define ADS122C04_CONVERSION_TIMEOUT 75

// Define 2/3/4-Wire, Temperature and Raw modes
#define ADS122C04_4WIRE_MODE         0x0
#define ADS122C04_3WIRE_MODE         0x1
#define ADS122C04_2WIRE_MODE         0x2
#define ADS122C04_TEMPERATURE_MODE   0x3
#define ADS122C04_RAW_MODE           0x4
#define ADS122C04_4WIRE_HI_TEMP      0x5
#define ADS122C04_3WIRE_HI_TEMP      0x6
#define ADS122C04_2WIRE_HI_TEMP      0x7
#define ADS122C04_POL_OP_MODE        0x8 // Custom mode to configure for POL_OP usage

// ADS122C04 Table 16 in Datasheet
#define ADS122C04_RESET_CMD          0x06     //0000 011x      Reset
#define ADS122C04_START_CMD          0x08     //0000 100x      Start/Sync
#define ADS122C04_POWERDOWN_CMD      0x02     //0000 001x      PowerDown
#define ADS122C04_RDATA_CMD          0x10     //0001 xxxx      RDATA
#define ADS122C04_RREG_CMD           0x20     //0010 rrxx      Read REG rr= register address 00 to 11
#define ADS122C04_WREG_CMD           0x40     //0100 rrxx      Write REG rr= register address 00 to 11

#define ADS122C04_WRITE_CMD(reg)     (ADS122C04_WREG_CMD | (reg << 2))    //Shift is 2-bit in ADS122C04
#define ADS122C04_READ_CMD(reg)      (ADS122C04_RREG_CMD | (reg << 2))    //Shift is 2-bit in ADS122C04

// ADS122C04 Table 16 in Datasheet
#define ADS122C04_CONFIG_0_REG      0 // Configuration Register 0
#define ADS122C04_CONFIG_1_REG      1 // Configuration Register 1
#define ADS122C04_CONFIG_2_REG      2 // Configuration Register 2
#define ADS122C04_CONFIG_3_REG      3 // Configuration Register 3

// Unshifted register definitions
// The bit field register definitions will do the bit shifting

// Configuration Register 0
// ADS122C04 Table 19 in Datasheet

// Input Multiplexer Configuration
#define ADS122C04_MUX_AIN0_AIN1     0x0
#define ADS122C04_MUX_AIN0_AIN2     0x1
#define ADS122C04_MUX_AIN0_AIN3     0x2
#define ADS122C04_MUX_AIN1_AIN0     0x3
#define ADS122C04_MUX_AIN1_AIN2     0x4
#define ADS122C04_MUX_AIN1_AIN3     0x5
#define ADS122C04_MUX_AIN2_AIN3     0x6
#define ADS122C04_MUX_AIN3_AIN2     0x7
#define ADS122C04_MUX_AIN0_AVSS     0x8
#define ADS122C04_MUX_AIN1_AVSS     0x9
#define ADS122C04_MUX_AIN2_AVSS     0xa
#define ADS122C04_MUX_AIN3_AVSS     0xb
#define ADS122C04_MUX_REFPmREFN     0xc
#define ADS122C04_MUX_AVDDmAVSS     0xd
#define ADS122C04_MUX_SHORTED       0xe

// Gain Configuration
#define ADS122C04_GAIN_1            0x0
#define ADS122C04_GAIN_2            0x1
#define ADS122C04_GAIN_4            0x2
#define ADS122C04_GAIN_8            0x3
#define ADS122C04_GAIN_16           0x4
#define ADS122C04_GAIN_32           0x5
#define ADS122C04_GAIN_64           0x6
#define ADS122C04_GAIN_128          0x7

// PGA Bypass (PGA is disabled when the PGA_BYPASS bit is set)
#define ADS122C04_PGA_DISABLED      0x1
#define ADS122C04_PGA_ENABLED       0x0

// Configuration Register 1
// ADS122C04 Table 19 in Datasheet

// Data Rate
// Turbo mode = Normal mode * 2 (Samples per Second)
// Normal mode
#define ADS122C04_DATA_RATE_20SPS   0x0
#define ADS122C04_DATA_RATE_45SPS   0x1
#define ADS122C04_DATA_RATE_90SPS   0x2
#define ADS122C04_DATA_RATE_175SPS  0x3
#define ADS122C04_DATA_RATE_330SPS  0x4
#define ADS122C04_DATA_RATE_600SPS  0x5
#define ADS122C04_DATA_RATE_1000SPS 0x6

// Operating Mode
#define ADS122C04_OP_MODE_NORMAL    0x0
#define ADS122C04_OP_MODE_TURBO     0x1

// Conversion Mode
#define ADS122C04_CONVERSION_MODE_SINGLE_SHOT   0x0
#define ADS122C04_CONVERSION_MODE_CONTINUOUS    0x1

// Voltage Reference Selection
#define ADS122C04_VREF_INTERNAL            0x0 //2.048V internal
#define ADS122C04_VREF_EXT_REF_PINS        0x1 //REFp and REFn external
#define ADS122C04_VREF_AVDD                0x2 //Analog Supply AVDD and AVSS

// Temperature Sensor Mode
#define ADS122C04_TEMP_SENSOR_OFF          0x0
#define ADS122C04_TEMP_SENSOR_ON           0x1

// Configuration Register 2
// ADS122C04 Table 22 in Datasheet

// Conversion Result Ready Flag (READ ONLY)

// Data Counter Enable
#define ADS122C04_DCNT_DISABLE             0x0
#define ADS122C04_DCNT_ENABLE              0x1

// Data Integrity Check Enable
#define ADS122C04_CRC_DISABLED             0x0
#define ADS122C04_CRC_INVERTED             0x1
#define ADS122C04_CRC_CRC16_ENABLED        0x2

// Burn-Out Current Source
#define ADS122C04_BURN_OUT_CURRENT_OFF     0x0
#define ADS122C04_BURN_OUT_CURRENT_ON      0x1

// IDAC Current Setting
#define ADS122C04_IDAC_CURRENT_OFF         0x0
#define ADS122C04_IDAC_CURRENT_10_UA       0x1
#define ADS122C04_IDAC_CURRENT_50_UA       0x2
#define ADS122C04_IDAC_CURRENT_100_UA      0x3
#define ADS122C04_IDAC_CURRENT_250_UA      0x4
#define ADS122C04_IDAC_CURRENT_500_UA      0x5
#define ADS122C04_IDAC_CURRENT_1000_UA     0x6
#define ADS122C04_IDAC_CURRENT_1500_UA     0x7

// Configuration Register 3
// ADS122C04 Table 23 in Datasheet

// IDAC1 Routing Configuration
#define ADS122C04_IDAC1_DISABLED           0x0
#define ADS122C04_IDAC1_AIN0               0x1
#define ADS122C04_IDAC1_AIN1               0x2
#define ADS122C04_IDAC1_AIN2               0x3
#define ADS122C04_IDAC1_AIN3               0x4
#define ADS122C04_IDAC1_REFP               0x5
#define ADS122C04_IDAC1_REFN               0x6

// IDAC2 Routing Configuration
#define ADS122C04_IDAC2_DISABLED           0x0
#define ADS122C04_IDAC2_AIN0               0x1
#define ADS122C04_IDAC2_AIN1               0x2
#define ADS122C04_IDAC2_AIN2               0x3
#define ADS122C04_IDAC2_AIN3               0x4
#define ADS122C04_IDAC2_REFP               0x5
#define ADS122C04_IDAC2_REFN               0x6

// Bit field type register configuration
// Configuration Map register ADS122C04
//--------------Address 0x00---------------------------------
struct CONFIG_REG_0{
  uint8_t PGA_BYPASS:1;                           // 0
  uint8_t GAIN:3;                                 // 1-3
  uint8_t MUX:4;                                  // 4-7
};
union CONFIG_REG_0_U {
  uint8_t all;
  struct CONFIG_REG_0 bit;
};

//--------------Address 0x01---------------------------------
struct CONFIG_REG_1{
  uint8_t TS:1;                                   // 0
  uint8_t VREF:2;                                 // 1-2
  uint8_t CMBIT:1;                                // 3
  uint8_t MODE:1;                                 // 4
  uint8_t DR:3;                                   // 5-7
};

union CONFIG_REG_1_U {
  uint8_t all;
  struct CONFIG_REG_1 bit;
};

//--------------Address 0x02---------------------------------
struct CONFIG_REG_2{
  uint8_t IDAC:3;                                 // 0-2
  uint8_t BCS:1;                                  // 3
  uint8_t CRC:2;                                  // 4-5
  uint8_t DCNT:1;                                 // 6
  uint8_t DRDY:1;                                 // 7
};

union CONFIG_REG_2_U {
  uint8_t all;
  struct CONFIG_REG_2 bit;
};

//--------------Address 0x03---------------------------------
struct CONFIG_REG_3{
  uint8_t RESERVED:2;                             // 0-1
  uint8_t I2MUX:3;                                // 2-4
  uint8_t I1MUX:3;                                // 5-7
};

union CONFIG_REG_3_U {
  uint8_t all;
  struct CONFIG_REG_3 bit;
};

// All four registers
typedef struct ADS122C04Reg{
  union CONFIG_REG_0_U reg0;
  union CONFIG_REG_1_U reg1;
  union CONFIG_REG_2_U reg2;
  union CONFIG_REG_3_U reg3;
} ADS122C04Reg_t;

// Union for the 14-bit internal Temperature
// To simplify converting from uint16_t to int16_t
// without using a cast
union internal_temperature_union{
  int16_t INT16;
  uint16_t UINT16;
};

// Union for the 24-bit raw voltage
// To simplify converting from uint32_t to int32_t
// without using a cast
union raw_voltage_union{
  int32_t INT32;
  uint32_t UINT32;
};

// struct to hold the initialisation parameters
typedef struct{
  uint8_t inputMux;
  uint8_t gainLevel;
  uint8_t pgaBypass;
  uint8_t dataRate;
  uint8_t opMode;
  uint8_t convMode;
  uint8_t selectVref;
  uint8_t tempSensorEn;
  uint8_t dataCounterEn;
  uint8_t dataCRCen;
  uint8_t burnOutEn;
  uint8_t idacCurrent;
  uint8_t routeIDAC1;
  uint8_t routeIDAC2;
} ADS122C04_initParam;

/** Replicates functionality of Arduino millis() function. */
inline unsigned long _millis(){
  auto time = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count();
}

class SFE_ADS122C04
{
public:
  SFE_ADS122C04(void);

  //By default use the default I2C address, and use Wire port
  //Returns true if module is detected
  bool begin(uint8_t deviceAddress = 0x45, std::string bus = "/dev/i2c-1");

  void enableDebugging(); // enable debug messages
  void disableDebugging(); // disable debug messages

  float readPT100Centigrade(void); // Read the PT100 temperature in Centigrade
  float readPT100Fahrenheit(void); // Read the PT100 temperature in Fahrenheit

  // Read the raw signed 24-bit ADC value as int32_t
  // This uses the internal 2.048V reference with the gain set to 1
  // The LSB is 2.048 / 2^23 = 0.24414 uV (0.24414 microvolts)
  int32_t readRawVoltage(uint8_t rate = ADS122C04_DATA_RATE_20SPS);

  // Read the raw signed 24-bit ADC value as uint32_t
  // The ADC data is returned in the least-significant 24-bits
  uint32_t readADC(void);

  // Read the internal temperature (C)
  float readInternalTemperature(uint8_t rate = ADS122C04_DATA_RATE_20SPS);

  bool reset(void); // Reset the ADS122C04
  bool start(void); // Start a conversion
  bool powerdown(void); // Put the chip into low power mode

  // Default to using 'safe' settings (disable the IDAC current sources)
  // Configure the chip for the chosen mode
  bool configureADCmode(uint8_t wire_mode = ADS122C04_RAW_MODE,
                        uint8_t rate = ADS122C04_DATA_RATE_20SPS);

  // Default to using 'safe' settings (disable the IDAC current sources)

  // Configure the input multiplexer
  bool setInputMultiplexer(uint8_t mux_config = ADS122C04_MUX_AIN1_AIN0);

  // Configure the gain
  bool setGain(uint8_t gain_config = ADS122C04_GAIN_1);

  // Enable/disable the Programmable Gain Amplifier
  bool enablePGA(uint8_t enable = ADS122C04_PGA_DISABLED);

  // Set the data rate (sample speed)
  bool setDataRate(uint8_t rate = ADS122C04_DATA_RATE_20SPS);
  bool setOperatingMode(uint8_t mode = ADS122C04_OP_MODE_NORMAL); // Configure the operating mode (normal / turbo)
  bool setConversionMode(uint8_t mode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT); // Configure the conversion mode (single-shot / continuous)
  bool setVoltageReference(uint8_t ref = ADS122C04_VREF_INTERNAL); // Configure the voltage reference
  bool enableInternalTempSensor(uint8_t enable = ADS122C04_TEMP_SENSOR_OFF); // Enable / disable the internal temperature sensor
  bool setDataCounter(uint8_t enable = ADS122C04_DCNT_DISABLE); // Enable / disable the conversion data counter
  bool setDataIntegrityCheck(uint8_t setting = ADS122C04_CRC_DISABLED); // Configure the data integrity check
  bool setBurnOutCurrent(uint8_t enable = ADS122C04_BURN_OUT_CURRENT_OFF); // Enable / disable the 10uA burn-out current source
  bool setIDACcurrent(uint8_t current = ADS122C04_IDAC_CURRENT_OFF); // Configure the internal programmable current sources
  bool setIDAC1mux(uint8_t setting = ADS122C04_IDAC1_DISABLED); // Configure the IDAC1 routing
  bool setIDAC2mux(uint8_t setting = ADS122C04_IDAC2_DISABLED); // Configure the IDAC2 routing

  bool checkDataReady(void); // Check the status of the DRDY bit in Config Register 2

  uint8_t getInputMultiplexer(void); // Get the input multiplexer configuration
  uint8_t getGain(void); // Get the gain setting
  uint8_t getPGAstatus(void); // Get the Programmable Gain Amplifier status
  uint8_t getDataRate(void); // Get the data rate (sample speed)
  uint8_t getOperatingMode(void); // Get the operating mode (normal / turbo)
  uint8_t getConversionMode(void); // Get the conversion mode (single-shot / continuous)
  uint8_t getVoltageReference(void); // Get the voltage reference configuration
  uint8_t getInternalTempSensorStatus(void); // Get the internal temperature sensor status
  uint8_t getDataCounter(void); // Get the data counter status
  uint8_t getDataIntegrityCheck(void); // Get the data integrity check configuration
  uint8_t getBurnOutCurrent(void); // Get the burn-out current status
  uint8_t getIDACcurrent(void); // Get the IDAC setting
  uint8_t getIDAC1mux(void); // Get the IDAC1 mux configuration
  uint8_t getIDAC2mux(void); // Get the IDAC2 mux configuration

  // Print the ADS122C04 configuration (but only if enableDebugging has been called)
  void printADS122C04config(void);

  // Requested in #5
  uint8_t getDeviceAddress(void) { return (_deviceAddress); }
  uint8_t getWireMode(void) { return (_wireMode); }

  // Manual config overrides
  // Usage:
  // - Initialise using generic initialisation
  // - Get initParam struct pointer
  // - Update desired options
  // - Reinitialise device (using updated initParam).
  ADS122C04_initParam* getCurrentInitParams(void);
  bool reinitialise(void);

private:
  //Variables
  int _i2c_fd;		//The generic connection to user's chosen I2C hardware
  uint8_t _deviceAddress; //Keeps track of I2C address. setI2CAddress changes this.

  bool _printDebug = false; //Flag to print debugging variables

  // Keep a copy of the wire mode so we can restore it after reading the internal temperature
  uint8_t _wireMode = ADS122C04_4WIRE_MODE;

  // Resistance of the reference resistor
  const float PT100_REFERENCE_RESISTOR = 1620.0;

  // Amplifier gain setting
  // ** MAKE SURE THE CONFIG REGISTER 0 GAIN IS THE SAME AS THIS **
  const float PT100_AMPLIFIER_GAIN = 8.0;
  const float PT100_AMP_GAIN_HI_TEMP = 4.0;

  // Internal temperature sensor resolution
  // One 14-bit LSB equals 0.03125°C
  const float TEMPERATURE_SENSOR_RESOLUTION = 0.03125;

  ADS122C04_initParam lastInitParams; // Retain the last initParam struct.
  ADS122C04Reg_t ADS122C04_Reg; // Global to hold copies of all four configuration registers

  void debugPrint(char *message); // print a debug message
  void debugPrintln(char *message); // print a debug message with line feed

  bool ADS122C04_init(ADS122C04_initParam *param); // initialise the ADS122C04 with these parameters

  bool ADS122C04_writeReg(uint8_t reg, uint8_t writeValue); // write a value to the selected register
  bool ADS122C04_readReg(uint8_t reg, uint8_t *readValue); // read a value from the selected register (returned in readValue)

  bool ADS122C04_getConversionData(uint32_t *conversionData); // read the raw 24-bit conversion result
  bool ADS122C04_getConversionDataWithCount(uint32_t *conversionData, uint8_t *count); // read the raw conversion result and count (if enabled)

  bool ADS122C04_sendCommand(uint8_t command); // write to the selected command register
  bool ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value); // write a value to the selected command register
};
#endif
