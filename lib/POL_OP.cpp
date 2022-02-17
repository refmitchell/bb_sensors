#include "bb_sensors/POL_OP.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#define LOG(x) std::cout << x << std::endl;

POL_OP::POL_OP(std::string i2c_bus,
               uint8_t A2D1_addr,
               uint8_t A2D2_addr,
               bool debug){
  if (!(A2Ds[0].begin(A2D1_addr, i2c_bus))){
    LOG("A2D1, error initialisation. Exiting.");
    exit(1);
  }

  if (!(A2Ds[1].begin(A2D2_addr, i2c_bus))){
    LOG("A2D2, error initialisation. Exiting.");
    exit(1);
  }

  if (debug){
    for (int i = 0; i < 2; i++){ A2Ds[i].enableDebugging(); }
  }else{
    for (int i = 0; i < 2; i++){ A2Ds[i].disableDebugging(); }
  }
}

uint32_t POL_OP::read_raw_data(SFE_ADS122C04 A2D){
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  A2D.start(); // Start conversion

  // Record the start time so we can timeout
  // _millis() provided by A2D header, should fix this (bb_util)
  unsigned long start_time = _millis();
  bool drdy = false;

  while ((drdy == false) &&
         (_millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT))
         ){

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    drdy = A2D.checkDataReady();
  }

  if (drdy == false) {
    LOG("DRDY timed out.");
    return 0; // Error codes?
  }

  // Read raw data and covert to volts (method 1).
  int32_t raw_ADC_data = A2D.readADC();
  return raw_ADC_data;
  //  float volts_1 = ((float)raw_ADC_data) * 244.14e-9;
}

bool POL_OP::read_A2D(uint8_t idx, uint32_t* readings){
  /*
    Performs one complete read cycle of a specific A2D.
    Results are stored in readings, which should be
    declared by the caller.

    Output array should be large enough for both results.
   */
  if (idx > N_A2Ds - 1) {
    LOG("A2D read err, A2D idx out of range.");
    return false;
  }

  A2Ds[idx].setInputMultiplexer(ADS122C04_MUX_AIN1_AIN0);
  readings[0] = read_raw_data(A2Ds[idx]);

  A2Ds[idx].setInputMultiplexer(ADS122C04_MUX_AIN3_AIN2);
  readings[1] = read_raw_data(A2Ds[idx]);

  return true;
}

bool POL_OP::read_sensor(uint32_t *readings){
  for (int i = 0; i < N_A2Ds; i++){
    uint32_t temp[2];
  }
}
