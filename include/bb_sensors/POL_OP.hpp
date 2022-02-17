#ifndef POL_OP_H
#define POL_OP_H

//A2D header
#include "bb_sensors/ADS122C04_ADC_PI.hpp"

#include <cstdint>
#include <string>

#define N_A2Ds 2

class POL_OP
{
private:
  SFE_ADS122C04 A2Ds[2];

  // Placeholder
  uint32_t read_raw_data(SFE_ADS122C04 A2D);

public:
  // Do all i2c initialisation here.
  POL_OP(std::string i2c_bus = "/dev/i2c-1",
         uint8_t A2D1_addr = 0x40,
         uint8_t A2D2_addr = 0x41,
         bool debug = false);

  bool read_sensor(uint32_t* readings);
  bool read_A2D(uint8_t idx, uint32_t* readings);
};

#endif
