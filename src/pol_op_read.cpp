/**
   \file pol_op_read.cpp
   \brief ROS node to read from a single polarsiation opponent unit.

   This was a test node which was used to test basic I2C interaction with
   the polarisation opponent units constructed for Gkanias et al. (2023).

   It does not interact with any other nodes on the network and simply prints
   the output from the photodiodes on the device. The code assumes the device
   is wired directly onto the I2C bus on the Raspberry Pi without any
   multiplexing.

   \note Each polarisation opponent unit is uses two I2C analogue to digital
   converters (ADCs). The four photodiodes are split across the two ADCs,
   meaning we need to read from both to read from all four photodiodes.
   Hence, two I2C addresses for one unit.

   \author Robert Mitchell
*/

#include <iostream>
#include <sstream>

#include "bb_sensors/POL_OP.hpp"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define LOG(x) std::cout << x << std::endl

#define A2D_ADDR_1 0x40 /**< I2C address of the 1st ADC on the PCB. */
#define A2D_ADDR_2 0x41 /**< I2C address of the 2nd ADC on the PCB. */

int main(int argc, char **argv){
  /* Initialise the ROS node */
  ros::init(argc, argv, "pol_op_test");
  ros::NodeHandle n;

  /* Initialise polarisation opponent device */
  std::string bus = "/dev/i2c-1";
  POL_OP po(bus, A2D_ADDR_1, A2D_ADDR_2, false);

  /* Read from the photodiodes and print the output at the terminal. */
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    int readings[4];
    bool succcess = po.read_sensor(readings);

    std::stringstream outstr;
    outstr <<
      "Readings: \n";
    for (int i = 0; i < 4; i++){
      outstr << "R" << i+1 << ": " << (int) readings[i] << "\n";
    }
    ROS_INFO(outstr.str().c_str());

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
