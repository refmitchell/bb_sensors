/*
  ROS Node to read from the I2C mux and broadcast the results 
  to the rest of the network.
*/

#include <iostream>
#include <sstream>

#include "bb_sensors/POL_OP.hpp"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define LOG(x) std::cout << x << std::endl

#define A2D_ADDR_1 0x40
#define A2D_ADDR_2 0x41

int main(int argc, char **argv){
  ros::init(argc, argv, "pol_op_test");
  ros::NodeHandle n;

  //
  // Setup - Initialise any I2C devices in use here.
  //
  std::string bus = "/dev/i2c-1";
  POL_OP po(bus, A2D_ADDR_1, A2D_ADDR_2, false);

  // Broadcast outputs from connected encoders
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    uint32_t readings[4];
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
