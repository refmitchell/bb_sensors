/*
  ROS Node to read from the I2C mux and broadcast the results 
  to the rest of the network.
*/

extern "C" {
#include <linux/i2c-dev.h>
#include <linux/ioctl.h>
#include <fcntl.h> // open() and O_RDWR
#include <unistd.h> // close()
}

#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>

#include "bb_sensors/TCA9548/AMS_5600_PI.hpp"
#include "bb_sensors/POL_OP.hpp"

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#define LOG(x) std::cout << x << std::endl

#define MUX_ADDR 0x70
#define A2D_ADDR_1 0x40
#define A2D_ADDR_2 0x41

// Parameter server values
std::string adc_conversion_mode;
int pd_read_dly;

int init_mux(std::string bus){
  int fd;

  if ((fd = open(bus.c_str(), O_RDWR)) < 0) std::exit(1);

  if (ioctl(fd, I2C_SLAVE, MUX_ADDR) < 0){
    close(fd);
    std::exit(1);
  }

  return fd;
}

bool select_channel(const uint8_t channel, const int fd){
  int ret = i2c_smbus_write_byte_data(fd, 0x00, (1 << channel));
  if (ret == -1) {LOG("Failed to select channel");}
  // Allow time for channel switch
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

bool param_check(ros::NodeHandle& n){
  return false;
}


void initialise(ros::NodeHandle& n,
                std::vector<POL_OP>& pol_ops,
                int n_pol_ops){
  return;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "i2c_mux_pol_op");
  ros::NodeHandle n;


  //
  // Setup - Initialise any I2C devices in use here.
  //
  std::string bus = "/dev/i2c-1";
  int fd = init_mux(bus);

  int n_pol_ops = 1;

  std::vector<ros::Publisher> pubs;
  std::vector<POL_OP> pol_ops;

  // Init each pol_op unit and its publisher
  for (int i = 0; i < n_pol_ops; i++){
    LOG("Channel: "<< i);
    select_channel(i, fd);
    POL_OP poi(bus, A2D_ADDR_1, A2D_ADDR_2, false, i);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::stringstream name;
    name << "pol_op_" << i;
    ros::Publisher poi_pub =
      n.advertise<std_msgs::Int32MultiArray>(name.str().c_str(), 1000);
    pubs.push_back(poi_pub);
    pol_ops.push_back(poi);
  }

  // Broadcast outputs from connected encoders
  ros::Rate loop_rate(20);

  while(ros::ok()) {
    bool reinit = param_check(n);
    int s_readings = 4;
    int readings[s_readings];
    for (int i = 0; i < n_pol_ops; i++){
      for (int j = 0; j < s_readings; j++) readings[j] = 0; // Clear readings
      // Read from pol_op
      select_channel(i, fd);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      bool success = pol_ops[i].read_sensor(readings, 1);
      std::vector<int> res(readings, readings+s_readings);

      // Publish photodiode data on the correct topic.
      std_msgs::Int32MultiArray msg;
      msg.data = res;
      pubs[i].publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  LOG("Exiting.");
  return 0;
}
