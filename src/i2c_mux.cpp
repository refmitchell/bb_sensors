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

#include "bb_sensors/AMS_5600_PI.hpp"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define LOG(x) std::cout << x << std::endl

#define MUX_ADDR 0x70
#define ENC_ONE_CHANNEL 0
#define ENC_TWO_CHANNEL 1


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
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

// Return 0-360 angle from a given encoder
double getAngle(AMS_5600& encoder, const int mux){
  select_channel(encoder.getChannel(), mux);
  return encoder.getRawAngle() * 0.087;
}

int getSpeed(AMS_5600& encoder, const int mux){
  select_channel(encoder.getChannel(), mux);
  return encoder.getSpeed();
}


// True on magnet detected, false otherwise for given encoder
bool detectMagnet(AMS_5600& encoder, const int mux){
  select_channel(encoder.getChannel(), mux);
  return encoder.detectMagnet();
}

// Init a given encoder
void encoderInit(AMS_5600& encoder, const std::string bus, const int mux){
  select_channel(encoder.getChannel(), mux);
  return encoder.init(bus);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "i2c_mux");
  ros::NodeHandle n;

  ros::Publisher wind_speed_pub = n.advertise<std_msgs::Float64>("wind_speed", 1000);
  ros::Publisher wind_dir_pub = n.advertise<std_msgs::Float64>("wind_direction", 1000);
  //
  // Setup - Initialise any I2C devices in use here.
  //
  AMS_5600 e1(ENC_ONE_CHANNEL); // Wired to channel 0
  AMS_5600 e2(ENC_TWO_CHANNEL); // Wired to channel 1

  std::string bus = "/dev/i2c-1";

  int mux_fd = init_mux(bus);
  encoderInit(e1, bus, mux_fd);
  encoderInit(e2, bus, mux_fd);

  // Broadcast outputs from connected encoders
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    double e1_out = detectMagnet(e1, mux_fd) ? getAngle(e1, mux_fd) : -1;
    double e2_out = detectMagnet(e2, mux_fd) ? getSpeed(e2, mux_fd) : -1;

    std::stringstream outstr;
    outstr << "E1: " << e1_out <<  ", E2: " << e2_out << std::endl;

    std_msgs::Float64 speed_msg;
    std_msgs::Float64 direction_msg;

    speed_msg.data = e2_out;
    direction_msg.data = e1_out;

    wind_speed_pub.publish(speed_msg);
    wind_dir_pub.publish(direction_msg);

    //    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
