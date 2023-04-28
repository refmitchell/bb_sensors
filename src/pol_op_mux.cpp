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

#include "bb_sensors/ADS122C04_ADC_PI.hpp" // initParams
#include "bb_sensors/POL_OP.hpp" // POL_OP class wrapper
#include "bb_sensors/argparse.h" // ArgumentParser

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#define LOG(x) std::cout << x << std::endl

#define MUX_ADDR 0x70
#define A2D_ADDR_1 0x40
#define A2D_ADDR_2 0x41

argparse::ArgumentParser parser("Parser");
bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"--gain"})
    .description("Set the PGA gain for the ADCs (3-bit config, see datasheet for"
                 " ADS122C04). Valid values 0-7, gain will be 2^g where g is the"
                 " argument provided here.")
    .required(false);

  parser.add_argument()
    .names({"--rate"})
    .description("Set the conversion rate for the ADCs. Valid values 0-6."
                 " 0 = 20SPS, each increment approximately doubles the"
                 " data rate up to 6 = 1000SPS.")
    .required(false);

  parser.add_argument()
    .names({"-n","--number_of_units"})
    .description("The number of POL-OP units connected to the mux (1 - 8).")
    .required(false);

  parser.enable_help();

  auto err = parser.parse(argc, const_cast<const char**>(argv));
  if (err) {
    std::stringstream ss;
    ss<<err;
    ROS_ERROR("argparse error: %s", ss.str().c_str());
    return false;
  }

  return true;
}

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

int main(int argc, char **argv){
  if(!initParser(parser, argc, argv)) return -1;
  if(parser.exists("help")){
    parser.print_help();
    return 0;
  }

  int pga_gain =
    parser.exists("gain") ?
    parser.get<int>("gain") :
    0;

  int data_rate =
    parser.exists("rate") ?
    parser.get<int>("rate") :
    0;

  int n_pol_ops =
    parser.exists("number_of_units") ?
    parser.get<int>("number_of_units") :
    1;

  ROS_INFO("Gain setting: %d", pga_gain);
  ROS_INFO("Number of POL-OPs: %d", n_pol_ops);

  ros::init(argc, argv, "i2c_mux_pol_op");
  ros::NodeHandle n;

  //
  // Setup - Initialise any I2C devices in use here.
  //
  std::string bus = "/dev/i2c-1";
  int fd = init_mux(bus);

  std::vector<ros::Publisher> pubs;
  std::vector<POL_OP> pol_ops;

  // Init each pol_op unit and its publisher
  for (int i = 0; i < n_pol_ops; i++){
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


  // Set custom options for each pol_op unit
  for (int i = 0; i < n_pol_ops; i++){
    pol_ops[i].set_gain(pga_gain);
    pol_ops[i].set_data_rate(data_rate);
  }

  // Broadcast outputs from connected encoders
  ros::Rate loop_rate(10);

  auto start = std::chrono::system_clock::now();
  while(ros::ok()) {
    int s_readings = 4;
    int readings[s_readings];
    auto full_read_start = std::chrono::system_clock::now();
    for (int i = 0; i < n_pol_ops; i++){

      for (int j = 0; j < s_readings; j++) readings[j] = 0; // Clear readings
      // Read from pol_op
      select_channel(i, fd);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      // Min working delay = 25; 11ms was used for pol experiments
      // before this was known.
      bool success = pol_ops[i].read_sensor_interleaved(readings, 25);

      std::vector<int> res(readings, readings+s_readings);

      // Publish photodiode data on the correct topic.
      std_msgs::Int32MultiArray msg;
      msg.data = res;
      auto time = std::chrono::system_clock::now() - start;
      auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time);
      std::cout << "Time since last publication: " << millis.count() << "ms" << std::endl;
      pubs[i].publish(msg);
      start = std::chrono::system_clock::now();
      //ROS_INFO("Unit %d: {%d, %d, %d, %d}", i, res[0], res[1], res[2], res[3]);
    }


    auto full_read_time = std::chrono::system_clock::now() - full_read_start;
    auto full_read_millis = std::chrono::duration_cast<std::chrono::milliseconds>(full_read_time);
    std::cout << "Full read time: " << full_read_millis.count() << "ms" << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }
  LOG("Exiting.");
  return 0;
}
