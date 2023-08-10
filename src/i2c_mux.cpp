/**
   \file i2c_mux.cpp
   \brief ROS node which reads from a custom wind sensor.


   This node reads from two AS5600 magnetic encoders operating over
   I2C.  As the encoders use the same I2C address, the code reads from
   an Adafruit multiplexer placed between the RPi board and encoders.
   This code is designed to read directly from the I2C pins on the
   Raspberry Pi 3B+.

   Wind direction is read from mux channel 0, direction is read from
   mux channel 1. These are read using getDirection and getSpeed and
   then broadcast on `/wind_direction` and `/wind_speed` respectively.



   I2C Multiplexer: TCA9548A
   Adafruit pacakge: https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/overview

   I2C magnetic encoder: AS5600
   Seeed studio package: https://wiki.seeedstudio.com/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/

   \note
   - The magnetic encoder is incorrectly called the `AMS_5600` throughout the
     code which is incorrect. It is the AS5600.
   - The node will run if magnets are not in proximity to one or
   both of the encoders. If no magnet is detected, the node will
   broadcast -1 on the relevant topic.
   - Wind speed is absolute, the direction of rotation is not
   important
   - Wind speed is not calibrated and you cannot make any assumptions
   about units.

   \author Robert Mitchell
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

#define MUX_ADDR 0x70 /**< The I2C address for the mux. */
#define ENC_ONE_CHANNEL 0 /**< Encoder 1 mux channel. */
#define ENC_TWO_CHANNEL 1 /**< Encoder 2 mux channel. */


/**
   Initialises the multiplexer on the I2C bus. If the initialisation fails then
   the node will shut down.

   \param bus The string identifier for the I2C bus, see Linux kernel documentation.
   \return A file descriptor which can be used to access the multiplexer.
*/
int init_mux(std::string bus){
  int fd;

  if ((fd = open(bus.c_str(), O_RDWR)) < 0) std::exit(1);

  if (ioctl(fd, I2C_SLAVE, MUX_ADDR) < 0){
    close(fd);
    std::exit(1);
  }

  return fd;
}

/**
   Selects a given channel on the multiplexer and delays to allow time for
   channel selection to take place.

   Will print a message to stdout if the channel selection fails.

   \param channel The channel (0 - 7) to read from.
   \param fd The file descriptor used to communicate with the mux
             on the I2C bus.
   \note This function does not return any value despite its return type.
 */
bool select_channel(const uint8_t channel, const int fd){
  int ret = i2c_smbus_write_byte_data(fd, 0x00, (1 << channel));
  if (ret == -1) {LOG("Failed to select channel");}
  // Allow time for channel switch
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

/**
   Fetch an angle between 0 and 360 degrees from the encoder. Used here to 
   ascertain wind direction.

   \param encoder An object representing the AS5600 encoder we want to read from.
   \param mux The file descriptor used to communicate with the mux on the
              I2C bus.
   \return The current angle between the magnet and the encoder.
*/
double getAngle(AMS_5600& encoder, const int mux){
  select_channel(encoder.getChannel(), mux);
  return encoder.getRawAngle() * 0.087;
}

/**
   Fetch the rotational speed of the magnet over the encoder. Used here to 
   ascertain wind speed.

   \param encoder An object representing the AS5600 encoder we want to read from.
   \param mux The file descriptor used to communicate with the mux on the
              I2C bus.
   \return The current angle between the magnet and the encoder.
*/
int getSpeed(AMS_5600& encoder, const int mux){
  select_channel(encoder.getChannel(), mux);
  return encoder.getSpeed();
}

/**
   Checks whther the encoder is under the influence of a magnetic field (i.e.
   is there a magnet in range?)

   \param encoder An object representing the AS5600 encoder we want to read from.
   \param mux The file descriptor used to communicate with the mux on the
              I2C bus.
   \return true if a magnetic field is detected, false otherwise.
*/
bool detectMagnet(AMS_5600& encoder, const int mux){
  select_channel(encoder.getChannel(), mux);
  return encoder.detectMagnet();
}

/**
   Initialise an AS5600 encoder.

   \param encoder An object representing the AS5600 encoder we want to read from.
   \param bus The string identifier for the I2C bus, see Linux kernel documentation.
   \param mux The file descriptor used to communicate with the mux on the
              I2C bus.
   \return true if a magnetic field is detected, false otherwise.
*/
void encoderInit(AMS_5600& encoder, const std::string bus, const int mux){
  select_channel(encoder.getChannel(), mux);
  return encoder.init(bus);
}

/**
    Main loop.
*/
int main(int argc, char **argv){
  /* Initialise node and publishers */
  ros::init(argc, argv, "i2c_mux");
  ros::NodeHandle n;
  ros::Publisher wind_speed_pub = n.advertise<std_msgs::Float64>("wind_speed", 1000);
  ros::Publisher wind_dir_pub = n.advertise<std_msgs::Float64>("wind_direction", 1000);

  AMS_5600 e1(ENC_ONE_CHANNEL); // Wired to channel 0
  AMS_5600 e2(ENC_TWO_CHANNEL); // Wired to channel 1

  /*
    This string specifies the I2C bus we want to talk to. The string
    specifier is different per system and this one is specific to the
    Raspberry Pi 3B+ used on the beetlebot.
  */
  std::string bus = "/dev/i2c-1";

  /* Initialise the mux and the encoders. */
  int mux_fd = init_mux(bus);
  encoderInit(e1, bus, mux_fd);
  encoderInit(e2, bus, mux_fd);

  /* Broadcast outputs from connected encoders */
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
