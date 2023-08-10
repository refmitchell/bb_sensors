/**
   \file camera.cpp
   \brief ROS node which reads from an RPi camera.

   This node reads a downsampled image from a connected RPi camera
   module and publishes it on the network. OpenCV is used to read from
   the camera; the resultant CV image is converted to a ROS image
   pointer for broadcast.

   In theory, this node should read from any connected camera (USB or
   RPi, so long as there is only one connected) but this has not been
   tested.

   This node takes no arguments and has no runtime configuration
   options.

   \warning The image is captured at low resolution rather than
   performing downsampling later on. This is done for efficiency
   reasons but it means there is no way to recover a full resolution
   image. This can be changed by setting the CAMERA_RES definition and
   re-compiling.
 */

/**
    \brief Debug compile flag.

    Set to 1 to generate a video window with the camera output.
 */
#define DEBUG 0

/** Camera capture resolution (for a square window). */
#define CAMERA_RES 64

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[]){
  // ROS Node init
  ros::init(argc, argv, "raw_pub_node");

  // Instantiate node handle
  ros::NodeHandle n;

  // Set up image transport publisher
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("frames", 1);

  // Start OpenCV video capture
  cv::VideoCapture cap;
  if(!cap.open(0)) {
    ROS_ERROR("OpenCV capture failed to open.");
    return -1;
  }


  // Comment these lines to capture at full resolution
  // (probably only worth it for testing).
  cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_RES);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_RES);


  // Capture convert and publish images
  ROS_INFO("Publishing...");
  while(ros::ok()){

    // Get OpenCV frame
    cv::Mat frame;

    cap >> frame;
    if (frame.empty()){
      ROS_INFO("Camera frame empty, exiting.");
      return 0;
    }

    #if DEBUG
    // If we're in debug mode allow the node to create a
    // video window.
    cv::imshow("Publisher - Video", frame);
    if (cv::waitKey(10) == 27) break;
    #endif

    // Convert to sensor_msgs/Image
    sensor_msgs::ImagePtr img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    // Publish our camera image - want to publish them as frequently as we can.
    pub.publish(img_msg);
  }

  return 0;
}
