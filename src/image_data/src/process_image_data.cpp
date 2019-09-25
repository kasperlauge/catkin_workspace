#include <sstream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// To build changes from this service run catkin build in ~/catkin_ws

// Start the multi UAV PX4 program simulation
// Call this service using "rosrun test_ros arm_uav" in command line

void process_image_data(const sensor_msgs::Image::ConstPtr& imageMsg) {
    ROS_INFO_STREAM(imageMsg->width);
    ROS_INFO_STREAM(imageMsg->height);
};

int main(int argc, char **argv)
{

  const int numberOfUAVs = 2;

  ros::init(argc, argv, "process_image_data");

  ros::NodeHandle nodeHandle;

  auto imageSubscriber = nodeHandle.subscribe<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", 1, &process_image_data);


  ros::spin();

  return 0;
}