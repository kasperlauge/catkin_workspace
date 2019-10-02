#include <sstream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
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

  // auto imageSubscriber = nodeHandle.subscribe<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", 1, &process_image_data);
  auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", nodeHandle);
  auto p_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/iris_sensors_0/camera_red_iris/depth/points");


  ros::spin();

  return 0;
}