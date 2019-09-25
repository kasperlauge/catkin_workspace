#include "uav.h"
#include <sstream>

// To build changes from this service run catkin build in ~/catkin_ws

// Start the multi UAV PX4 program simulation
// Call this service using "rosrun test_ros arm_uav" in command line

int main(int argc, char **argv)
{

  const int numberOfUAVs = 2;

  ros::init(argc, argv, "arm_uav");

  ros::NodeHandle n;

  std::vector<UAV> uavs;
  uavs.reserve(numberOfUAVs);

  for (int i = 0; i < numberOfUAVs; i++) {
    uavs.push_back(UAV(n, i));
  }

  for (int i = 0; i < numberOfUAVs; i++) {
    uavs[i].takeOff(i + 4);
  }


  ros::spin();

  return 0;
}