// #include "uav.h"
#include <sstream>

/*
NEW trial

*/
#include "mavros_msgs/State.h"
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/tf.h"
#include "ros/ros.h"
#include <mavros_msgs/CommandBool.h>
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/SetMode.h>

class UAV
{
private:
public:
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  ros::NodeHandle nodeHandle;
  mavros_msgs::State current_state;
  ros::Subscriber stateSubscriber;
  geometry_msgs::PoseStamped pose;

  UAV(ros::NodeHandle n)
  {
    nodeHandle = n;

    // Set Callback for being able to monitor State
    // auto stateTopic = "/uav" + std::to_string(UAV::uavNumber) + "/mavros/state";
    auto stateTopic = "/uav0/mavros/state";

    ROS_INFO(stateTopic);
    this->stateSubscriber = this->nodeHandle.subscribe<mavros_msgs::State>(stateTopic, 1, &UAV::state_cb, this);
  };

  void state_cb(const mavros_msgs::State::ConstPtr &msg)
  {
    this->current_state = *msg;
  }
  void setPose(geometry_msgs::PoseStamped _pose)
  {
    this->pose = _pose;
  }

  void takeOff(const int meters)
  {

    // Listen for the armed state - and takeoff when armed

    // Arm the UAV
    UAV::armUAV();

    ros::Rate rate(10);

    while (ros::ok() && !this->current_state.armed)
    {
      ros::spinOnce();
      rate.sleep();
      ROS_INFO("%d", this->current_state.armed);
    }

    // ROS_INFO("CB currentStateSet %d", this->current_state.armed);

    takeOffUAV(meters);

    // for (size_t i = 0; i < 20; i++)
    // {
    //     ros::spinOnce();
    //     rate2.sleep();
    //     ROS_INFO("%d", this->current_state.armed);
    // }
    // run();

    //WAIT 5 Seconds
    for (size_t i = 0; i < 50; i++)
    {
      ros::spinOnce();
      rate.sleep();
      /* code */
    }
    

    setOffboard();

  }

  void setOffboard()
  {
    ROS_INFO("Offboard attempt");
    ros::Time last_request = ros::Time::now();

    this->arming_client = this->nodeHandle.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    this->set_mode_client = this->nodeHandle.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if (arming_client.call(arm_cmd) &&
            arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
  }

  void takeOffUAV(const int meters)
  {
    auto takeOffService = "/uav0/mavros/cmd/takeoff";
    ros::service::waitForService(takeOffService);

    ros::ServiceClient set_takeoff_client = UAV::nodeHandle.serviceClient<mavros_msgs::CommandTOL>(takeOffService);

    mavros_msgs::CommandTOL set_takeoff_srv;

    auto positionTopic = "/uav0/mavros/global_position/global";
    auto currentPosition = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(positionTopic);

    set_takeoff_srv.request.altitude = currentPosition->altitude - 40 + meters;
    set_takeoff_srv.request.latitude = currentPosition->latitude;
    set_takeoff_srv.request.longitude = currentPosition->longitude;

    if (set_takeoff_client.call(set_takeoff_srv))
    {
      ROS_INFO("Takeoff set");
    }
    else
    {
      ROS_ERROR("Something went wrong in takeoff. Pis!");
    }
  };

  void armUAV()
  {
    auto armingService = "/uav0/mavros/cmd/arming";
    // ROS_INFO_STREAM(armingService);
    ros::service::waitForService(armingService);

    ros::ServiceClient set_arming_client = UAV::nodeHandle.serviceClient<mavros_msgs::CommandBool>(armingService);

    mavros_msgs::CommandBool set_arming_srv;
    set_arming_srv.request.value = 1;

    if (set_arming_client.call(set_arming_srv))
    {
      ROS_INFO("Arming set");
    }
    else
    {
      ROS_ERROR("Something went wrong in arming. Pis!");
    }
  }
};

// To build changes from this service run catkin build in ~/catkin_ws

// Start the multi UAV PX4 program simulation
// Call this service using "rosrun test_ros arm_uav" in command line

int main(int argc, char **argv)
{

  const int numberOfUAVs = 1;

  ros::init(argc, argv, "arm_uav");

  ros::NodeHandle n;

  // std::vector<UAV> uavs;
  // uavs.reserve(numberOfUAVs);

  // for (int i = 0; i < numberOfUAVs; i++) {
  //   uavs.push_back(UAV(n, i));
  // }

  // for (int i = 0; i < numberOfUAVs; i++) {
  //   // uavs[i].takeOff(i + 4);
  //   uavs[i].run();
  // }

  UAV tmp(n);

  tmp.takeOff(10);

  ROS_INFO("Spinning");

  ros::spin();

  return 0;
}