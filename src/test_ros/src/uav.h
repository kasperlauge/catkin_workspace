#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class UAV
{
    ros::NodeHandle nodeHandle;
    int uavNumber;
    int takeOffMeters;
    bool hasBeenArmed = false;
    ros::Subscriber stateSubscriber;
    ros::Subscriber positionSubscriber;
    mavros_msgs::State current_state;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Publisher local_pos_pub;
    geometry_msgs::PoseStamped pose;

public:
    UAV(ros::NodeHandle n, int uavNumber);
    void takeOff(const int meters);
    void setOffboard();
    void turn();
    void setPose(geometry_msgs::PoseStamped _pose);
    void run();
    void state_cb(const mavros_msgs::State::ConstPtr &msg);

private:
    void armed(const mavros_msgs::State::ConstPtr &msg);
    void armUAV();
    void takeOffUAV(const int meters);
};