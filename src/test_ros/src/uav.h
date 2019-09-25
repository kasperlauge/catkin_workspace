#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "sensor_msgs/NavSatFix.h"

class UAV {
    ros::NodeHandle nodeHandle;
    int uavNumber;
    int takeOffMeters;
    bool hasBeenArmed = false;
    ros::Subscriber stateSubscriber;
    ros::Subscriber positionSubscriber;
    public:
        UAV(ros::NodeHandle n, int uavNumber);
        void takeOff(const int meters);
    private:
        void armed(const mavros_msgs::State::ConstPtr& msg);
        void armUAV();
        void takeOffUAV(const int meters);
};