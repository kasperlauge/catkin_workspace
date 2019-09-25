#include "uav.h"
#include <mavros_msgs/CommandBool.h>
#include "mavros_msgs/CommandTOL.h"

UAV::UAV(ros::NodeHandle n, int uavNumber) {
    UAV::nodeHandle = n;
    UAV::uavNumber = uavNumber;
};

void UAV::armUAV() {
    auto armingService = "/uav" + std::to_string(UAV::uavNumber) + "/mavros/cmd/arming";
    ROS_INFO_STREAM(armingService);
    ros::service::waitForService(armingService); 

    ros::ServiceClient set_arming_client = UAV::nodeHandle.serviceClient<mavros_msgs::CommandBool>(armingService);

    mavros_msgs::CommandBool set_arming_srv;
    set_arming_srv.request.value = 1;

    if (set_arming_client.call(set_arming_srv)) {
        ROS_INFO("Arming set");
    }
    else {
        ROS_ERROR("Something went wrong in arming. Pis!");
    }
};

void UAV::takeOffUAV(const int meters) {
    auto takeOffService = "/uav" + std::to_string(UAV::uavNumber) + "/mavros/cmd/takeoff";
    ros::service::waitForService(takeOffService); 

    ros::ServiceClient set_takeoff_client = UAV::nodeHandle.serviceClient<mavros_msgs::CommandTOL>(takeOffService);

    mavros_msgs::CommandTOL set_takeoff_srv;

    auto positionTopic = "/uav" + std::to_string(UAV::uavNumber) + "/mavros/global_position/global";
    auto currentPosition = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(positionTopic);

    set_takeoff_srv.request.altitude = currentPosition->altitude - 40 + meters;
    set_takeoff_srv.request.latitude = currentPosition->latitude;
    set_takeoff_srv.request.longitude = currentPosition->longitude;

    if (set_takeoff_client.call(set_takeoff_srv)) {
        ROS_INFO("Takeoff set");
    }
    else {
        ROS_ERROR("Something went wrong in takeoff. Pis!");
    }
};

void UAV::armed(const mavros_msgs::State::ConstPtr& msg) {
    if (msg->armed > 0) {
        UAV::takeOffUAV(UAV::takeOffMeters);
        UAV::stateSubscriber.shutdown();
    }
};

void UAV::takeOff(const int meters) {
    UAV::takeOffMeters = meters;
    // Listen for the armed state - and takeoff when armed
    auto stateTopic = "/uav" + std::to_string(UAV::uavNumber) + "/mavros/state";
    UAV::stateSubscriber = UAV::nodeHandle.subscribe<mavros_msgs::State>(stateTopic, 1, &UAV::armed, this);

    // Arm the UAV
    UAV::armUAV();
};