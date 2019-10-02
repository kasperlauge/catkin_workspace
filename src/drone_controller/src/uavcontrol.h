#ifndef SOME_HEADER_FILE_H
#define SOME_HEADER_FILE_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class UAVController
{
private:
    /* data */
    geometry_msgs::PoseStamped _actualpose;
    ros::NodeHandle _nodehandle;
    ros::Publisher _position_publisher;
    geometry_msgs::PoseStamped _goalpose;

    mavros_msgs::State current_state;

    void cb_setCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        this->_actualpose = *msg;
    }

public:
    UAVController(ros::NodeHandle _nh);
    ~UAVController();

    void rotate(double rad);
};

void UAVController::rotate(double rad)
{
    this->_actualpose
}

UAVController::UAVController(ros::NodeHandle _nh)
{
    this->_nodehandle = _nh;
    this->_position_publisher = this->_nodehandle.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    this->_nodehandle.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 1, &UAVController::cb_setCurrentPose, this);
}

UAVController::~UAVController()
{
}

#endif