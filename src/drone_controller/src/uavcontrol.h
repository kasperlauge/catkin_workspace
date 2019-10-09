#ifndef UAV_CONTROLLER_H
#define UAV_CONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * UAV Controller
 * 
 * Responsible for controlling the UAV actions. Any class or function that wishes to do 
 * actions that should change the position or orientation of the UAV should call through this interface.
 * 
 */
class UAVController
{
private:
    /* data */
    ros::NodeHandle _nodehandle;
    ros::Publisher _position_publisher;
    geometry_msgs::PoseStamped _goalpose;

public:
    UAVController(ros::NodeHandle _nh);

    void publish();
    void rotate(double rad);
};

/**
 * Publishes the goalpose, must be repeatedly called.
 */
void UAVController::publish()
{
    _position_publisher.publish(this->_goalpose);
}

/**
 * Rotate UAV specified rad, couterclockwise.
 *
 * @param rad Amount of rads to rotate UAV
 */
void UAVController::rotate(double rad)
{ 
    // Declare quaternions for future use
    tf2::Quaternion q_orig, q_rot, q_new;

    // Get Current pose in quaternion
    tf2::convert(this->_goalpose.pose.orientation, q_orig);

    //Create quaturnion for rotation using eular angles
    q_rot.setRPY(0, 0, rad);

    //Calculate new pose
    q_new = q_rot * q_orig;
    q_new.normalize();

    //Set new goalpose
    tf2::convert(q_new, this->_goalpose.pose.orientation);
}

/**
 * Constructor for the UAV controller (TODO evaluate if this should be a singleton.) 
 * 
 * @param _nh The Ros nodehandle.
 * 
 */
UAVController::UAVController(ros::NodeHandle _nh) : _nodehandle(_nh)
{
    this->_position_publisher = this->_nodehandle.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    auto pos = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose",this->_nodehandle);
    geometry_msgs::PoseStamped tmp;
    
    //Set iniital position
    this->_goalpose.pose.position.z = 6.0;
    this->_goalpose.pose.position.y = 0.0;
    this->_goalpose.pose.position.x = 0.0;
    this->_goalpose.pose.orientation = pos->pose.orientation;
}

#endif