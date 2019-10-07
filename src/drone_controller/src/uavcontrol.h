#ifndef SOME_HEADER_FILE_H
#define SOME_HEADER_FILE_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

class UAVController
{
private:
    /* data */
    geometry_msgs::PoseStamped _actualpose;
    ros::NodeHandle _nodehandle;
    ros::Publisher _position_publisher;
    geometry_msgs::PoseStamped _goalpose;

    void cb_setCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        this->_actualpose = *msg;
    }

public:
    UAVController(ros::NodeHandle _nh);
    ~UAVController();
    void publish();

    void rotate(double rad);
};

void UAVController::publish()
{
    // ROS_INFO("Publishing goal pose");
    _position_publisher.publish(this->_goalpose);
}

void UAVController::rotate(double rad)
{
    // Declare quaternions for future use
    tf2::Quaternion q_orig, q_rot, q_new;

    // Get Current pose in quaternion
    tf2::convert(this->_actualpose.pose.orientation, q_orig);

    //Rotate by rad in yaw direction
    q_rot.setRPY(0, 0, rad);

    //Calculate new pose
    q_new = q_rot * q_orig;
    q_new.normalize();

    // Put new pose into goalpose
    tf2::convert(q_new, this->_goalpose.pose.orientation);
}

UAVController::UAVController(ros::NodeHandle _nh)
{
    this->_nodehandle = _nh;
    this->_position_publisher = this->_nodehandle.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    this->_nodehandle.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 1, &UAVController::cb_setCurrentPose, this);
    geometry_msgs::PoseStamped tmp;
    tmp.pose.position.x = 0;
    tmp.pose.position.y = 0;
    tmp.pose.position.z = 6;
    // tmp.pose.position.x = 0;
    // tmp.pose.position.x = 0;
    // tmp.pose.position.x = 0;
    
}

UAVController::~UAVController()
{
}

#endif