#include "uav.h"
#include <mavros_msgs/CommandBool.h>
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/SetMode.h>

#include "tf/tf.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include <sstream>
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
// #include "t/tf.h"
#include "tf/tf.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



UAV::UAV(ros::NodeHandle n, int uavNumber)
{
    UAV::nodeHandle = n;
    UAV::uavNumber = uavNumber;

    // Set Callback for being able to monitor State
    // auto stateTopic = "/uav" + std::to_string(UAV::uavNumber) + "/mavros/state";
    auto stateTopic = "/uav0/mavros/state";

    ROS_INFO(stateTopic);
    UAV::stateSubscriber = UAV::nodeHandle.subscribe<mavros_msgs::State>(stateTopic, 1, &UAV::state_cb, this);
};

void UAV::state_cb(const mavros_msgs::State::ConstPtr &msg)
{

    this->current_state = *msg;

    ROS_INFO("Status %d",current_state.connected);
}

void UAV::armUAV()
{
    auto armingService = "/uav" + std::to_string(UAV::uavNumber) + "/mavros/cmd/arming";
    ROS_INFO_STREAM(armingService);
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
};

void UAV::takeOffUAV(const int meters)
{
    auto takeOffService = "/uav" + std::to_string(UAV::uavNumber) + "/mavros/cmd/takeoff";
    ros::service::waitForService(takeOffService);

    ros::ServiceClient set_takeoff_client = UAV::nodeHandle.serviceClient<mavros_msgs::CommandTOL>(takeOffService);

    mavros_msgs::CommandTOL set_takeoff_srv;

    auto positionTopic = "/uav" + std::to_string(UAV::uavNumber) + "/mavros/global_position/global";
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

void UAV::armed(const mavros_msgs::State::ConstPtr &msg)
{
    if (msg->armed > 0)
    {
        UAV::takeOffUAV(UAV::takeOffMeters);
        UAV::stateSubscriber.shutdown();
    }
};

void UAV::takeOff(const int meters)
{
    UAV::takeOffMeters = meters;
    // Listen for the armed state - and takeoff when armed

    // Arm the UAV
    UAV::armUAV();

    ros::Rate rate2(10);

    rate2.sleep();

    for (size_t i = 0; i < 20; i++)
    {
        ros::spinOnce();
        rate2.sleep();
        ROS_INFO("%d", this->current_state.armed);
    }
    ROS_INFO("CB currentStateSet %d", this->current_state.armed);

    takeOffUAV(meters);

    for (size_t i = 0; i < 20; i++)
    {
        ros::spinOnce();
        rate2.sleep();
        ROS_INFO("%d", this->current_state.armed);
    }
    run();
};

void UAV::setOffboard()
{
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

void UAV::run()
{
    ROS_INFO("RUN Called");
    this->local_pos_pub = this->nodeHandle.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    this->arming_client = this->nodeHandle.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    this->set_mode_client = this->nodeHandle.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");

    tf2::Quaternion q_orig;

    // Cannot exceed 500ms due to The px4 flight stack has a timeout of 500ms between two Offboard commands
    // https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html
    ros::Rate rate(20.0);

     ROS_INFO("GOT HERE1!");
    // Wait for connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("%d",current_state.connected);
    }
    ROS_INFO("GOT HERE!");

    q_orig.setRPY(0, 0, 0);
    this->pose.pose.position.x = 0;
    this->pose.pose.position.y = 0;
    this->pose.pose.position.z = 6;
    tf2::convert(q_orig, this->pose.pose.orientation);

    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(this->pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ros::Rate rate2(10);
    while (ros::ok())
    {
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
        tf2::Quaternion q_rot, q_new;

        tf2::convert(this->pose.pose.orientation, q_orig);
        q_rot.setRPY(0, 0, 1.1 / 10);

        q_new = q_rot * q_orig;
        q_new.normalize();

        tf2::convert(q_new, this->pose.pose.orientation);
        // auto hep = tf::createQuaternionFromRPY();

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate2.sleep();
    }
}

void UAV::setPose(geometry_msgs::PoseStamped _pose)
{
    this->pose = _pose;
}

void UAV::turn()
{
}