#include "uav.h"
#include "nav_msgs/Odometry.h"
/**
 * Function to publish position preflight, needed to make offboard turn on
 * 
 */
void UAV::setInitialPosition()
{
    tf2::Quaternion q_orig;
    geometry_msgs::PoseStamped tmp;

    tmp.pose.position.x = 0;
    tmp.pose.position.y = 0;
    tmp.pose.position.z = 6;

    q_orig.setRPY(0, 0, 0);
    tf2::convert(q_orig, tmp.pose.orientation);

    setPose(tmp);
}

void UAV::setup()
{
    auto stateTopic = "/uav0/mavros/state";
    this->stateSubscriber = this->nodeHandle.subscribe<mavros_msgs::State>(stateTopic, 1, &UAV::state_cb, this);
    this->local_pos_pub = this->nodeHandle.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    this->arming_client = this->nodeHandle.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    this->set_mode_client = this->nodeHandle.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
    this->service = this->nodeHandle.advertiseService("sampling", &UAV::sampleImages, this);
}

bool UAV::sampleImages(recon_msgs::Sampleimages::Request &req, recon_msgs::Sampleimages::Response &res)
{

    ROS_INFO("RUN Called");
    bool done = false;

    // Cannot exceed 500ms due to The px4 flight stack has a timeout of 500ms between two Offboard commands
    // https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html
    ros::Rate rate(20.0);

    // Wait for connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    this->setInitialPosition();

    //Set mode to OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //Arm drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    this->last_request = ros::Time::now();

    ros::Rate rate2(10);
    while (ros::ok() && !done)
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

        if (current_state.mode == "OFFBOARD" && current_state.armed)
        {
            auto created_time = ros::Time::now();
            auto elapsed_time = ros::Time::now();
            int counter = 0;
            ros::Rate rate(20);
            while (ros::ok() && counter < 4)
            {
                //Wait 30 seconds before starting to sample. (Could be changed with checking that UAV was close to position)
                if (ros::Time::now() - created_time > ros::Duration(25.0) && ros::Time::now() - elapsed_time > ros::Duration(2.0))
                {
                    ROS_INFO("Image Taken");

                    //Sample an image and a pointcloud
                    auto position = ros::topic::waitForMessage<nav_msgs::Odometry>("/uav0/mavros/local_position/odom");
                    auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", this->nodeHandle);
                    auto cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/iris_sensors_0/camera_red_iris/depth/points", this->nodeHandle);
                    
                    res.positions.push_back(*position);
                    res.Images.push_back(*image);
                    res.pointClouds.push_back(*cloud);

                    //Rotate UAV 90 degrees
                    this->_uav_control.rotate(M_PI / 2);

                    elapsed_time = ros::Time::now();
                    counter++;
                }
                //publish goalpose
                _uav_control.publish();
                rate.sleep();
            }
            done = true;
        }

        local_pos_pub.publish(this->pose);
        ros::spinOnce();
        rate.sleep();
    }

    return true;
}


void UAV::setPose(geometry_msgs::PoseStamped _pose)
{
    this->pose = _pose;
}

UAV::UAV(ros::NodeHandle n) : _uav_control(n)
{
    this->nodeHandle = n;

    // Set Callback for being able to monitor State
    this->setup();
};

/**
 * Will change when there is a set strategy function.
 */
UAV::~UAV()
{
}

void UAV::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    this->current_state = *msg;
}