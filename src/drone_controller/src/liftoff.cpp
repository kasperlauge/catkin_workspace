#include "ros/ros.h"

#include "mavros_msgs/State.h"
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/tf.h"
#include "ros/ros.h"
#include <mavros_msgs/CommandBool.h>
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/SetMode.h>
#include "image_sampling/GetSampledImages.h"
#include "slz_recognition/ImageInfo.h"

class UAV
{
private:
    /* data */
    ros::NodeHandle nodeHandle;
    ros::Subscriber stateSubscriber;
    ros::Time last_request;

    mavros_msgs::State current_state;

    ros::ServiceClient slz_recognition_client;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient get_image_client;
    ros::Publisher local_pos_pub;
    geometry_msgs::PoseStamped pose;

    std::vector<std::tuple<int, int>> slz_positions;

    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void setPose(geometry_msgs::PoseStamped _pose);
    void rotate(float rad);
    void setInitialPosition();

public:
    UAV(ros::NodeHandle n);
    ~UAV();
    void run();
};

void UAV::rotate(float rad)
{
    tf2::Quaternion q_orig, q_rot, q_new;

    tf2::convert(this->pose.pose.orientation, q_orig);

    tf2::convert(this->pose.pose.orientation, q_orig);
    q_rot.setRPY(0, 0, rad);

    q_new = q_rot * q_orig;
    q_new.normalize();

    tf2::convert(q_new, this->pose.pose.orientation);
}

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

void UAV::run()
{
    ROS_INFO("RUN Called");
    this->local_pos_pub = this->nodeHandle.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    this->arming_client = this->nodeHandle.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    this->set_mode_client = this->nodeHandle.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
    this->get_image_client = this->nodeHandle.serviceClient<image_sampling::GetSampledImages>("/GetImage");
    this->slz_recognition_client = this->nodeHandle.serviceClient<slz_recognition::ImageInfo>("find_slz");

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

    //Counter for timer
    size_t counter = 0;

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

        if (!(counter % 100))
        {
            this->rotate(M_PI / 2);
            image_sampling::GetSampledImages msg;
            this->get_image_client.call(msg);

            slz_recognition::ImageInfo slz_msg;

            std::vector<sensor_msgs::Image> images;
            images.push_back(msg.response.Image);

            slz_msg.request.Image = images;
            slz_recognition_client.call(slz_msg);
            // std::tuple<int, int> p1 = std::make_pair(slz_msg.response.x[0], slz_msg.response.y[0]);
            // slz_positions.push_back(p1);

            // ROS_INFO("places are : %d", slz_msg.response.x[0]);
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
        counter++;
    }
}

void UAV::setPose(geometry_msgs::PoseStamped _pose)
{
    this->pose = _pose;
}

UAV::UAV(ros::NodeHandle n)
{
    this->nodeHandle = n;

    // Set Callback for being able to monitor State
    auto stateTopic = "/uav0/mavros/state";
    this->stateSubscriber = this->nodeHandle.subscribe<mavros_msgs::State>(stateTopic, 1, &UAV::state_cb, this);
};

UAV::~UAV()
{
}
void UAV::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    this->current_state = *msg;
}

int main(int argv, char **argc)
{
    ros::init(argv, argc, "DroneLiftoff");

    ros::NodeHandle n;

    UAV drone(n);

    drone.run();

    return 0;
}