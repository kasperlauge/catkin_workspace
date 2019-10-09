#include "mavros_msgs/State.h"
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/SetMode.h>
#include "uavcontrol.h"
#include "drone_controller/Sampleimages.h"


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
    ros::ServiceServer service;
    UAVController _uav_control;


    void setup();
    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void setPose(geometry_msgs::PoseStamped _pose);
    void setInitialPosition();

public:
    UAV(ros::NodeHandle n);
    ~UAV();
    bool sampleImages(drone_controller::Sampleimages::Request &req, drone_controller::Sampleimages::Response &res);
};