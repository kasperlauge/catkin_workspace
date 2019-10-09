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
#include "flightstat.h"


class UAV
{
private:
    FligtStatregy *_flightstrat;

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
    void setup();
    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void setPose(geometry_msgs::PoseStamped _pose);
    void setInitialPosition();

    // enum FligtStategyTypes
    // {
    //     SAMPLE
    // };

    // void setStrategy(int type);

public:
    UAV(ros::NodeHandle n);
    ~UAV();
    void run();
};