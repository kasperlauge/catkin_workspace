#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "sensor_msgs/NavSatFix.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Image.h"
#include "recon_protocol/ProtocolInfo.h"
#include "recon_protocol/SLZCoordinates.h"

class ProtocolRunner {
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service;
    ros::ServiceClient sampling_client;
    ros::ServiceClient slz_recognition_client;
    ros::ServiceClient transform_coordinates_client;
    ros::Publisher coordinate_publisher;
    public:
        ProtocolRunner(ros::NodeHandle n);
        bool handleProtocolCall(recon_protocol::ProtocolInfo::Request &req, recon_protocol::ProtocolInfo::Response &res);
        void setup();
};