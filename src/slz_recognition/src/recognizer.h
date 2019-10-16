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
#include "recon_msgs/ImageInfo.h"

class Recognizer {
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service;
    public:
        Recognizer(ros::NodeHandle n);
        bool processImages(recon_msgs::ImageInfo::Request &req, recon_msgs::ImageInfo::Response &res);
        void setup();
};