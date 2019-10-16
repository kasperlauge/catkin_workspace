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
#include "recon_msgs/CoordinateInfo.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class Transformer {
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service;
    public:
        Transformer(ros::NodeHandle n);
        bool transformCoordinates(recon_msgs::CoordinateInfo::Request &req, recon_msgs::CoordinateInfo::Response &res);
        void setup();
};