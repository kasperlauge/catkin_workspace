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
#include "coordinate_transformation/CoordinateInfo.h"

class Transformer {
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service;
    public:
        Transformer(ros::NodeHandle n);
        bool transformCoordinates(coordinate_transformation::CoordinateInfo::Request &req, coordinate_transformation::CoordinateInfo::Response &res);
        void setup();
};