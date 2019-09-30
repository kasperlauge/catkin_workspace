#include "recognizer.h"
#include <mavros_msgs/CommandBool.h>
#include "mavros_msgs/CommandTOL.h"

Recognizer::Recognizer(ros::NodeHandle n)
{
    Recognizer::nodeHandle = n;
};

bool Recognizer::processImages(slz_recognition::ImageInfo::Request &req, slz_recognition::ImageInfo::Response &res)
{
    ROS_INFO("recognize_slz called!");

    std::vector<int64> tmp;

    tmp.push_back(2);
    tmp.push_back(4);

    res.x = tmp;
    res.y = tmp;
    return true;
};

void Recognizer::setup()
{
    Recognizer::service = Recognizer::nodeHandle.advertiseService("find_slz", &Recognizer::processImages, this);
}