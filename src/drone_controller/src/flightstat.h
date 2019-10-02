#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "uavcontrol.h"

class FligtStatregy
{
private:
protected:
    /* data */
    ros::NodeHandle _nodehandle;
public:
    FligtStatregy(ros::NodeHandle _nh);
    ~FligtStatregy();
    virtual void run() = 0;
};

FligtStatregy::FligtStatregy(ros::NodeHandle _nh)
{
    this->_nodehandle = _nh;
}

FligtStatregy::~FligtStatregy()
{
}

class SampleStrat : public FligtStatregy
{
private:
    /* data */

    std::vector<sensor_msgs::Image> _sample_images;
    ros::Time created_time;
    ros::Time elapsed_time;

public:
    SampleStrat(ros::NodeHandle _nh);
    ~SampleStrat();
    /* virtual */ void run();
};

SampleStrat::SampleStrat(ros::NodeHandle _nh) : FligtStatregy(_nh)
{
    created_time = ros::Time::now();
}

SampleStrat::~SampleStrat()
{
}

void SampleStrat::run()
{
    ROS_INFO("SampleStrat is run");
    
    // if (ros::Time::now()- created_time > ros::Duration(8) && ros::Time::now()- elapsed_time > ros::Duration(1))
    // {
    //     auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", this->_nodehandle);
    //     _sample_images.push_back(*image);

    //     ROS_INFO("Image Taken");
    //     this->rotate(M_PI / 2);
    // }
}


