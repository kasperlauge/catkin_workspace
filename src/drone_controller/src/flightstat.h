#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "uavcontrol.h"

class FligtStatregy
{
private:
protected:
    /* data */
    ros::NodeHandle _nodehandle;
    UAVController _uav_control;

public:
    FligtStatregy(ros::NodeHandle _nh);
    ~FligtStatregy();
    virtual void run() = 0;
};

FligtStatregy::FligtStatregy(ros::NodeHandle _nh) : _uav_control(_nh), _nodehandle(_nh)
{
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
    ros::Rate rate(20);
    while (ros::ok())
    {

        // ROS_INFO("SampleStrat is run");
        if (ros::Time::now() - created_time > ros::Duration(20.0) && ros::Time::now() - elapsed_time > ros::Duration(4.0))
        {
            ROS_INFO("Image Taken");
            
            auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", this->_nodehandle);
            _sample_images.push_back(*image);


            // _uav_control.rotate(M_PI / 2);

            this->elapsed_time = ros::Time::now();
            
        }
        _uav_control.publish();
        rate.sleep();
    }
}
