#ifndef UAV_FLIGHTSTRAT_H
#define UAV_FLIGHTSTRAT_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "uavcontrol.h"
#include "slz_recognition/ImageInfo.h"
/**
 * Base class for a Strategy pattern. There could be different overall strategies for finding a SLZ
 * and eventually landing. These stratgies should inhierit this class.
 * 
 */
class FligtStatregy
{
private:
protected:
    /* data */
    ros::NodeHandle _nodehandle;
    UAVController _uav_control;

public:
    FligtStatregy(ros::NodeHandle _nh);
    virtual void run() = 0;
};

FligtStatregy::FligtStatregy(ros::NodeHandle _nh) : _uav_control(_nh), _nodehandle(_nh)
{
}

/**
 * Flight strategy for initial sampling of images to be used for SLZ detection
 * 
 */
class SampleStrat : public FligtStatregy
{
private:
    /* data */
    std::vector<sensor_msgs::Image> _sample_images;
    std::vector<sensor_msgs::PointCloud2> _sample_cloud;
    ros::Time created_time;
    ros::Time elapsed_time;

public:
    SampleStrat(ros::NodeHandle _nh);
    ~SampleStrat();
    /* virtual */ void run();
};

/**
 * @Constructor
 * 
 * @param _nh Ros nodehandle
 */
SampleStrat::SampleStrat(ros::NodeHandle _nh) : FligtStatregy(_nh)
{
    created_time = ros::Time::now();
}

/**
 * The actual implementation of the sampling strategy. Takes an image every 4 seconds, and rotates
 * 90 degrees between each. Runs 4 times.
 * 
 */
void SampleStrat::run()
{
    int counter = 0;
    ros::Rate rate(20);
    while (ros::ok() && counter < 4)
    {
        //Wait 30 seconds before starting to sample. (Could be changed with checking that UAV was close to position)
        if (ros::Time::now() - created_time > ros::Duration(25.0) && ros::Time::now() - elapsed_time > ros::Duration(2.0))
        {
            ROS_INFO("Image Taken");

            //Sample an image and a pointcloud
            auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", this->_nodehandle);
            auto cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/iris_sensors_0/camera_red_iris/depth/points", this->_nodehandle);
            _sample_cloud.push_back(*cloud);
            _sample_images.push_back(*image);

            //Rotate UAV 90 degrees
            _uav_control.rotate(M_PI / 2);

            this->elapsed_time = ros::Time::now();
            counter++;
        }
        //publish goalpose
        _uav_control.publish();
        rate.sleep();
    }

    // auto tmp = ros::service::waitForService("find_slz");
    slz_recognition::ImageInfo msg;
    msg.request.Images = _sample_images;
    msg.request.pointClouds = _sample_cloud;

    ros::ServiceClient client = this->_nodehandle.serviceClient<slz_recognition::ImageInfo>("find_slz");

    if (client.call(msg))
    {
        // DO something!
    }

    ROS_INFO("Leaving strategy");
}

#endif