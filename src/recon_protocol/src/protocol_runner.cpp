#include "protocol_runner.h"
#include "recon_msgs/Sampleimages.h"
#include "recon_msgs/ImageInfo.h"
#include "recon_msgs/CoordinateInfo.h"

ProtocolRunner::ProtocolRunner(ros::NodeHandle n)
{
    ProtocolRunner::nodeHandle = n;
};

bool ProtocolRunner::handleProtocolCall(recon_msgs::ProtocolInfo::Request &req, recon_msgs::ProtocolInfo::Response &res)
{
    ROS_INFO("recon protocol called!");

    recon_msgs::Sampleimages sampleMsg;

    if (this->sampling_client.call(sampleMsg))
    {
        ROS_INFO("Sampling returned");
        recon_msgs::ImageInfo reconMsg;
        reconMsg.request.Images = sampleMsg.response.Images;
        reconMsg.request.pointClouds = sampleMsg.response.pointClouds;
        if (this->slz_recognition_client.call(reconMsg))
        {
            ROS_INFO("Images returned");
            recon_msgs::CoordinateInfo coordinateMsg;
            coordinateMsg.request.SlzData = reconMsg.response.SlzData;
            if (this->transform_coordinates_client.call(coordinateMsg))
            {

                // for (auto &j : coordinateMsg.response.CoordinateData)
                // {
                //     for (auto &k : j.x)
                //     {
                //         ROS_INFO("X coordinate could be: %f", k);
                //     }
                // }

                // ROS_INFO("Coordinate transform returned");
                // for (int i = 0; i < coordinateMsg.response.CoordinateData.at(1).x.size(); i++) {
                //     int x = coordinateMsg.response.CoordinateData.at(1).x.at(i);
                //     int y = coordinateMsg.response.CoordinateData.at(1).y.at(i);
                //     int z = coordinateMsg.response.CoordinateData.at(1).z.at(i);
                //     std::cout << "[x,y,z]: [" << x << "," << y << "," << z << "]" << std::endl;
                // }

                std::vector<recon_msgs::CoordinateData> coordinateData(coordinateMsg.response.CoordinateData);
                recon_msgs::SLZCoordinates slz_coordinates;
                slz_coordinates.CoordinateData = coordinateData;
                ProtocolRunner::coordinate_publisher.publish(slz_coordinates);
            }
        }
    }

    res.Success = true;

    return true;
};

void ProtocolRunner::setup()
{
    this->sampling_client = this->nodeHandle.serviceClient<recon_msgs::Sampleimages>("sampling");
    this->slz_recognition_client = this->nodeHandle.serviceClient<recon_msgs::ImageInfo>("find_slz");
    this->transform_coordinates_client = this->nodeHandle.serviceClient<recon_msgs::CoordinateInfo>("transform_coordinates");

    // Advertise topic with coordinates
    ProtocolRunner::coordinate_publisher = ProtocolRunner::nodeHandle.advertise<recon_msgs::SLZCoordinates>("slz_coordinates", 1000);

    ProtocolRunner::service = ProtocolRunner::nodeHandle.advertiseService("recon_protocol", &ProtocolRunner::handleProtocolCall, this);
}