#include "protocol_runner.h"
#include "drone_controller/Sampleimages.h"
#include "slz_recognition/ImageInfo.h"
#include "coordinate_transformation/CoordinateInfo.h"

ProtocolRunner::ProtocolRunner(ros::NodeHandle n)
{
    ProtocolRunner::nodeHandle = n;
};

bool ProtocolRunner::handleProtocolCall(recon_protocol::ProtocolInfo::Request &req, recon_protocol::ProtocolInfo::Response &res)
{
    ROS_INFO("recon protocol called!");

    drone_controller::Sampleimages sampleMsg;

    if (this->sampling_client.call(sampleMsg)) {
        slz_recognition::ImageInfo reconMsg;
        reconMsg.request.Images = sampleMsg.response.Images;
        reconMsg.request.pointClouds = sampleMsg.response.pointClouds;
        if (this->slz_recognition_client.call(reconMsg)) {
            ROS_INFO("Images returned");
            coordinate_transformation::CoordinateInfo coordinateMsg;
            coordinateMsg.request.SlzData = reconMsg.response.SlzData;
            if (this->transform_coordinates_client.call(coordinateMsg)) {
                ROS_INFO("Coordinate transform called");
                // for (int i = 0; i < coordinateMsg.response.CoordinateData.at(1).x.size(); i++) {
                //     int x = coordinateMsg.response.CoordinateData.at(1).x.at(i);
                //     int y = coordinateMsg.response.CoordinateData.at(1).y.at(i);
                //     int z = coordinateMsg.response.CoordinateData.at(1).z.at(i);
                //     std::cout << "[x,y,z]: [" << x << "," << y << "," << z << "]" << std::endl;
                // }
                // ProtocolRunner::coordinate_publisher.publish(coordinateMsg.response.CoordinateData);
            }
        }
    }



    res.Success = true;

    return true;
};

void ProtocolRunner::setup()
{
    this->sampling_client = this->nodeHandle.serviceClient<drone_controller::Sampleimages>("sampling");
    this->slz_recognition_client = this->nodeHandle.serviceClient<slz_recognition::ImageInfo>("find_slz");
    this->transform_coordinates_client = this->nodeHandle.serviceClient<coordinate_transformation::CoordinateInfo>("transform_coordinates");

    // Advertise topic with coordinates
    // ProtocolRunner::coordinate_publisher = ProtocolRunner::nodeHandle.advertise<coordinate_transformation::CoordinateData>("slz_coordinates", 1000);

    ProtocolRunner::service = ProtocolRunner::nodeHandle.advertiseService("recon_protocol", &ProtocolRunner::handleProtocolCall, this);
}