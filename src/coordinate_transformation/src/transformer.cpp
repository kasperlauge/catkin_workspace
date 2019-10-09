#include "transformer.h"
#include <coordinate_transformation/CoordinateData.h>
#include <slz_recognition/SlzData.h>

Transformer::Transformer(ros::NodeHandle n)
{
    Transformer::nodeHandle = n;
};

bool Transformer::transformCoordinates(coordinate_transformation::CoordinateInfo::Request &req, coordinate_transformation::CoordinateInfo::Response &res)
{
    ROS_INFO("transform coordinates called!");
    // auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", nodeHandle);

    // Should be provided from same source as image data in meta information
    auto p_cloud1 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/iris_sensors_0/camera_red_iris/depth/points");
    auto p_cloud2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/iris_sensors_0/camera_red_iris/depth/points");
    auto p_cloud3 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/iris_sensors_0/camera_red_iris/depth/points");
    auto p_cloud4 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/iris_sensors_0/camera_red_iris/depth/points");

    std::vector<sensor_msgs::PointCloud2ConstPtr> pointClouds;
    pointClouds.reserve(4);

    pointClouds.push_back(p_cloud1);
    pointClouds.push_back(p_cloud2);
    pointClouds.push_back(p_cloud3);
    pointClouds.push_back(p_cloud4);

    std::vector<slz_recognition::SlzData> imageInfos = req.SlzData;
    auto numberOfPointClouds = imageInfos.size();
    res.CoordinateData.reserve(numberOfPointClouds);


    // Loop through all of the sampled image data - for each imagedata
    for(std::vector<int>::size_type i = 0; i != imageInfos.size(); i++) {
        // std::cout << static_cast<unsigned>(it->x.at(0)) << std::endl;
        coordinate_transformation::CoordinateData coordinateData;
        coordinateData.x.reserve(imageInfos.at(i).x.size());
        coordinateData.y.reserve(imageInfos.at(i).x.size());
        coordinateData.z.reserve(imageInfos.at(i).x.size());

        for (int j = 0; j < imageInfos.at(i).x.size(); j++) {
            // https://answers.ros.org/question/191265/pointcloud2-access-data/
            auto pointCloud = pointClouds.at(i);
            int u = imageInfos.at(i).x.at(j);
            int v = imageInfos.at(i).y.at(j);
            int arrayPosition = v*pointCloud->row_step + u*pointCloud->point_step;

            // compute position in array where x,y,z data start
            int arrayPosX = arrayPosition + pointCloud->fields[0].offset; // X has an offset of 0
            int arrayPosY = arrayPosition + pointCloud->fields[1].offset; // Y has an offset of 4
            int arrayPosZ = arrayPosition + pointCloud->fields[2].offset; // Z has an offset of 8

            int x = pointCloud->data[arrayPosX];
            int y = pointCloud->data[arrayPosY];
            int z = pointCloud->data[arrayPosZ];

            coordinateData.x.push_back(x);
            coordinateData.y.push_back(y);
            coordinateData.z.push_back(z);
        }

        res.CoordinateData.push_back(coordinateData);
    }

    return true;
};

void Transformer::setup()
{
    Transformer::service = Transformer::nodeHandle.advertiseService("transform_coordinates", &Transformer::transformCoordinates, this);
}