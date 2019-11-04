#include "transformer.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

Transformer::Transformer(ros::NodeHandle n)
{
    Transformer::nodeHandle = n;
};

bool Transformer::transformCoordinates(recon_msgs::CoordinateInfo::Request &req, recon_msgs::CoordinateInfo::Response &res)
{
    ROS_INFO("transform coordinates called!");

    std::vector<sensor_msgs::PointCloud2> pointClouds(req.SlzData.PointClouds);

    std::vector<recon_msgs::ImageCoordinateData> imageInfos(req.SlzData.ImageCoordinateData);
    auto numberOfPointClouds = imageInfos.size();
    res.CoordinateData.reserve(numberOfPointClouds);

    ROS_INFO("allocated vectors");

    ROS_INFO_STREAM("nr. of images: " << imageInfos.size());
    bool tmp = true;
    // Loop through all of the sampled image data - for each imagedata
    for (std::vector<int>::size_type i = 0; i != imageInfos.size(); i++)
    {
        // Convert cloud to PCL::XYZ
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(pointClouds[i], pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        if(tmp){
            ROS_INFO("Saving cloud");
            pcl::io::savePCDFileASCII("test_pcd.pcd", *temp_cloud);
            tmp = false;
        }

        recon_msgs::CoordinateData coordinateData;
        coordinateData.x.reserve(imageInfos.at(i).x.size());
        coordinateData.y.reserve(imageInfos.at(i).x.size());
        coordinateData.z.reserve(imageInfos.at(i).x.size());

        ROS_INFO("%d", imageInfos.at(i).x.size());

        for (size_t j = 0; j < imageInfos.at(i).x.size(); j++)
        {
            const u_int32_t index = imageInfos.at(i).x.at(j) + 800 * imageInfos.at(i).y.at(j);

            if (!isnan(temp_cloud->points[index].x))
            {
                coordinateData.x.push_back(temp_cloud->points[index].x);
                coordinateData.y.push_back(temp_cloud->points[index].y);
                coordinateData.z.push_back(temp_cloud->points[index].z);
            }
        }

        res.CoordinateData.push_back(coordinateData);
    }

    return true;
};

void Transformer::setup()
{
    Transformer::service = Transformer::nodeHandle.advertiseService("transform_coordinates", &Transformer::transformCoordinates, this);
}