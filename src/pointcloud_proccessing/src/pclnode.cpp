#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

void viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(1.0, 0.5, 1.0);
    // pcl::PointXYZ o;
    // o.x = 1.0;
    // o.y = 0;
    // o.z = 0;
    // viewer.addSphere(o, 0.25, "sphere", 0);
    // std::cout << "i only run once" << std::endl;
}

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "pclproc");
    ros::NodeHandle nh;
    ROS_INFO("Coordinate transform running");

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lars/test_pcd.pcd", *outputCloud);
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*outputCloud, *cloud, indices);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ROS_INFO("Cform running");
    ec.extract(cluster_indices);
    ROS_INFO("Coordinate transfosadfag");

    int count = 1;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        std::cout << cluster_indices.size() << std::endl;
        ROS_INFO("Coordinate  running");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        std::string name = "/home/lars/hei" + count;
        pcl::io::savePCDFileASCII(name, *cloud_cluster);
        count++;
    }
    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // viewer.showCloud(cloud_cluster);

    // viewer.runOnVisualizationThreadOnce(viewerOneOff);

    // while (!viewer.wasStopped())
    // {
    //     //you can also do cool processing here
    //     //FIXME: Note that this is running in a separate thread from viewerPsycho
    //     //and you should guard against race conditions yourself...
    // }

    ros::spin();

    return 0;
};