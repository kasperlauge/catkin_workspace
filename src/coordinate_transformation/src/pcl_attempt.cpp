#include "ros/ros.h"
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/project_inliers.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/surface/convex_hull.h>
// int main()
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//     if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1) //* load the file
//     {
//         PCL_ERROR("Couldn't read file test_pcd.pcd \n");
//         return (-1);
//     }

//     std::cout << cloud->width << std::endl;

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::ConvexHull<pcl::PointXYZ> chull;
//     chull.setInputCloud(cloud);
//     chull.reconstruct(*cloud_hull);

//     std::cerr << "Convex hull has: " << cloud_hull->points.size() << " data points." << std::endl;

//     pcl::PCDWriter writer;
//     writer.write("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

//     return 0;
// }

#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/gp3.h>

using namespace pcl;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_polygon");
    ros::NodeHandle n;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    else
    {

        cout << "loaded" << endl;

        cout << "begin passthrough filter" << endl;
        PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
        PassThrough<PointXYZ> filter;
        filter.setInputCloud(cloud);
        filter.filter(*filtered);
        cout << "passthrough filter complete" << endl;

        cout << "begin moving least squares" << endl;
        MovingLeastSquares<PointXYZ, PointXYZ> mls;
        mls.setInputCloud(filtered);
        mls.setSearchRadius(0.01);
        mls.setPolynomialFit(true);
        mls.setPolynomialOrder(2);
        mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
        mls.setUpsamplingRadius(0.005);
        mls.setUpsamplingStepSize(0.003);

        PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
        mls.process(*cloud_smoothed);
        cout << "MLS complete" << endl;

        cout << "begin normal estimation" << endl;
        NormalEstimationOMP<PointXYZ, Normal> ne;
        ne.setNumberOfThreads(8);
        ne.setInputCloud(filtered);
        ne.setRadiusSearch(0.5);
        Eigen::Vector4f centroid;
        compute3DCentroid(*filtered, centroid);
        ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

        PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
        ne.compute(*cloud_normals);
        cout << "normal estimation complete" << endl;
        cout << "reverse normals' direction" << endl;

        for (size_t i = 0; i < cloud_normals->size(); ++i)
        {
            cloud_normals->points[i].normal_x *= -1;
            cloud_normals->points[i].normal_y *= -1;
            cloud_normals->points[i].normal_z *= -1;
        }

        cout << "combine points and normals" << endl;
        PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
        concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);
        pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::copyPointCloud(*cloud_smoothed_normals, *mls_cloud);
        pcl::io::savePCDFile<pcl::PointXYZ>("my_ownfiltered_cloud.pcd", *mls_cloud);

        cout << "begin poisson reconstruction" << endl;
        // GridProjection<PointNormal> poisson;

        pcl::Poisson<pcl::PointNormal> poisson;
        // pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points (maximum edge length)
        // poisson.setSearchRadius(0.025);

        // // Set typical values for the parameters
        // poisson.setMu(5.5);
        // poisson.setMaximumNearestNeighbors(1000);
        // poisson.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        // poisson.setMinimumAngle(M_PI / 18);       // 10 degrees
        // poisson.setMaximumAngle(2 * M_PI);    // 120 degrees
        // poisson.setNormalConsistency(true);

        // // Poisson<PointNormal> poisson;
        poisson.setDepth(9);
        poisson.setInputCloud(cloud_smoothed_normals);
        PolygonMesh mesh;

        std::cout << "downsampled mesh" << cloud_smoothed_normals->size() << std::endl;

        // pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        // tree2->setInputCloud(cloud_smoothed_normals);
        // poisson.setSearchMethod(tree2);
        poisson.reconstruct(mesh);

        cout << "poisson reconstruction ended" << endl;
        // std::cout << mesh.cloud << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh.cloud, *vertices);

        std::vector<geometry_msgs::Point> ps;

        // ps.points.reserve(vertices->size());
        // access each vertex
        for (int idx = 0; idx < ((vertices->size()) / 3) * 3; idx++)
        {
            pcl::PointXYZ v = vertices->points[idx];

            geometry_msgs::Point tmp;

            tmp.x = v._PointXYZ::data[0];
            tmp.y = v._PointXYZ::data[1];
            tmp.z = v._PointXYZ::data[2];

            ps.push_back(tmp);
        }
        pcl::io::savePCDFile<pcl::PointXYZ>("my_ownfilteredafter_cloud.pcd", *vertices);

        std::cout << mesh.polygons.size() << std::endl;

        std::cout << "points size:" << std::endl;
        std::cout << ps.size() << std::endl;

        ros::Publisher chatter_pub = n.advertise<geometry_msgs::PolygonStamped>("mypoly", 1000);
        ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        ros::Publisher pc = n.advertise<sensor_msgs::PointCloud2>("/scan_to_clouds_pub_node/cloud", 1000);

        //only if using a MESH_RESOURCE marker type:

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        // marker.ns = "my_namespace";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.points = ps;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        ros::Rate loop_rate(1);

        sensor_msgs::PointCloud2 hep;
        pcl::toROSMsg(*vertices, hep);

        hep.header.frame_id = "world";

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPolygonMesh(mesh, "meshes", 0);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

        while (ros::ok())
        {
            pc.publish(hep);
            marker_pub.publish(marker);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return (0);
}