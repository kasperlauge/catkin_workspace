#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/NavSatFix.h"
#include "image_sampling/GetSampledImages.h"

class ImageProccessor
{
     ros::NodeHandle _nh;
     ros::ServiceServer service;

public:
     ImageProccessor()
     {
          this->service = this->_nh.advertiseService("getImage", &ImageProccessor::getimage, this);
     }

     bool getimage(image_sampling::GetSampledImages::Request &req, image_sampling::GetSampledImages::Response &res)
     {
          auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", _nh);
          cv_bridge::CvImagePtr cv_ptr;

          cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

          // cv::imshow("OPENCV_WINDOW", cv_ptr->image);
          // cv::waitKey(0);

          // auto src = cv_ptr->image;
          // cv::Mat greyMat;

          // cv::cvtColor(cv_ptr->image, greyMat, CV_BGR2GRAY);

          // auto img = ;
          sensor_msgs::ImagePtr tmp = cv_ptr->toImageMsg();
          // res.Image = tmp;
          // res.Image()
          // res.Image = img.get()

          res.Image = *tmp;

          return true;
     }
};

int main(int argc, char **argv)
{
     ros::init(argc, argv, "image_converter");
     ImageProccessor ip;

     ros::spin();

     return 0;
};