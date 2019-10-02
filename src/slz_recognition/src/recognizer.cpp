#include "recognizer.h"

Recognizer::Recognizer(ros::NodeHandle n)
{
    Recognizer::nodeHandle = n;
};

bool Recognizer::processImages(slz_recognition::ImageInfo::Request &req, slz_recognition::ImageInfo::Response &res)
{
    ROS_INFO("recognize_slz called!");

    // Get the enriched images from the request
    // Mocked by the raw image
    auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/iris_sensors_0/camera_red_iris/image_raw", Recognizer::nodeHandle);
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat output;
    cv::Mat inputGrey;
    cv::Mat thImg;
    cv::Mat reverseThImg;
    cv::Mat candidateRegions;
    cv::Mat typeTransform;

    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);


    // Process image - this part should find all regions which makes good SLZs
    // After they have been found mark all the SLZ pixels a 255 and all other pixels as 0
    cv::cvtColor(cv_ptr->image, inputGrey, cv::COLOR_BGR2GRAY);

    int kernel_size = 9;
    double sig = 5, lm = 4, gm = 0.05, ps = CV_PI/4;
    double theta = 45;
    double th = 15;
    double maxValue = 255;

    auto gaborKernel = cv::getGaborKernel(cv::Size(kernel_size,kernel_size), sig, theta, lm, gm, ps, CV_32F);
    cv::filter2D(inputGrey, output, CV_32F, gaborKernel);
    cv::threshold(output, thImg, th, maxValue, cv::THRESH_BINARY);
    cv::subtract(cv::Scalar::all(255),thImg,reverseThImg);



    // Extract image coordinates based on all non-zero coordinates
    cv::Mat processedImage = reverseThImg;
    processedImage.convertTo(typeTransform, CV_8UC1);
    cv::findNonZero(typeTransform, candidateRegions);
    auto numberOfNonZeroCoordinates = candidateRegions.total();
    res.x.reserve(numberOfNonZeroCoordinates);
    res.y.reserve(numberOfNonZeroCoordinates);

    for (int i = 0; i < numberOfNonZeroCoordinates; i++) {
        res.x.push_back(candidateRegions.at<cv::Point>(i).x);
        res.y.push_back(candidateRegions.at<cv::Point>(i).y);
    }

    // cv::imshow("OPENCV_WINDOW1", inputGrey);
    // cv::imshow("OPENCV_WINDOW2", output);
    // cv::imshow("OPENCV_WINDOW3", thImg);
    // cv::imshow("OPENCV_WINDOW4", reverseThImg);
    // cv::waitKey(0);

    return true;
};

void Recognizer::setup()
{
    Recognizer::service = Recognizer::nodeHandle.advertiseService("find_slz", &Recognizer::processImages, this);
}