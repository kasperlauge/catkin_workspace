#include "recognizer.h"


Recognizer::Recognizer(ros::NodeHandle n)
{
    Recognizer::nodeHandle = n;
};

bool Recognizer::processImages(recon_msgs::ImageInfo::Request &req, recon_msgs::ImageInfo::Response &res)
{
    ROS_INFO("recognize_slz called!");
    // Parameters
    int lowThreshold = 50;
    const int ratio = 3;
    const int kernel_size = 3;

    std::vector<sensor_msgs::Image> images(req.Images);
    auto numberOfImages = images.size();

    for (int i = 0; i < images.size(); i++) {
        auto sensorImg = images.at(i);
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(images[i], sensor_msgs::image_encodings::BGR8);
        cv::Mat currentImg = cv_ptr->image;

        // Store image file for evaluation
        std::ostringstream oss;
        oss << "/home/kasperlauge/Desktop/samples/img_" << i << ".png";
        cv::imwrite(oss.str(), currentImg);
        // end store image file


        cv::Mat candidateIndices = cv::Mat::zeros(currentImg.size(), CV_8UC1);
        cv::Mat grayImg;
        cv::Mat detected_edges;
        cv::cvtColor(currentImg, grayImg, cv::COLOR_RGB2GRAY);
        cv::blur(grayImg, detected_edges, cv::Size(kernel_size,kernel_size));
        cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

        // Segment areas
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(detected_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        cv::Mat markers = cv::Mat::zeros(detected_edges.size(), CV_32SC1);

        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::drawContours(markers, contours, static_cast<int>(i), cv::Scalar(static_cast<int>(i)+1), -1);
        }
        cv::Mat imgResult;
        detected_edges.convertTo(imgResult, CV_8UC3);
        cv::cvtColor(imgResult, imgResult, cv::COLOR_GRAY2RGB);
        cv::circle(markers, cv::Point(5,5), 3, cv::Scalar(255), -1);
        cv::watershed(imgResult, markers);

        // Now classify the different segment of the images
        cv::Mat classifiedImage;

        // Filter out UAV and sky
        cv::Vec3b averageSkyColor(178,178,178);
        cv::Vec3b averageUavColor(26,26,26);

        // Special case of background pixels
        auto th = 5;
        double slzGaborTh = -50;
        auto bgIndices = markers == 255;
        cv::Scalar average = cv::mean(currentImg, bgIndices);
        cv::Vec3b averageSegment(average[0], average[1], average[2]);
        auto distSky = cv::norm(averageSegment, averageSkyColor);
        auto distUav = cv::norm(averageSegment, averageUavColor);

        if (distSky < th || distUav < th) {
            cv::Mat regions;
            cv::bitwise_or(candidateIndices, bgIndices, regions);
            regions.copyTo(candidateIndices);
        }

        // Process the rest of the contours
        for (int k = 0; k < contours.size(); k++) {
            auto indices = markers == (k + 1);
            average = cv::mean(currentImg, indices);
            cv::Vec3b averageSegment(average[0], average[1], average[2]);
            auto distSky = cv::norm(averageSegment, averageSkyColor);
            auto distUav = cv::norm(averageSegment, averageUavColor);
            if (distSky < th || distUav < th) {
                cv::Mat regions;
                cv::bitwise_or(candidateIndices, indices, regions);
                regions.copyTo(candidateIndices);
            } else {
                // Process non-background and non-UAV segments
                int kernel_size = 9;
                double sig = 5, lm = 3.0, gm = 0.05, ps = CV_PI/4;
                double theta = 45;
                double th = 15;
                double maxValue = 255;

                auto gaborKernel = cv::getGaborKernel(cv::Size(kernel_size,kernel_size), sig, theta, lm, gm, ps, CV_32F);

                cv::Mat coordinates(indices);
                cv::Mat roiImg;
                grayImg.copyTo(roiImg, coordinates);

                cv::Mat filteredRoi;
                cv::filter2D(roiImg, filteredRoi, CV_32F, gaborKernel);
                auto avgResponse = cv::mean(filteredRoi, coordinates);
                
                // If the gabor response is too high the segment is not very flat
                if (avgResponse[0] > slzGaborTh) {
                    cv::Mat regions;
                    cv::bitwise_or(candidateIndices, indices, regions);
                    regions.copyTo(candidateIndices);
                }
            }
        }

        // Extract image coordinates based on all non-zero coordinates
        cv::Mat reverse;
        cv::bitwise_not(candidateIndices, reverse);
        cv::Mat candidateRegions;
        cv::findNonZero(reverse, candidateRegions);
        auto numberOfNonZeroCoordinates = candidateRegions.total();

        recon_msgs::ImageCoordinateData imgRes;
        imgRes.x.reserve(numberOfNonZeroCoordinates);
        imgRes.y.reserve(numberOfNonZeroCoordinates);

        for (int j = 0; j < numberOfNonZeroCoordinates; j++) {
            int x = candidateRegions.at<cv::Point>(j).x;
            int y = candidateRegions.at<cv::Point>(j).y;
            imgRes.x.push_back(x);
            imgRes.y.push_back(y);
        }

        res.SlzData.ImageCoordinateData.push_back(imgRes);
        res.SlzData.PointClouds = req.pointClouds;
    }

    return true;
};

void Recognizer::setup()
{
    Recognizer::service = Recognizer::nodeHandle.advertiseService("find_slz", &Recognizer::processImages, this);
}