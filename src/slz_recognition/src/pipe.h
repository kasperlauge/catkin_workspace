#include <initializer_list>
#include <iostream>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

template <class T>
class filter
{
private:
    int a;

public:
    filter(){};

    virtual void run(T *data) = 0;
};

class addingFilter : public filter<int>
{
public:
    addingFilter(){};

    void run(int *data)
    {
        *data = *data + 1;
        std::cout << *data << " : Running\n";
    };
};

class GaussFilter : public filter<cv::Mat>
{
public:
    GaussFilter(){};

    void run(cv::Mat *data)
    {
        cv::waitKey(0);
    };
};

class GaborFilter : public filter<cv::Mat>
{
public:
    GaborFilter(){};

    void run(cv::Mat *data)
    {
        int kernel_size = 9;
        double sig = 5, lm = 4, gm = 0.05, ps = CV_PI / 4;
        double theta = 45;
        double th = 15;
        double maxValue = 255;

        cv::Mat output;
        cv::Mat thImg;
        cv::Mat reverseThImg;
        cv::Mat candidateRegions;
        cv::Mat typeTransform;

        auto gaborKernel = cv::getGaborKernel(cv::Size(kernel_size, kernel_size), sig, theta, lm, gm, ps, CV_32F);
        cv::filter2D(*data, output, CV_32F, gaborKernel);
        cv::threshold(output, thImg, th, maxValue, cv::THRESH_BINARY);
        cv::subtract(cv::Scalar::all(255), thImg, reverseThImg);
        
        cv::imshow("OPENCV_WINDOW4", reverseThImg);
        cv::waitKey(0);
        
        // Extract image coordinates based on all non-zero coordinates
        cv::Mat processedImage = reverseThImg;
        processedImage.convertTo(typeTransform, CV_8UC1);
        cv::findNonZero(typeTransform, candidateRegions);
    };
};

/**
 * Template class for making a pipe. filters are added, and can be used for data manipulation
 */

template <class T>
class imagepipe
{
private:
    /* data */
    T *data;
    std::vector<filter<T> *> filters;

public:
    imagepipe(T *data, std::initializer_list<filter<T> *> args) : data(data)
    {
        for (filter<T> *i : args)
            filters.push_back(i);
    }

    void runfilter()
    {
        for (auto it = filters.begin(); it != filters.end(); ++it)
        {
            (*it)->run(data);
        }
    }
};