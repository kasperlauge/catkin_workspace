#include "recognizer.h"

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "slz_recognition");
    ros::NodeHandle nh;

    Recognizer recognizer(nh);
    recognizer.setup();

    ros::spin();

    return 0;
};