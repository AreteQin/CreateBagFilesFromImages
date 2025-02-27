#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Time.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>

#include "Thirdparty/DLib/FileFunctions.h"


using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "BagFromImages");

    if (argc != 5) {
        cerr
                << "Usage: rosrun BagFromImages BagFromImages <path to image directory> <image extension .ext> <frequency> <path to output bag>"
                << endl;
        return 0;
    }

    ros::start();

    // Vector of paths to image
    vector<string> filenames = DUtils::FileFunctions::Dir(argv[1], argv[2], true);

    cout << "Images: " << filenames.size() << endl;

    // Frequency
    double freq = atof(argv[3]);

    // Output bag
    rosbag::Bag bag_out(argv[4], rosbag::bagmode::Write);

    ros::Time t = ros::Time::now();

    const float T = 1.0f / freq;
    ros::Duration d(T);

    for (size_t i = 0; i < filenames.size(); i++) {
        if (!ros::ok())
            break;

        cv::Mat im = cv::imread(filenames[i], cv::IMREAD_COLOR);
        cv_bridge::CvImage cvImage;
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::BGR8;
        cvImage.header.stamp = t;
        bag_out.write("/dji_osdk_ros/main_wide_RGB", ros::Time(t), cvImage.toImageMsg());
        t += d;
        cout << i << " / " << filenames.size() << endl;
    }

    bag_out.close();

    ros::shutdown();

    return 0;
}
