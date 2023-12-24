//
// Created by qin on 05/10/23.
//

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Time.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include "vision_msgs/Detection2DArray.h"

#include "Thirdparty/DLib/FileFunctions.h"

using namespace std;

void tokenize(std::string const &str, const char delim,
              std::vector<std::string> &out) {
    size_t start; // size_t is Unsigned integral type, alias of one of the fundamental unsigned integer types.
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != std::string::npos) {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "BagFromRGBAndDepth");

    if (argc != 6) {
        cerr
                << "Usage: rosrun BagFromImages BagFromImages <path to depth directory> "
                << "<path to image directory> <image extension .ext> <frequency> <path to output bag> "
                << endl;
        return 0;
    }

    ros::start();

    // vector of paths to depth
    vector<string> depth_filenames = DUtils::FileFunctions::Dir(argv[1], argv[3], true);

    // Vector of paths to image
    vector<string> image_filenames = DUtils::FileFunctions::Dir(argv[2], argv[3], true);

    cout << "Images: " << image_filenames.size() << endl;

    // Frequency
    double freq = atof(argv[4]);

    // Output bag
    rosbag::Bag bag_out(argv[5], rosbag::bagmode::Write);

    ros::Time t = ros::Time::now();

    const float T = 1.0f / freq;
    ros::Duration d(T);

    for (size_t i = 0; i < image_filenames.size(); i++) {
        if (!ros::ok())
            break;

        cv::Mat im = cv::imread(image_filenames[i], cv::IMREAD_COLOR);
        cv::Mat depth = cv::imread(depth_filenames[i], cv::IMREAD_ANYDEPTH);

        cv_bridge::CvImage cvImage, cvDepth;

        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::BGR8;
        cvImage.header.stamp = t;
        bag_out.write("/dji_osdk_ros/main_camera_images", ros::Time(t), cvImage.toImageMsg());

        cvDepth.image = depth;
        cvDepth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        cvDepth.header.stamp = t;
        bag_out.write("/dji_osdk_ros/main_camera_depth_images", ros::Time(t), cvDepth.toImageMsg());

        t += d;
        cout << i << " / " << image_filenames.size() << endl;

        // show images
        cv::imshow("rgb", im);
        cv::imshow("depth", depth);
        cv::waitKey(1);
    }

    bag_out.close();

    ros::shutdown();

    return 0;
}
