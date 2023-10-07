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
    ros::init(argc, argv, "BagFromImagesAndBoxes");

    if (argc != 6) {
        cerr
                << "Usage: rosrun BagFromImages BagFromImages <path to boxes directory> "
                << "<path to image directory> <image extension .ext> <frequency> <path to output bag> "
                << endl;
        return 0;
    }

    ros::start();

    // vector of paths to boxes
    vector<string> boxes_filenames = DUtils::FileFunctions::Dir(argv[1], "csv", true);

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
        // read boxes
        ifstream boxes_file(boxes_filenames[i]);
        string line_string;
        getline(boxes_file, line_string); // skip the first line
        vision_msgs::Detection2DArray detections;
        while (getline(boxes_file, line_string)) {
            std::vector<string> data;
            tokenize(line_string, ',', data);
            int x_min = stoi(data[1]);
            int y_min = stoi(data[2]);
            int x_max = stoi(data[3]);
            int y_max = stoi(data[4]);
            vision_msgs::Detection2D detection;
            detection.bbox.center.x = (x_min + x_max) / 2;
            detection.bbox.center.y = (y_min + y_max) / 2;
            detection.bbox.size_x = x_max - x_min;
            detection.bbox.size_y = y_max - y_min;
            detections.detections.push_back(detection);
        }
        // draw rectangular boxes on image
        for (size_t i = 0; i < detections.detections.size(); i++) {
            cv::Point2f center(detections.detections[i].bbox.center.x, detections.detections[i].bbox.center.y);
            cv::Size2f size(detections.detections[i].bbox.size_x, detections.detections[i].bbox.size_y);
            cv::RotatedRect rect(center, size, 0);
            cv::Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++)
                cv::line(im, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));
        }
        cv_bridge::CvImage cvImage;
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::RGB8;
        cvImage.header.stamp = t;
        bag_out.write("/camera/image_raw", ros::Time(t), cvImage.toImageMsg());
        t += d;
        cout << i << " / " << image_filenames.size() << endl;
    }

    bag_out.close();

    ros::shutdown();

    return 0;
}
