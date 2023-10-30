# BagFromImages

ROS package to generate a rosbag from a collection of images. Images are ordered alphabetically. The timestamp for each image is assigned according to the specified frequency. 

The bag will publish the images to topic `/camera/image_raw`.

Tested in ROS Noetic.

## Dependencies:

`sudo apt install ros-noetic-vision-msgs`

## Usage:

    rosrun BagFromImages BagFromImages PATH_TO_IMAGES IMAGE_EXTENSION FREQUENCY PATH_TO_OUPUT_BAG
  
 - `PATH_TO_IMAGES`: Path to the folder with the images
 - `IMAGE_EXTENSION`: .jpg, .png, etc. write the dot "."
 - `FREQUENCY`: Frames per second.
 - `PATH_TO_OUTPUT_BAG`: Path to save the bag (including the filename e.g. directory/filename.bag)

```angular2html
/home/qin/Downloads/ForDistanceEstimation/Frames png 5 /home/qin/Downloads/ForDistanceEstimation/out_without_boxes_5FPS.bag
/home/qin/Downloads/ForDistanceEstimation/20232018/02_frames png 10 /home/qin/Downloads/ForDistanceEstimation/out_without_boxes_10FPS.bag
```