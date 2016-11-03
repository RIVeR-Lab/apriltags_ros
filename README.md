apriltags_ros  [![Build Status](https://api.travis-ci.org/RIVeR-Lab/apriltags_ros.png)](https://travis-ci.org/RIVeR-Lab/apriltags_ros)
=============

AprilTags for ROS

The pre-requisites for using this package are the following:

1. usb_cam (http://wiki.ros.org/usb_cam) package and apriltag_ros package should be installed

2. Create a launch file similar to example.launch in apriltag_ros package (you will find that in the apriltag_ros/launch folder. In the launch file mention the tag ids you want to detect and their size in meter.

3. Calibrate your camera and import the yaml file. Copy that yaml file with appropriate camera name in the home/user/.ros/camera_info folder.

4. roslaunch apriltag_ros [launchfilename].launch
