#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <AprilTags/TagDetector.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

namespace apriltags_ros{

typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image,
  sensor_msgs::CameraInfo
> SyncPolicy;


class AprilTagDescription{
 public:
  AprilTagDescription(int id, double size, std::string &frame_name):id_(id), size_(size), frame_name_(frame_name){}
  AprilTagDescription(){}
  double size(){return size_;}
  int id(){return id_;} 
  std::string& frame_name(){return frame_name_;} 
 private:
  int id_;
  double size_;
  std::string frame_name_;
};


class AprilTagDetector{
 public:
  AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~AprilTagDetector();
 private:
  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
  std::map<int, AprilTagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);

 private:
  std::map<int, AprilTagDescription> descriptions_;
  std::string sensor_frame_id_;

  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > cam_info_sub_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > cam_image_sub_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  ros::Publisher image_pub_;

  ros::Publisher detections_pub_;
  ros::Publisher pose_pub_;

  boost::shared_ptr<AprilTags::TagDetector> tag_detector_;
};



}


#endif
