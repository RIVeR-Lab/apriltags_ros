#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <AprilTags/TagDetector.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

namespace apriltags_ros{

using namespace message_filters::sync_policies;

class AprilTagDescription{
 public:
  AprilTagDescription(int id, double size, std::string &frame_name):id_(id), size_(size), frame_name_(frame_name){}
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
  void enableCb(const std_msgs::Bool& msg);
  void imageCb(const sensor_msgs::PointCloud2ConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg_in,
		const sensor_msgs::CameraInfoConstPtr& info_msg);
  std::map<int, AprilTagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);
  bool getTransform(std::string t1, std::string t2, tf::Transform& output);

	tf::Transform getDepthImagePlaneTransform(const sensor_msgs::PointCloud2ConstPtr& cloud,
		std::pair<float,float> polygon[4], AprilTags::TagDetection& detection, tf::Vector3 x);

 private:
  std::map<int, AprilTagDescription> descriptions_;
  std::string sensor_frame_id_;

	boost::shared_ptr<image_transport::ImageTransport> rgb_it_;

  // Subscriptions
  ros::NodeHandlePtr rgb_nh_;
  image_transport::ImageTransport it_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point_cloud_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;

  image_transport::SubscriberFilter sub_rgb_;

  typedef ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher cloud_original_pub_;
  ros::Publisher cloud_plane_inlier_pub_;
  ros::Publisher detections_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher plane_pose_pub_;
  ros::Subscriber enable_sub_;
  tf::TransformBroadcaster tf_pub_;
  boost::shared_ptr<AprilTags::TagDetector> tag_detector_;
  bool projected_optics_;
  bool enabled_;

  tf::TransformListener tf_listener_;
  std::string output_frame_id_;
};

}


#endif
