#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <depth_image_proc/depth_traits.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace apriltags_ros{

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
 	it_(nh),
 	enabled_(true)
{
  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
    }
  }

  if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
    sensor_frame_id_ = "";
  }

  if(!pnh.getParam("output_frame_id", output_frame_id_)){
    output_frame_id_ = "";
  }

  std::string tag_family;
  pnh.param<std::string>("tag_family", tag_family, "36h11");

  pnh.param<bool>("projected_optics", projected_optics_, false);

  const AprilTags::TagCodes* tag_codes;
  if(tag_family == "16h5"){
    tag_codes = &AprilTags::tagCodes16h5;
  }
  else if(tag_family == "25h7"){
    tag_codes = &AprilTags::tagCodes25h7;
  }
  else if(tag_family == "25h9"){
    tag_codes = &AprilTags::tagCodes25h9;
  }
  else if(tag_family == "36h9"){
    tag_codes = &AprilTags::tagCodes36h9;
  }
  else if(tag_family == "36h11"){
    tag_codes = &AprilTags::tagCodes36h11;
  }
  else{
    ROS_WARN("Invalid tag family specified; defaulting to 36h11");
    tag_codes = &AprilTags::tagCodes36h11;
  }

  pnh.param<bool>("start_enabled", enabled_, false);

	// Read parameters
  int queue_size = 5;
  pnh.param("queue_size", queue_size, 5);

  enable_sub_ = pnh.subscribe("enable", 1, &AprilTagDetector::enableCb, this);
  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));

  rgb_it_.reset( new image_transport::ImageTransport(nh) );

	sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_point_cloud_, sub_rgb_, sub_info_) );
	sync_->registerCallback(boost::bind(&AprilTagDetector::imageCb, this, _1, _2, _3));

	sub_rgb_.subscribe(*rgb_it_, "image_rect", 5);
	sub_info_.subscribe(nh, "camera_info", 5);

	sub_point_cloud_.subscribe(nh, "cloud_rect", 5);

  ROS_ERROR("Registering image_rect, camera_info, and cloud_rect (queue_size: %d)", queue_size);

  image_pub_ = it_.advertise("tag_detections_image", 1);
  detections_pub_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tag_detections_pose", 1);
}
AprilTagDetector::~AprilTagDetector(){

}

void AprilTagDetector::enableCb(const std_msgs::Bool& msg) {
  enabled_ = msg.data;
}

void AprilTagDetector::imageCb(const sensor_msgs::PointCloud2ConstPtr& cloud,
	const sensor_msgs::ImageConstPtr& rgb_msg_in,
	const sensor_msgs::CameraInfoConstPtr& cam_info) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud, *pointCloud);

//  // Check for trigger / timing
//  if (!enabled_) {
//    return;
//  }

	// Check for bad inputs
  if (cloud->header.frame_id != rgb_msg_in->header.frame_id)
  {
    ROS_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
			cloud->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(rgb_msg_in, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray);
  ROS_DEBUG("%d tag detected", (int)detections.size());

  double fx;
  double fy;
  double px;
  double py;
  if (projected_optics_) {
    // use projected focal length and principal point
    // these are the correct values
    fx = cam_info->P[0];
    fy = cam_info->P[5];
    px = cam_info->P[2];
    py = cam_info->P[6];
  } else {
    // use camera intrinsic focal length and principal point
    // for backwards compatability
    fx = cam_info->K[0];
    fy = cam_info->K[4];
    px = cam_info->K[2];
    py = cam_info->K[5];
  }

  if(!sensor_frame_id_.empty()) {
    cv_ptr->header.frame_id = sensor_frame_id_;
  }

  std_msgs::Header header = cv_ptr->header;
  bool transform_output = false;
  tf::Transform output_transform;
  if (!output_frame_id_.empty()) {
    if (getTransform(output_frame_id_, cv_ptr->header.frame_id, output_transform)) {
      transform_output = true;
      header.frame_id = output_frame_id_;
    } else {
      ROS_WARN_THROTTLE(10.0, "Could not get transform to specified frame %s.", output_frame_id_.c_str());
      return;
    }
  }

  AprilTagDetectionArray tag_detection_array;
  geometry_msgs::PoseArray tag_pose_array;
  tag_pose_array.header = header;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections) {
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);

    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }

    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

		ROS_DEBUG("April Tag detected in rect: %f, %f - %f, %f - %f, %f - %f, %f", detection.p[0].first, detection.p[0].second,
			detection.p[1].first, detection.p[1].second, detection.p[2].first, detection.p[2].second,
			detection.p[3].first, detection.p[3].second);

    detection.draw(cv_ptr->image);

    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;

		// Optional
		seg.setOptimizeCoefficients (true);

		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);

		seg.setInputCloud (pointCloud);
		seg.segment (*inliers, *coefficients);

		if (inliers->indices.size () == 0)
		{
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		}
		else
		{
			std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    																				<< coefficients->values[1] << " "
    																				<< coefficients->values[2] << " "
    																				<< coefficients->values[3] << std::endl;

			std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
			for (size_t i = 0; i < inliers->indices.size (); ++i)
			{
				std::cerr << inliers->indices[i] << "    " << pointCloud->points[inliers->indices[i]].x << " "
																									 << pointCloud->points[inliers->indices[i]].y << " "
																									 << pointCloud->points[inliers->indices[i]].z << std::endl;
			}
		}

    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    geometry_msgs::Pose pose;
    pose.position.x = transform(0, 3);
    pose.position.y = transform(1, 3);
    pose.position.z = transform(2, 3);
    pose.orientation.x = rot_quaternion.x();
    pose.orientation.y = rot_quaternion.y();
    pose.orientation.z = rot_quaternion.z();
    pose.orientation.w = rot_quaternion.w();

    if (transform_output) {

      tf::Transform untransformedPose;
      ROS_DEBUG("output transformer: %f, %f, %f ... %f, %f, %f, %f",
        output_transform.getOrigin().getX(),
        output_transform.getOrigin().getY(),
        output_transform.getOrigin().getZ(),
        output_transform.getRotation().getX(),
        output_transform.getRotation().getY(),
        output_transform.getRotation().getZ(),
        output_transform.getRotation().getW());
      tf::poseMsgToTF(pose, untransformedPose);
      ROS_DEBUG("Untransformed: %f, %f, %f ... %f, %f, %f, %f",
        untransformedPose.getOrigin().getX(),
        untransformedPose.getOrigin().getY(),
        untransformedPose.getOrigin().getZ(),
        untransformedPose.getRotation().getX(),
        untransformedPose.getRotation().getY(),
        untransformedPose.getRotation().getZ(),
        untransformedPose.getRotation().getW());
      tf::Transform transformedPose = output_transform * untransformedPose;
      ROS_DEBUG("transformed: %f, %f, %f ... %f, %f, %f, %f",
        transformedPose.getOrigin().getX(),
        transformedPose.getOrigin().getY(),
        transformedPose.getOrigin().getZ(),
        transformedPose.getRotation().getX(),
        transformedPose.getRotation().getY(),
        transformedPose.getRotation().getZ(),
        transformedPose.getRotation().getW());
      tf::poseTFToMsg(transformedPose, pose);
    }

    geometry_msgs::PoseStamped tag_pose;
    tag_pose.pose = pose;
    tag_pose.header = header;

    AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id = detection.id;
    tag_detection.size = tag_size;
    tag_detection_array.detections.push_back(tag_detection);
    tag_pose_array.poses.push_back(tag_pose.pose);

    tf::Stamped<tf::Transform> tag_transform;
    tf::poseStampedMsgToTF(tag_pose, tag_transform);
    tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));
  }
  detections_pub_.publish(tag_detection_array);
  pose_pub_.publish(tag_pose_array);
  image_pub_.publish(cv_ptr->toImageMsg());
}

std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}

bool AprilTagDetector::getTransform(std::string t1, std::string t2, tf::Transform& output) {
  try {
    tf::StampedTransform robotTransform;

    if (tf_listener_.canTransform(t1, t2,
        ros::Time(0))) {
      tf::StampedTransform robotTransform;
      tf_listener_.lookupTransform(t1, t2,
          ros::Time(0), robotTransform);

      output = robotTransform;
      return true;
    } else {
      ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, "apriltag_transform", "Could not lookup transform from" << t1 << " to " << t2 << ".");
      return false;
    }
  } catch(const tf::TransformException& e) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, "apriltag_transform", "TF Exception: " << e.what());
      return false;
  }
}

// Draw Point Cloud
inline void AprilTagDetector::createMat(pcl::PointCloud < pcl::PointXYZRGB >& cloud, cv::Mat& mat)
{
	mat = cv::Mat(3, cloud.points.size(), CV_64FC1);

	for( int i=0; i < cloud.points.size(); i++)
	{
		mat.at<double>(0,i) = cloud.points.at(i).x;
		mat.at<double>(1,i) = cloud.points.at(i).y;
		mat.at<double>(2,i) = cloud.points.at(i).z;
	}
}

int AprilTagDetector::PointInPoly(int size, std::pair<float,float> polygon[], float x, float y)
{
  int i, j, c = 0;

  for (i = 0, j = size - 1; i < size; j = i++)
	{
		if ( ((polygon[i].second > y) != (polygon[j].second > y)) &&
			(x < (polygon[j].first-polygon[i].first) * (y-polygon[i].second) / (polygon[j].second-polygon[i].second) + polygon[i].first) ) {
			c = !c;
		}
  }

  return c;
}

}

//Eigen::Matrix4d AprilTagDetector::getDepthImagePlaneTransform(const sensor_msgs::PointCloud2& cloud,
//	std::pair<float,float> polygon[4])
//
//	// Crop the point cloud to the bounding box of the image
//
//	return transform;
//}
