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
#include <opencv2/opencv.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <cmath>

namespace apriltags_ros{

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
  it_(nh),
  enabled_(true),
  decimate_rate_(3),
  decimate_count_(0),
  plane_model_distance_threshold_(0.01),
  plane_inlier_threshold_(0.7f),
  plane_angle_threshold_(0.0872665f),
  publish_plane_cloud_(false)
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

  pnh.param<int>("decimate_rate", decimate_rate_, 3);

  pnh.param<bool>("publish_plane_cloud", publish_plane_cloud_, false);

  pnh.param<float>("plane_model_distance_threshold", plane_model_distance_threshold_, 0.01f);

  pnh.param<float>("plane_inlier_threshold", plane_inlier_threshold_, 0.7f);
  plane_inlier_threshold_ = std::max(0.0f, std::min(1.0f, plane_inlier_threshold_));

  pnh.param<float>("plane_angle_threshold", plane_angle_threshold_,0.0872665f);
  plane_angle_threshold_ = std::max(0.0f, std::min(90.0f, plane_angle_threshold_));

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  // Read parameters
  int queue_size = 100;
  pnh.param("queue_size", queue_size, 100);

  ROS_INFO("April tag info: start_enabled: %d, publish_plane_cloud: %d, plane_inlier_threshold: %f",
    enabled_,
    publish_plane_cloud_,
    plane_inlier_threshold_);

  enable_sub_ = nh.subscribe("enable", 1, &AprilTagDetector::enableCb, this);
  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));

  rgb_it_.reset( new image_transport::ImageTransport(nh) );

  sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_point_cloud_, sub_rgb_, sub_info_) );
  sync_->registerCallback(boost::bind(&AprilTagDetector::imageCb, this, _1, _2, _3));

  sub_rgb_.subscribe(*rgb_it_, "image_rect", 5);
  sub_info_.subscribe(nh, "camera_info", 5);

  sub_point_cloud_.subscribe(nh, "cloud_rect", 5);

  image_pub_ = it_.advertise("tag_detections_image", 1);
  plane_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1);

  detections_pub_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tag_detections_pose", 1);
  plane_pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("plane_poses", 1);
}

AprilTagDetector::~AprilTagDetector(){

}

void AprilTagDetector::enableCb(const std_msgs::Bool& msg) {
  enabled_ = msg.data;

  ROS_INFO("April tag enabled: %d", enabled_);
}

double absoluteAngleDiff(double angleA, double angleB)
{
  return M_PI - std::abs(M_PI - std::abs((angleA - angleB)));
}

void AprilTagDetector::imageCb(const sensor_msgs::PointCloud2ConstPtr& cloud,
  const sensor_msgs::ImageConstPtr& rgb_msg_in,
  const sensor_msgs::CameraInfoConstPtr& cam_info) {
  // Check for trigger / timing
  if (!enabled_) {
    ROS_DEBUG_THROTTLE(5.0, "April images received but not enabled.");

    return;
  }

  ROS_DEBUG_THROTTLE(5.0, "April images received.");

  if ((decimate_count_++ % decimate_rate_) == 0)
  {
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
    std::vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(gray);
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
    geometry_msgs::PoseArray plane_pose_array;
    plane_pose_array.header = header;

    BOOST_FOREACH(AprilTags::TagDetection detection, detections) {
      std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);

      if(description_itr == descriptions_.end()){
        ROS_INFO_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
        continue;
      }

      AprilTagDescription description = description_itr->second;
      double tag_size = description.size();

      ROS_DEBUG_THROTTLE(5.0, "April Tag detected in rect: %f, %f - %f, %f - %f, %f - %f, %f", detection.p[0].first, detection.p[0].second,
        detection.p[1].first, detection.p[1].second, detection.p[2].first, detection.p[2].second,
        detection.p[3].first, detection.p[3].second);

      Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);

      detection.draw(cv_ptr->image);

      Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
      Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);
      rot_quaternion.normalize();

      geometry_msgs::Pose pose;
      pose.position.x = transform(0, 3);
      pose.position.y = transform(1, 3);
      pose.position.z = transform(2, 3);

      pose.orientation.x = rot_quaternion.x();
      pose.orientation.y = rot_quaternion.y();
      pose.orientation.z = rot_quaternion.z();
      pose.orientation.w = rot_quaternion.w();

      // Align the x axis to the detected plane for the purposes of alignment and visualization
      tf::Vector3 xAxis(rot(0,0), rot(1,0), rot(2,0));

      tf::Transform planeTransform = getDepthImagePlaneTransform(cloud, detection.p, detection, xAxis);

      tf::Matrix3x3 aprilTagRotation;
      tf::matrixEigenToTF(rot, aprilTagRotation);

      double aprilTagRoll, aprilTagPitch, aprilTagYaw;
      aprilTagRotation.getRPY(aprilTagRoll, aprilTagPitch, aprilTagYaw);

      double planeRoll, planePitch, planeYaw;
      planeTransform.getBasis().getRPY(planeRoll, planePitch, planeYaw);

      double diffRoll = absoluteAngleDiff(aprilTagRoll, planeRoll);
      double diffPitch = absoluteAngleDiff(aprilTagPitch, planePitch);

      bool validPose = true;

      // The maximum allowed angle delta for each axis
      if ((diffRoll > plane_angle_threshold_) || (diffPitch > plane_angle_threshold_))
      {
        ROS_DEBUG_THROTTLE(5.0, "April tag and plane poses do not match!");

        ROS_DEBUG_THROTTLE(5.0, "April angle: %f, %f", aprilTagRoll, aprilTagPitch);
        ROS_DEBUG_THROTTLE(5.0, "Plane angle: %f, %f", planeRoll, planePitch);
        ROS_DEBUG_THROTTLE(5.0, "Diff: %f, %f", diffRoll, diffPitch);

        validPose = false;
      }

      geometry_msgs::Pose planePose;
      tf::poseTFToMsg(planeTransform, planePose);

      // Align the origin of the detected plane with the position of the april tag detection
      planePose.position = pose.position;

      if (transform_output) {

        tf::Transform untransformedPose;
        tf::Transform untransformedPlanePose;
        ROS_DEBUG("output transformer: %f, %f, %f ... %f, %f, %f, %f",
          output_transform.getOrigin().getX(),
          output_transform.getOrigin().getY(),
          output_transform.getOrigin().getZ(),
          output_transform.getRotation().getX(),
          output_transform.getRotation().getY(),
          output_transform.getRotation().getZ(),
          output_transform.getRotation().getW());
        tf::poseMsgToTF(pose, untransformedPose);
        tf::poseMsgToTF(planePose, untransformedPlanePose);
        ROS_DEBUG("Untransformed: %f, %f, %f ... %f, %f, %f, %f",
          untransformedPose.getOrigin().getX(),
          untransformedPose.getOrigin().getY(),
          untransformedPose.getOrigin().getZ(),
          untransformedPose.getRotation().getX(),
          untransformedPose.getRotation().getY(),
          untransformedPose.getRotation().getZ(),
          untransformedPose.getRotation().getW());
        tf::Transform transformedPose = output_transform * untransformedPose;
        tf::Transform transformedPlanePose = output_transform * untransformedPlanePose;
        ROS_DEBUG("transformed: %f, %f, %f ... %f, %f, %f, %f",
          transformedPose.getOrigin().getX(),
          transformedPose.getOrigin().getY(),
          transformedPose.getOrigin().getZ(),
          transformedPose.getRotation().getX(),
          transformedPose.getRotation().getY(),
          transformedPose.getRotation().getZ(),
          transformedPose.getRotation().getW());
        tf::poseTFToMsg(transformedPose, pose);
        tf::poseTFToMsg(transformedPlanePose, planePose);
      }

      geometry_msgs::PoseStamped tag_pose;
      tag_pose.pose = pose;
      tag_pose.header = header;

      if (validPose)
      {
        AprilTagDetection tag_detection;
        tag_detection.pose = tag_pose;
        tag_detection.id = detection.id;
        tag_detection.size = tag_size;
        tag_detection_array.detections.push_back(tag_detection);

        tf::Stamped<tf::Transform> tag_transform;
        tf::poseStampedMsgToTF(tag_pose, tag_transform);
        tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));
      }

      // Publish both poses either way to debug/visualise
      tag_pose_array.poses.push_back(tag_pose.pose);
      plane_pose_array.poses.push_back(planePose);
   }
    detections_pub_.publish(tag_detection_array);
    pose_pub_.publish(tag_pose_array);
    plane_pose_pub_.publish(plane_pose_array);
    image_pub_.publish(cv_ptr->toImageMsg());
  }
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

tf::Transform getPlaneTransform(pcl::ModelCoefficients coeffs, tf::Vector3 xAxis)
{
  ROS_ASSERT(coeffs.values.size() > 3);

  double a = coeffs.values[0];
  double b = coeffs.values[1];
  double c = coeffs.values[2];
  double d = coeffs.values[3];

  // Assume plane coefficients are normalized
  tf::Vector3 position(-a*d, -b*d, -c*d);
  tf::Vector3 zAxis(a, b, c);

  // Make sure z points "up"
  if (zAxis.dot( tf::Vector3(0, 0, -1)) < 0)
  {
    zAxis = -1.0 * zAxis;
  }

  tf::Vector3 yAxis = zAxis.cross(xAxis).normalized();

  tf::Matrix3x3 orientation;
  orientation[0] = xAxis;
  orientation[1] = yAxis;
  orientation[2] = zAxis;

  tf::Quaternion orientationRos;
  orientation.transpose().getRotation(orientationRos);

  return tf::Transform(orientationRos, position);
}

tf::Transform AprilTagDetector::getDepthImagePlaneTransform(const sensor_msgs::PointCloud2ConstPtr& cloud,
  std::pair<float,float> polygon[4], AprilTags::TagDetection& detection, tf::Vector3 xAxisVector)
{
  tf::Transform transform = tf::Transform::getIdentity();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pointCloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr polygonInliers(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeInliers(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointIndices::Ptr polygonInlierIndices(new pcl::PointIndices());

  pcl::PointCloud<pcl::PointXYZ> clipPolygon;
  clipPolygon.push_back(pcl::PointXYZ(polygon[0].first, polygon[0].second, 0));
  clipPolygon.push_back(pcl::PointXYZ(polygon[1].first, polygon[1].second, 0));
  clipPolygon.push_back(pcl::PointXYZ(polygon[2].first, polygon[2].second, 0));
  clipPolygon.push_back(pcl::PointXYZ(polygon[3].first, polygon[3].second, 0));
  clipPolygon.push_back(pcl::PointXYZ(polygon[0].first, polygon[0].second, 0));

  for (int x = 0; x < pointCloud->width; x++)
  {
    for (int y = 0; y < pointCloud->height; y++)
    {
        int i = x + (pointCloud->width * y);

        pcl::PointXYZ point(x, y, 0 );

        if (pcl::isXYPointIn2DXYPolygon<pcl::PointXYZ>(point, clipPolygon))
        {
          polygonInlierIndices->indices.push_back(i);
        }
    }
  }

  ROS_DEBUG_THROTTLE(5.0, "Points in detection polygon: %zu", polygonInlierIndices->indices.size());

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(pointCloud);
  extract.setIndices(polygonInlierIndices);
  extract.setNegative(false);
  extract.filter(*polygonInliers);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  // Optional
  seg.setOptimizeCoefficients(true);

  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(plane_model_distance_threshold_);

  pcl::PointIndices::Ptr planeInlierIndices(new pcl::PointIndices());

  seg.setInputCloud(polygonInliers);
  seg.segment(*planeInlierIndices, *coefficients);

  if (planeInlierIndices->indices.size () == 0)
  {
    ROS_DEBUG("Could not estimate a planar model for the given dataset.");
  }
  else
  {
    size_t numIndices = planeInlierIndices->indices.size();

    ROS_DEBUG_THROTTLE(5.0, "Points in plane: %zu out of %zu", numIndices, polygonInliers->size());

    if (publish_plane_cloud_)
    {
      // Extract the inliers
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(polygonInliers);
      extract.setIndices(planeInlierIndices);
      extract.setNegative(false);
      extract.filter(*planeInliers);

      sensor_msgs::PointCloud2 planeInliersRos;
      pcl::toROSMsg(*planeInliers, planeInliersRos);
      plane_cloud_pub_.publish(planeInliersRos);
    }

    // Pick a threshold
    if (numIndices > (polygonInliers->size() * plane_inlier_threshold_))
    {
      transform = getPlaneTransform(*coefficients, xAxisVector);
    }
  }

  return transform;
}

}
