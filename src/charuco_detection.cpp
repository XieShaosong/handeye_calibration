#include "charuco_detection.h"

CharucoDetection::CharucoDetection() : Node("charuco_detection")
{
  RCLCPP_INFO(this->get_logger(), "CharucoDetection ready.");
  declare_parameter("image_topic", "/camera/camera/color/image_raw");
  declare_parameter("camera_info_topic", "/camera/camera/color/camera_info");

  get_parameter("image_topic", image_topic_);
  get_parameter("camera_info_topic", camera_info_topic_);
  RCLCPP_INFO(this->get_logger(), "image_topic: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "camera_info_topic: %s", camera_info_topic_.c_str());
}

CharucoDetection::~CharucoDetection()
{
  stopCharucoDetection();
}

void CharucoDetection::startCharucoDetection()
{
  image_transport_ptr_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  camera_image_sub_ = image_transport_ptr_->subscribe(image_topic_, 10, std::bind(&CharucoDetection::cameraImageCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10, std::bind(&CharucoDetection::cameraInfoCallback, this, std::placeholders::_1));

  tracking_image_pub_ = image_transport_ptr_->advertise("/tracking_image", 1);

  grid_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/grid_pose", 1);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(this->get_logger(), "CharucoDetection started.");
}

void CharucoDetection::stopCharucoDetection()
{
  camera_image_sub_.shutdown();
  camera_info_sub_.reset();
  tracking_image_pub_.shutdown();
  grid_pose_pub_.reset();
  tf_broadcaster_.reset();
  RCLCPP_INFO(this->get_logger(), "CharucoDetection stopped.");
}

void CharucoDetection::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  camera_info_ = msg;
}

void CharucoDetection::cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  detectAndGenerateChArUco(cv_ptr->image, msg->header.frame_id);
}

void CharucoDetection::detectAndGenerateChArUco(const cv::Mat image, const std::string camera_frame)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Detecting ChArUco board...");
  vector<vector<cv::Point2f>> marker_corners;
  vector<int> marker_ids;
  cv::Mat image_copy;
  image.copyTo(image_copy);
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
  cv::Ptr<cv::aruco::CharucoBoard> charucoboard = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.03f, dictionary);
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, params);

  if (marker_corners.size() < 10)
    return;

  cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i)
    camera_matrix.at<double>(i / 3, i % 3) = camera_info_->k[i];

  std::vector<double> camera_dist = camera_info_->d;
  cv::Mat dist_coeffs = cv::Mat::zeros(camera_dist.size(), 1, CV_64F);
  for (size_t i = 0; i < camera_dist.size(); ++i)
    dist_coeffs.at<double>(i, 0) = camera_dist[i];

  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
  cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image_copy, charucoboard, charuco_corners, charuco_ids, camera_matrix, dist_coeffs);
  if (charuco_corners.size() > 0)
  {
    cv::Vec3d rotation_vec, translation_vec;
    bool valid = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, charucoboard, camera_matrix, dist_coeffs, rotation_vec, translation_vec);
    if (valid)
    {
      cv::Mat rotation_matrix;
      cv::Rodrigues(rotation_vec, rotation_matrix);
      Eigen::Matrix3d eigen_rotation_matrix;
      cv::cv2eigen(rotation_matrix, eigen_rotation_matrix);
      Eigen::Quaterniond q(eigen_rotation_matrix);

      geometry_msgs::msg::Pose pose;
      pose.position.x = translation_vec(0);
      pose.position.y = translation_vec(1);
      pose.position.z = translation_vec(2);
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      grid_pose_pub_->publish(pose);

      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header = std_msgs::msg::Header();
      transform_stamped.header.frame_id = camera_frame;
      transform_stamped.child_frame_id = "handeye_target";
      transform_stamped.transform.translation.x = pose.position.x;
      transform_stamped.transform.translation.y = pose.position.y;
      transform_stamped.transform.translation.z = pose.position.z;
      transform_stamped.transform.rotation = pose.orientation;
      tf_broadcaster_->sendTransform(transform_stamped);

      RCLCPP_INFO_ONCE(this->get_logger(), "tf: frame_id: %s, child_frame_id: %s", camera_frame.c_str(), "handeye_target");

      cv::drawFrameAxes(image_copy, camera_matrix, dist_coeffs, rotation_vec, translation_vec, 0.1f);
      cv::aruco::drawDetectedMarkers(image_copy, marker_corners, marker_ids);

      std::shared_ptr<sensor_msgs::msg::Image> image_detected = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_copy).toImageMsg();
      tracking_image_pub_.publish(*image_detected);
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto charuco_detection = std::make_shared<CharucoDetection>();

  rclcpp::on_shutdown([charuco_detection]()
                      { charuco_detection->stopCharucoDetection(); });

  charuco_detection->startCharucoDetection();
  rclcpp::spin(charuco_detection);
  charuco_detection.reset();
  rclcpp::shutdown();
  return 0;
}
