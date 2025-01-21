#include "handeye_calibration.h"

HandeyeCalibration::HandeyeCalibration() : Node("handeye_calibration")
{
  declare_parameter("base_frame", "base_link");
  declare_parameter("ee_frame", "ee_link");
  declare_parameter("camera_frame", "camera_link");

  get_parameter("base_frame", base_frame_);
  get_parameter("ee_frame", ee_frame_);
  get_parameter("camera_frame", camera_frame_);

  RCLCPP_INFO(this->get_logger(), "base frame: %s, ee frame: %s, camera frame: %s", base_frame_.c_str(), ee_frame_.c_str(), camera_frame_.c_str());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  collect_service_ = this->create_service<std_srvs::srv::Trigger>(
      "collect_sample", std::bind(&HandeyeCalibration::collectSampleCallback, this, std::placeholders::_1, std::placeholders::_2));

  calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
      "clean_sample", std::bind(&HandeyeCalibration::cleanSampleCallback, this, std::placeholders::_1, std::placeholders::_2));

  ee_tfs_.clear();
  target_tfs_.clear();

  RCLCPP_INFO(this->get_logger(), "Handeye Calibration Node Started");
}

void HandeyeCalibration::collectSampleCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  // eye in hand
  auto ee_tf = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
  auto target_tf = tf_buffer_->lookupTransform(camera_frame_, "handeye_target", tf2::TimePointZero);

  ee_tfs_.push_back(ee_tf);
  target_tfs_.push_back(target_tf);

  RCLCPP_INFO(this->get_logger(), "Sample collected size: %ld", ee_tfs_.size());
  if (ee_tfs_.size() > 5)
    calibrateHandeye();

  res->success = true;
  res->message = "Sample collected";
}

void HandeyeCalibration::cleanSampleCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Sample cleaned");
  ee_tfs_.clear();
  target_tfs_.clear();
  res->success = true;
  res->message = "Sample cleaned";
}

Eigen::Isometry3d HandeyeCalibration::tf2eigen(const geometry_msgs::msg::TransformStamped &tf)
{
  geometry_msgs::msg::Transform transform = tf.transform;
  Eigen::Isometry3d isometry3d = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q;
  q.w() = transform.rotation.w;
  q.x() = transform.rotation.x;
  q.y() = transform.rotation.y;
  q.z() = transform.rotation.z;

  Eigen::Vector3d t(transform.translation.x, transform.translation.y, transform.translation.z);

  isometry3d.linear() = q.toRotationMatrix();
  isometry3d.translation() = t;

  return isometry3d;
}

void HandeyeCalibration::calibrateHandeye()
{
  if (ee_tfs_.size() < 5 || ee_tfs_.size() != target_tfs_.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Need atleast 5 samples to calibrate or Size mismatch");
    return;
  }

  std::vector<cv::Mat> r_base2ee, t_base2ee, r_camera2target, t_camera2target;
  std::vector<Eigen::Isometry3d> eigen_base2ee;
  std::vector<Eigen::Isometry3d> eigen_camera2target;
  for (size_t i = 0; i < ee_tfs_.size(); i++)
  {
    Eigen::Isometry3d ee_eigen, target_eigen;
    ee_eigen = tf2eigen(ee_tfs_[i]);
    target_eigen = tf2eigen(target_tfs_[i]);

    eigen_base2ee.push_back(ee_eigen);
    eigen_camera2target.push_back(target_eigen);

    Eigen::Matrix3d ee_rot = ee_eigen.rotation();
    cv::Mat r_ee(3, 3, CV_64F);
    cv::eigen2cv(ee_rot, r_ee);
    Eigen::Vector3d ee_trans = ee_eigen.translation();
    cv::Mat t_ee(3, 1, CV_64F);
    cv::eigen2cv(ee_trans, t_ee);

    Eigen::Matrix3d target_rot = target_eigen.rotation();
    cv::Mat r_target(3, 3, CV_64F);
    cv::eigen2cv(target_rot, r_target);
    Eigen::Vector3d target_trans = target_eigen.translation();
    cv::Mat t_target(3, 1, CV_64F);
    cv::eigen2cv(target_trans, t_target);

    r_base2ee.push_back(r_ee);
    t_base2ee.push_back(t_ee);
    r_camera2target.push_back(r_target);
    t_camera2target.push_back(t_target);
  }

  cv::Mat r_ee2camera, t_ee2camera;
  cv::calibrateHandEye(r_base2ee, t_base2ee, r_camera2target, t_camera2target, r_ee2camera, t_ee2camera, cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_DANIILIDIS);

  Eigen::Matrix3d eigen_r_ee2camera;
  cv::cv2eigen(r_ee2camera, eigen_r_ee2camera);
  Eigen::Vector3d rpy = eigen_r_ee2camera.eulerAngles(2, 1, 0);

  RCLCPP_INFO(this->get_logger(), "hand in eye calibration result: %s to %s", ee_frame_.c_str(), camera_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "RPY: %f %f %f", rpy[0], rpy[1], rpy[2]);
  RCLCPP_INFO(this->get_logger(), "Translation: %f %f %f", t_ee2camera.at<double>(0), t_ee2camera.at<double>(1), t_ee2camera.at<double>(2));

  size_t num_motions = r_base2ee.size() - 1;
  double rotation_err = 0.0;
  double translation_err = 0.0;

  Eigen::Matrix3d x_rotation;
  Eigen::Vector3d x_translation;
  cv::cv2eigen(r_ee2camera, x_rotation);
  cv::cv2eigen(t_ee2camera, x_translation);

  Eigen::Isometry3d X = Eigen::Isometry3d::Identity();
  X.linear() = x_rotation;
  X.translation() = x_translation;
  for (size_t i = 0; i < num_motions; ++i)
  {
    Eigen::Isometry3d A = eigen_base2ee[i].inverse() * eigen_base2ee[i + 1];
    Eigen::Isometry3d B = eigen_camera2target[i] * eigen_camera2target[i + 1].inverse();

    Eigen::Isometry3d AX = A * X;
    Eigen::Isometry3d XB = X * B;
    AX.linear().normalize();
    XB.linear().normalize();

    double r_err = Eigen::AngleAxisd(AX.rotation().transpose() * XB.rotation()).angle();
    rotation_err += r_err * r_err;

    double t_err = ((AX.translation() - XB.translation()).norm() +
                    (AX.inverse().translation() - XB.inverse().translation()).norm()) /
                   2.;

    translation_err += t_err * t_err;
  }

  RCLCPP_INFO(this->get_logger(), "ReprojectionError: %f m %f rad", std::sqrt(translation_err / num_motions), std::sqrt(rotation_err / num_motions));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto handeye_calibration = std::make_shared<HandeyeCalibration>();
  rclcpp::spin(handeye_calibration);
  handeye_calibration.reset();
  rclcpp::shutdown();
  return 0;
}
