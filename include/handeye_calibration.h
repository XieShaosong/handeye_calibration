#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

class HandeyeCalibration : public rclcpp::Node
{
public:
  HandeyeCalibration();

private:
  void collectSampleCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void cleanSampleCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

private:
  Eigen::Isometry3d tf2eigen(const geometry_msgs::msg::TransformStamped& tf);
  void calibrateHandeye();

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr collect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;

private:
  std::vector<geometry_msgs::msg::TransformStamped> ee_tfs_;
  std::vector<geometry_msgs::msg::TransformStamped> target_tfs_;
  std::string base_frame_;
  std::string ee_frame_;
  std::string camera_frame_;
};
