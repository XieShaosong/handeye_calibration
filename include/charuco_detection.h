#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.hpp>

using namespace std;

class CharucoDetection : public rclcpp::Node
{
public:
  CharucoDetection();
  ~CharucoDetection();
  void startCharucoDetection();
  void stopCharucoDetection();

private:
  void cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

private:
  void detectAndGenerateChArUco(const cv::Mat image, const std::string camera_frame);

private:
  std::shared_ptr<image_transport::ImageTransport> image_transport_ptr_;
	image_transport::Subscriber camera_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  image_transport::Publisher tracking_image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr grid_pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

private:
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  string image_topic_;
  string camera_info_topic_;
};
