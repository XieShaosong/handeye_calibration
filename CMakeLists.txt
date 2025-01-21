cmake_minimum_required(VERSION 3.8)
project(handeye_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS} 
)

add_executable(charuco_detection src/charuco_detection.cpp)

ament_target_dependencies(charuco_detection
  rclcpp 
  Eigen3 
  OpenCV 
  cv_bridge 
  std_srvs
  geometry_msgs
  sensor_msgs
  image_transport
  tf2_ros
)

add_executable(handeye_calibration src/handeye_calibration.cpp)

ament_target_dependencies(handeye_calibration
  rclcpp 
  Eigen3 
  OpenCV 
  std_srvs
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2
  tf2_geometry_msgs   
)

install(TARGETS charuco_detection handeye_calibration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
