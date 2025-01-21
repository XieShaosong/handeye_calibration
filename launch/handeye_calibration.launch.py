import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  config_yaml = os.path.join(get_package_share_directory('handeye_calibration'), 'config', 'handeye_calibration.yaml')

  charuco_detection_node = Node(package='handeye_calibration',
                             executable='charuco_detection',
                             name='charuco_detection',
                             parameters=[config_yaml],
                             output='screen')

  handeye_calibration_node = Node(package='handeye_calibration',
                                  executable='handeye_calibration',
                                  name='handeye_calibration',
                                  parameters=[config_yaml],
                                  output='screen')

  nodes_to_start = [charuco_detection_node, handeye_calibration_node]
  return LaunchDescription(nodes_to_start)
