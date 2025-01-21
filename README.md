# handeye_calibration

## Compatibility

| **Supported OS**          | **Supported ROS2 distribution**                         | **Supported handeye** |
|---------------------------|---------------------------------------------------------|-----------------------|
| Ubuntu 24.04              | [Jazzy](https://docs.ros.org/en/jazzy/index.html)       | eye in hand           |

This project was developed for ROS2 Jazzy on Ubuntu 24.04. Other versions of Ubuntu and ROS2 may work, but not tested.

1. Install [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

2. Install `colcon` and additional ROS package:
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

3. Setup workspace:
    ```bash
    mkdir -p ~/handeye_calibration/src
    cd ~/handeye_calibration/src
    git clone -b main https://github.com/XieShaosong/handeye_calibration.git
    ```

4. Install dependencies:
    ```bash
    cd ~/handeye_calibration
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
    ```

5. Print the charuco board and Setup:
    - Download the charuco board from [here](board/charuco_210x297_7x5_40_30_DICT_5X5.pdf) and print it at its original size. 

    - Set the parameters in `config/handeye_calibration.yaml`:
    ```yaml
    charuco_detection:
      ros__parameters:
        camera_topic: "/camera/camera/color/image_raw"          # image topic
        camera_info_topic: "/camera/camera/color/camera_info"   # camera info topic

    handeye_calibration:
      ros__parameters:
        base_frame: "base_link"                                 # robot base frame
        ee_frame: "tool0"                                       # robot ee frame
        camera_frame: "camera_link"                             # camera base frame
    ```

6. Build and source the workspace:
    ```bash
    cd ~/handeye_calibration
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash
    ```

**NOTE**: Remember to source the setup file and the workspace whenever a new terminal is opened:

```bash
source /opt/ros/jazzy/setup.bash
source ~/handeye_calibration/install/setup.bash
```

## Protocol Description
### Service: collect_sample
Service Type: std_srvs/srv/trigger

### Service: clean_sample
Service Type: std_srvs/srv/trigger

## Usage
1. Start the robot arm and camera:

  ```bash
  ros2 launch <robot_arm_launch_file>
  ros2 launch <camera_launch_file>
  ```

2. Launch the handeye calibration node:

  ```bash
  ros2 launch handeye_calibration handeye_calibration.launch.py
  ```

3. Open `rqt` and enable `image_view` and `service caller` plugins.
  - In `image_view`, subscribe to the `tracking_image` topic.
  - In `service caller`, subscribe to the `collect_sample` service.

4. Move Robot and Collect Samples:
  - Move the robot arm to different poses where the charuco board is clearly visible
  - Ensure the board is within camera's field of view
  - Call the `/collect_sample` service to record current pose:
     ```bash
     ros2 service call /collect_sample std_srvs/srv/Trigger
     ```
  - Repeat for 15-20 different poses
  - Check the tracking image in rqt to ensure proper detection
  - Use `/clean_sample` service to clear collected samples if needed:
     ```bash
     ros2 service call /clean_sample std_srvs/srv/Trigger
     ```

5. Get the transformation from ee_frame to camera_frame:
   - After collecting enough samples, the transformation will be printed in the terminal
   - Format: XYZ and RPY
