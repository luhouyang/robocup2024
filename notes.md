# ROBOCUP


## ros2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

1. Follow steps to install binaries

## gazebo: https://gazebosim.org/docs/latest/ros_installation/

1. RUN if there is any error with gazebo

    ```
    killall gzserver gzclient
    ```

## colcon: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

1. RUN everytime a new file is added (run from root of workspace)

    ```
    colcon build --symlink-install
    ```

## rosdep: https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html

1. GET dependencies (run from root of workspace)

    ```
    rosdep install -i --from-path src --rosdistro humble -y
    ```

## intel realsense driver: 

https://dev.intelrealsense.com/docs/firmware-releases-d400

https://dev.intelrealsense.com/docs/firmware-update-tool

https://github.com/IntelRealSense/librealsense/releases/tag/v2.55.1

https://dev.intelrealsense.com/docs/firmware-releases-d400

https://github.com/IntelRealSense/realsense-ros

```realsense-viewer```

```ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true```

## meshlab: https://www.cloudcompare.org

## audio: https://github.com/mgonzs13/audio_common

## [INSTALLS]
```
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt-get install ros-humble-ros-gz
sudo apt install ros-humble-image-transport-plugins
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt-get install ros-humble-rviz2 ros-humble-tf2-ros ros-humble-tf2-tools
sudo apt install v4l-utils ros-humble-v4l2-camera
sudo apt install ros-humble-twist-mux
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-ros-gz
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
```

## [TUTORIAL INSTALLS]
```
sudo apt install ros-humble-turtlebot3-gazebo
```

## [RUN]
```
sudo usermod -aG video $USER
```


## [YDLidar]

* Amazon (lidar on sell): https://www.amazon.com/SmartFly-info-YDLIDAR-Scanner-Ranging/dp/B07W613C1K

* YDLidar SDK: https://github.com/YDLIDAR/YDLidar-SDK

* driver: https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble

* ROS params for lidar (X2/X2L): https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/humble/details.md

1. Run steps in terminal (home dir) (Ubuntu):

    ```
    sudo apt install cmake pkg-config
    sudo apt-get install python swig
    sudo apt-get install python-pip
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    cd YDLidar-SDK
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

1. New terminal (home dir):
    ```
    git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
    cd ydlidar_ros2_driver
    colcon build --symlink-install
    ```

1. Edit ~/.bashrc file:
    ```
    source ~/ydlidar_ros2_driver/install/setup.bash
    ```

1. New terminal (home dir):
    ```
    sudo usermod -a -G dialout $USER
    ```

1. Locate ~/ydlidar_ros2_driver/params/ydlidar.yaml change "port", "baudrate", "sample_rate", "isSingleChannel"
OR
Change "params_declare -> 'FILE_NAME'" to the appropriate .yaml file from params folder according to lidar model:


    > **port = /dev/ttyUSB0	<- this is for RaspberryPi**
    >
    > **port = /dev/serial/by-path/pci-0000:00:0c.0-usb-0:2:1.0-port0 <- change according to device (find by running "ls /dev/serial/by-path")**
    > 
    > **baudrate = 115200	<- according to https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/humble/details.md**
    > 
    > **sample_rate = 3		<- according to https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/humble/details.md**
    > 
    > **isSingleChannel = true	<- according to https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/humble/details.md**

1. Connect YDLidar with USB

1. Open new termional in ydlidar_ros2_driver & run:
    ```
    ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
    ```

## [SLAM]

1. cp mapper_params_online_async.yaml to WORKSPACE

1. RUN gazebo, rviz2

    ```
    ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/package_name/config/mapper_params_online_async.yaml use_sim_time:=true
    ```

1. In rviz2 map (change all topic to map), then save & serialize

1. Close all

1. CHANGE mode: localization, map_file_name: filepath, map_start_at_dock: true

1. RUN in seperate terminals

    ```
    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=filename.yaml -p use_sim_time:=true
    ```

    ```
    ros2 run nav2_util lifecycle_bringup map_server
    ```

    ```
    ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
    ```

    ```
    ros2 run nav2_util lifecycle_bringup amcl
    ```

1. RUN gazebo, rviz2

1. USE 2D Pose Estimate to point in direction on bot in gazebo after driving around a bit

## [OMNI WHEEL]

1. Controller package: https://github.com/hijimasa/omni_wheel_controller

1. Velocity publisher from example: https://github.com/hijimasa/omni_wheel_controller_sample/tree/main

1. Working version of packages is included, run `colcon build` from workspace root only.

1. Info
```
# Topic: /joint_states

## Left Spin In Place

header:
  stamp:
    sec: 1499
    nanosec: 54000000
  frame_id: ''
name:
- wheel0_shaft_joint
- wheel1_shaft_joint
- wheel2_shaft_joint
- wheel3_shaft_joint
position:
- -7218.228571608138
- -6755.156594802749
- -7265.258190252658
- -6821.34946339218
velocity:
- -79.18731688693963
- -79.18777725259231
- -79.2113001734645
- -79.2116260709436
effort:
- 0.0
- 0.0
- 0.0
- 0.0
---

## Moving In A Straight Line

header:
  stamp:
    sec: 1357
    nanosec: 990000000
  frame_id: ''
name:
- wheel0_shaft_joint
- wheel1_shaft_joint
- wheel2_shaft_joint
- wheel3_shaft_joint
position:
- -90.17730397791074
- -1369.1110855908364
- 763.0103951699043
- 694.043253808813
velocity:
- -23.84312982487037
- -116.67396775479612
- 27.283205553859798
- 15.142812589465766
effort:
- 0.0
- 0.0
- 0.0
- 0.0
---
```