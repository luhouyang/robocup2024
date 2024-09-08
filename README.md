# ROBOCUP


## Main Package Template

* https://github.com/joshnewans/my_bot

## Follow the Setup Notes If You Run Into Any Isseus

* [Setup Notes](https://github.com/luhouyang/robocup2024/blob/main/notes.md)

## Install Ros2 Humble (Ubuntu 22.04 LTS)

```
locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

```
source /opt/ros/humble/setup.bash
ros2 topic list
```

Add `source /opt/ros/humble/setup.bash` to ~/.bashrc file to source ros2 automatically on all new terminals

## Add Groups

```
sudo usermod -aG video $USER
sudo usermod -aG audio $USER
sudo usermod -a -G dialout $USER
```

Restart to take effect

## Install Dependencies

Ros2 Packages
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

RealSense
```
```

YDLidar
```
```

## `~/.bashrc` file

```
#
# PREDEFINED CONFIGURATIONS
#

source /opt/ros/humble/setup.bash
source ~/ydlidar_ros2_driver/install/setup.bash
# path to ydlidar setup file assuming package is build from source, with project root dir at HOME
```

## Build & Run

1. Resolve Dependencies

  ```
  rosdep install -i --from-path src --rosdistro humble -y
  ```

1. Run colcon build in workspace root
   
   ```
   colcon build --symlink-install
   ```

1. Source the setup file. *Source the other packages too, or add them to ~/.bashrc*
   
   ```
   source install/setup.bash
   ```
   
1. Run ros2 launch
   
   ```
   ros2 launch main_package launch_sim.launch.py
   ```

## VS Code Preferences (User JSON)

* yapf (EeyoreLee)

* XML (Red Hat)

```json
{
  "workbench.colorTheme": "Default High Contrast",
  "[python]": {
    "editor.formatOnSaveMode": "file",
    "editor.formatOnSave": true,
    "editor.defaultFormatter": "eeyore.yapf",
    "editor.formatOnType": false,
  },
  "yapf.args": [
    "--style={based_on_style: google, column_limit: 80, indent_width: 4}",
    "--style={based_on_style: pep8; DEDENT_CLOSING_BRACKETS: True}",
    "--style={based_on_style: pep8; ALLOW_SPLIT_BEFORE_DICT_VALUE: False, SPLIT_ALL_COMMA_SEPARATED_VALUES: True}"
  ],
  "redhat.telemetry.enabled": true,
  "[xml]": {
    "editor.defaultFormatter": "redhat.vscode-xml",
    "editor.tabSize": 4,
    "editor.trimAutoWhitespace": true,
    "editor.formatOnSaveMode": "file",
    "editor.formatOnSave": true,
  },
  "xml.format.maxLineWidth": 120,
  
}
```
