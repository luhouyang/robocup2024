# ROBOCUP


## Main Package Template

* https://github.com/joshnewans/my_bot

## Follow the Setup Notes

* [Setup Notes](https://github.com/luhouyang/robocup2024/blob/main/notes.md)\

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
    "editor.formatOnType": false
  },
  "redhat.telemetry.enabled": true,
  "[xml]": {
    "editor.defaultFormatter": "redhat.vscode-xml",
    "editor.tabSize": 4,
    "editor.trimAutoWhitespace": true,
    "editor.formatOnSaveMode": "file",
    "editor.formatOnSave": true
  },
  "xml.format.maxLineWidth": 120,
}
```
