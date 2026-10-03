# Third-party packages

| Package | Source | Why |
|---|---|---|
| `topic_based_ros2_control` | https://github.com/PickNikRobotics/topic_based_ros2_control (main @ aa9f9ef, BSD-3) | ros2_control hardware that exchanges joint commands / states with Isaac Sim over ROS topics. Not packaged for Jazzy in apt. Built with `BUILD_TESTING=OFF` (see `colcon.meta`). |
