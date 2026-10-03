# Third-party packages

| Package | Source | Why |
|---|---|---|
| `topic_based_ros2_control` | https://github.com/PickNikRobotics/topic_based_ros2_control (main @ aa9f9ef, BSD-3) | ros2_control hardware that exchanges joint commands / states with Isaac Sim over ROS topics. Not packaged for Jazzy in apt. Built with `BUILD_TESTING=OFF` (see `colcon.meta`). **Patched** (`src/topic_based_system.cpp`, `write()`): joints without command interfaces are left out of the command message; upstream named them without a position, so Isaac rejected every command when state-only mimic joints were present; and non-finite (NaN) position commands, which ros2_control uses before a controller writes, are replaced by the measured position instead of being forwarded. |
