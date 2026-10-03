cmake_minimum_required(VERSION 3.8)
project(hruh_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS nodes/hruh_joystick.py nodes/joy_layout_normalizer.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
