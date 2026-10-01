cmake_minimum_required(VERSION 3.8)
project(mediapipe_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandLandmark.msg"
  "msg/HandLandmarkArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
