cmake_minimum_required(VERSION 3.8)
project(allegro_hand_isaacsim)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


add_executable(allegro_hand_sim2real src/sim2real.cpp)
ament_target_dependencies(allegro_hand_sim2real
  rclcpp
  sensor_msgs
)

add_executable(allegro_hand_sim src/sim.cpp)
ament_target_dependencies(allegro_hand_sim
  rclcpp
  sensor_msgs
  trajectory_msgs
)



install(TARGETS
  allegro_hand_sim2real
  allegro_hand_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
