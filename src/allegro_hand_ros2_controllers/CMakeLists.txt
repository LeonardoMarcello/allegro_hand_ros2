cmake_minimum_required(VERSION 3.8)
project(allegro_hand_ros2_controllers)

# =====================================================
# Compiler options
# =====================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================================
# Dependencies
# =====================================================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(bhand REQUIRED)
find_package(thunder_ahand REQUIRED)

# =====================================================
# Controller plugin library
# =====================================================

# Computed Torque Controller
# -----------------------------
add_library(allegro_hand_computed_torque SHARED
  src/thunder_ahand_finger.cpp
  src/ahand_finger_gen.cpp
  src/thunder_ahand_thumb.cpp
  src/ahand_thumb_gen.cpp
  src/allegro_hand_computed_torque.cpp
)

# ROS dependencies (NO yaml-cpp here)
ament_target_dependencies(allegro_hand_computed_torque
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  realtime_tools
  std_msgs
  sensor_msgs
  pluginlib

  thunder_ahand
)

target_link_libraries(allegro_hand_computed_torque
  yaml-cpp
  Eigen3::Eigen
  #${bhand_LIBRARIES} # <- needed when implementing via bhand library
)

# Include directories (target-scoped)
target_include_directories(allegro_hand_computed_torque PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  #${bhand_INCLUDE_DIRS} # <- needed when implementing via bhand library
  ${thunder_ahand_INCLUDE_DIRS} # <- needed when implementing via thunder ahand library
)

# =====================================================
# Plugin export
# =====================================================
pluginlib_export_plugin_description_file(
  controller_interface
  ahand_controllers_plugin.xml
)

# =====================================================
# Installation
# =====================================================
install(
  TARGETS allegro_hand_computed_torque
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# =====================================================
# Testing
# =====================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
