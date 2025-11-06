cmake_minimum_required(VERSION 3.8)
project(allegro_utils)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(bhand REQUIRED)

message(STATUS "bhand include dirs: ${bhand_INCLUDE_DIRS}")
message(STATUS "bhand libraries: ${bhand_LIBRARIES}")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# === Cpp ===
# Include Cpp "include" directory
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIRS}
  ${bhand_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../bhand/include
)

# Create Cpp Libraries
add_library(${PROJECT_NAME}
	src/thunder_ahand_finger.cpp
	src/ahand_finger_gen.cpp
  src/kalman_filter_joint.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp yaml-cpp)


# Create Cpp executable
add_executable(test_thunder
  src/test_thunder.cpp
)
target_link_libraries(test_thunder
  ${PROJECT_NAME}
  yaml-cpp
)
ament_target_dependencies(test_thunder rclcpp std_msgs sensor_msgs)


add_executable(test_controller
  src/test_controller.cpp
)
target_link_libraries(test_controller
  ${PROJECT_NAME}
  yaml-cpp
  ${bhand_LIBRARIES}
)
ament_target_dependencies(test_controller rclcpp std_msgs sensor_msgs bhand)



# Install Cpp executables
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
install(TARGETS
  test_thunder
  test_controller
  DESTINATION lib/${PROJECT_NAME}
)

# === Python ===
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/gui.py
  scripts/moveit_joint_broadcaster.py
  scripts/wave.py
  scripts/test_traject.py
  scripts/my_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

# install additional directories
install(DIRECTORY config
	DESTINATION share/${PROJECT_NAME}
)


# ====== BUILD ======

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
