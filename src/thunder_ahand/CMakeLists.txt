cmake_minimum_required(VERSION 3.5)

include(FetchContent)
FetchContent_Declare(
   yaml-cpp
   GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
   GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

project(thunder_ahand)

# =====================================================
# Compiler options
# =====================================================
## Find ament macros and libraries
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# Set the output directory for libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


# =====================================================
# Dependencies
# =====================================================
find_package(ament_cmake REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# =====================================================
# Build
# =====================================================

# source files
file(GLOB_RECURSE SOURCES
  ${CMAKE_SOURCE_DIR}/src/*.cpp
)

add_library(thunder_ahand SHARED ${SOURCES})

target_include_directories(thunder_ahand PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(thunder_ahand
  PRIVATE yaml-cpp
  PUBLIC Eigen3::Eigen
)

# =====================================================
# Installation
# =====================================================

install(
  TARGETS thunder_ahand
  EXPORT export_thunder_ahand
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# export
ament_export_targets(export_thunder_ahand HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp Eigen3)


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
