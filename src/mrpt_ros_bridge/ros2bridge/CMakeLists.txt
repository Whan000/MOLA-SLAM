cmake_minimum_required(VERSION 3.10)
project(mrpt_libros_bridge)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Required to detect ROS 2 environment
find_package(ament_cmake REQUIRED)

# Find dependencies listed in package.xml
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Find MRPT dependency (from mrpt_libapps)
find_package(mrpt-apps REQUIRED)

# Define the "lib${PROJECT_NAME}.so" shared library
add_library(${PROJECT_NAME} SHARED
  src/time.cpp
  src/pose.cpp
  src/laser_scan.cpp
  src/map.cpp
  src/gps.cpp
  src/range.cpp
  src/imu.cpp
  src/point_cloud2.cpp
  src/image.cpp
  src/stereo_image.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
  #rclcpp::rclcpp
  #rosbag2_cpp::rosbag2_cpp
  #tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
  cv_bridge::cv_bridge
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  mrpt::apps
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs_library
  std_msgs::std_msgs__rosidl_typesupport_cpp
  stereo_msgs::stereo_msgs__rosidl_typesupport_cpp
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2::tf2
)

# Installation rules
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Export package information
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  sensor_msgs
  nav_msgs
  stereo_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  cv_bridge
  mrpt-apps
)

# Unit tests
if(BUILD_TESTING)
  # find GTest and ament_cmake_gtest
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_${PROJECT_NAME}
    src/map_unittest.cpp
    src/pointcloud2_unittest.cpp
    src/pose_unittest.cpp
    src/time_unittest.cpp
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
    )
  endif()
endif()

ament_package()

# NOTE: Remove this when cv_bridge versions are >3.4.0 on all ROS distros.

# ===================================================================================================
# Credits: https://gist.github.com/jtanx/96ded5e050d5ee5b19804195ee5cf5f9
function(pad_string output str padchar length)
  string(LENGTH "${str}" _strlen)
  math(EXPR _strlen "${length} - ${_strlen}")

  if(_strlen GREATER 0)
    string(REPEAT ${padchar} ${_strlen} _pad)
    string(PREPEND str ${_pad})
  endif()

  set(${output} "${str}" PARENT_SCOPE)
endfunction()
# 2 hex digits for each version part:
# For example: "0.5.1"  => "0x000501"
macro(mrpt_version_to_hex VER TARGET_VAR_NAME)
	string(REGEX MATCHALL "[0-9]+" __parts "${${VER}}")

	if(__parts)
		list(GET __parts 0 __VERSION_NUMBER_MAJOR)
		list(GET __parts 1 __VERSION_NUMBER_MINOR)
		list(GET __parts 2 __VERSION_NUMBER_PATCH)
		pad_string(__VERSION_NUMBER_MAJOR ${__VERSION_NUMBER_MAJOR} "0" 2)
		pad_string(__VERSION_NUMBER_MINOR ${__VERSION_NUMBER_MINOR} "0" 2)
		pad_string(__VERSION_NUMBER_PATCH ${__VERSION_NUMBER_PATCH} "0" 2)
		set(${TARGET_VAR_NAME} "0x${__VERSION_NUMBER_MAJOR}${__VERSION_NUMBER_MINOR}${__VERSION_NUMBER_PATCH}")
	else()
		set(${TARGET_VAR_NAME} "0x000")
	endif()
endmacro()

# Convert package versions to hex so they can be used in preprocessor for wider
# versions compatibility of "one-source for all":
mrpt_version_to_hex(cv_bridge_VERSION  cv_bridge_VERSION_HEX)
target_compile_definitions(${PROJECT_NAME} PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})
# ===================================================================================================


# === Export a standard CMake config file for backwards compatibility from MRPT2 ===
include(CMakePackageConfigHelpers)

# Create an alias target so users can link with mrpt::ros2bridge
add_library(mrpt::ros2bridge ALIAS ${PROJECT_NAME})

# Project version: autodetect from caller package.xml
# Example line:" <version>0.3.2</version>"
file(READ ${CMAKE_SOURCE_DIR}/package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(MRPT_ROS2BRIDGE_VERSION ${CMAKE_MATCH_1})
message(STATUS "MRPT_ROS2BRIDGE_VERSION version: ${MRPT_ROS2BRIDGE_VERSION} (detected in package.xml)")


# Generate a simple config file that re-exports the target
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mrpt-ros2bridge-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros2bridge-config.cmake"
  INSTALL_DESTINATION lib/cmake/mrpt-ros2bridge
)

# Generate an accompanying version file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros2bridge-config-version.cmake"
  VERSION ${MRPT_ROS2BRIDGE_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Install both config and version files
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros2bridge-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros2bridge-config-version.cmake"
  DESTINATION lib/cmake/mrpt-ros2bridge
)
