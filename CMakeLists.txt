cmake_minimum_required(VERSION 3.16)
project(xsens_mti_driver)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  lib/xspublic
)

link_directories(
    lib/xspublic/xscontroller
    lib/xspublic/xscommon
    lib/xspublic/xstypes)

add_executable(
        xsens_mti_node
        src/main.cpp
        src/xdainterface.cpp
        src/xdacallback.cpp
)


ament_target_dependencies(xsens_mti_node
  rclcpp
  tf2_ros
  geometry_msgs
  sensor_msgs
  diagnostic_updater
)
target_link_libraries(
        xsens_mti_node
        xscontroller
        xscommon
        xstypes
)

# Install targets
install(
  TARGETS xsens_mti_node
  EXPORT xsens_mti_nodeTargets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY
    launch
    param
  DESTINATION share/${PROJECT_NAME}/
)

add_custom_command(TARGET xsens_mti_node
    PRE_BUILD COMMAND +make -C ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
)

ament_package()
