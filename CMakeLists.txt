cmake_minimum_required(VERSION 2.8.3)
project(s3000_laser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_updater
  dynamic_reconfigure
  sensor_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include external/SerialDevice/include/SerialDevice
  LIBRARIES S3000Driver SerialDevice
  CATKIN_DEPENDS roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  external/SerialDevice/include/SerialDevice
)

add_library(SerialDevice
  external/SerialDevice/src/SerialDevice.cc
)
add_library(S3000Driver
  src/sicks3000.cc
)

add_executable(s3000_laser src/s3000_laser.cc)
add_dependencies(s3000_laser S3000Driver SerialDevice)
target_link_libraries(s3000_laser
  S3000Driver
  SerialDevice
  ${catkin_LIBRARIES}
)

install(TARGETS SerialDevice S3000Driver s3000_laser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ external/SerialDevice/include/SerialDevice
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
