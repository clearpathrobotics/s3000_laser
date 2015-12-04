cmake_minimum_required(VERSION 2.8.3)
project(s3000_laser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_updater
  dynamic_reconfigure
  self_test
  sensor_msgs
  std_srvs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES s3000_laser SerialDevice
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  external/SerialDevice/include/SerialDevice
)

add_library(SerialDevice
  external/SerialDevice/src/SerialDevice.cc
)

add_executable(s3000_laser src/s3000_laser.cc src/sicks3000.cc)
add_dependencies(s3000_laser SerialDevice)

target_link_libraries(s3000_laser
  SerialDevice
  ${catkin_LIBRARIES}
)

install(TARGETS s3000_laser SerialDevice
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

