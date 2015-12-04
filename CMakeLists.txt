cmake_minimum_required(VERSION 2.8.3)
project(s3000_laser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_updater
  dynamic_reconfigure
  self_test
  sensor_msgs
  serial
  std_srvs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s3000_laser src/s3000_laser.cc src/sick_s3000.cc)
target_link_libraries(s3000_laser
  ${catkin_LIBRARIES}
)

install(TARGETS s3000_laser
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

