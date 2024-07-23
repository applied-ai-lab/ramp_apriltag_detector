cmake_minimum_required(VERSION 3.0.2)
project(ramp_apriltag_detector)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
)

# catkin_python_setup()

# catkin_package(
#     CATKIN_DEPENDS sensor_msgs
# )

# catkin_install_python(PROGRAMS
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY resources/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
