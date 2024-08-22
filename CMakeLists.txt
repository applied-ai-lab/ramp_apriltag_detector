cmake_minimum_required(VERSION 3.0.2)
project(ramp_apriltag_detector)

find_package(catkin REQUIRED COMPONENTS 
            rospy 
            apriltag_ros
            std_msgs 
            tf2_ros
            tf2_geometry_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS 
rospy
std_msgs 
tf2_ros)

catkin_install_python(PROGRAMS
    scripts/beam_perception.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
