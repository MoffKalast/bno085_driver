cmake_minimum_required(VERSION 3.0.2)
project(bno085_driver)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs diagnostic_msgs)

catkin_package(CATKIN_DEPENDS rospy sensor_msgs std_msgs diagnostic_msgs)

catkin_install_python(PROGRAMS scripts/bno085_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch param DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
