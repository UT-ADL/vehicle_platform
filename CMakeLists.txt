cmake_minimum_required(VERSION 2.8.12...3.25.2)
project(vehicle_platform)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS nodes/novatel_oem7_driver_diag/novatel_oem7_driver_diag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
        config launch rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
