cmake_minimum_required(VERSION 2.8.3)
project(vehicle_platform)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        delphi_esr_msgs
        message_generation
)

add_message_files(
        FILES
        EsrTrackArray.msg
)

generate_messages(
        DEPENDENCIES
        delphi_esr_msgs
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        delphi_esr_msgs
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(radar_topic_merger nodes/radar_topic_merger/radar_topic_merger.cpp)
target_link_libraries(radar_topic_merger ${catkin_LIBRARIES})
add_dependencies(radar_topic_merger ${catkin_EXPORTED_TARGETS})

install(
        TARGETS radar_topic_merger
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
        config launch scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})