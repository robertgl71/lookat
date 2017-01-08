cmake_minimum_required(VERSION 2.8.3)
project(lookat)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roslib geometry_msgs genmsg)
find_package( OpenCV REQUIRED )

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lookat src/lookat.cpp)
target_link_libraries(lookat ${catkin_LIBRARIES};${OpenCV_LIBS})
add_dependencies(lookat lookat_generate_messages_cpp)

add_executable(point_listener src/listener.cpp)
target_link_libraries(point_listener ${catkin_LIBRARIES})
add_dependencies(point_listener lookat_generate_messages_cpp)
