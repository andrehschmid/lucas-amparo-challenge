cmake_minimum_required(VERSION 2.8.3)
project(qt_publisher)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(qt_publisher src/qt_publisher.cpp)
target_link_libraries(qt_publisher ${catkin_LIBRARIES})
