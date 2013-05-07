cmake_minimum_required(VERSION 2.8.3)
project(matlab_rosbag)
find_package(catkin REQUIRED COMPONENTS rosbag)

catkin_package(CATKIN_DEPENDS
  rosbag rostime roscpp_serialization roscpp)

include_directories(${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS regex)

add_library(matlab_rosbag src/parser.cpp)

add_executable(read_bag_test src/read_bag_test.cpp)
target_link_libraries(read_bag_test matlab_rosbag ${catkin_LIBRARIES})

catkin_add_gtest(parser_test src/parser_test.cpp)
target_link_libraries(parser_test matlab_rosbag)
