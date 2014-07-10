cmake_minimum_required(VERSION 2.8.3)
project(matlab_rosbag)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS regex)
find_package(catkin REQUIRED COMPONENTS rosbag_storage tf2 roscpp_serialization)

catkin_package(CATKIN_DEPENDS rosbag_storage tf2 roscpp_serialization)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_library(matlab_rosbag src/parser.cpp)
target_link_libraries(matlab_rosbag ${catkin_LIBRARIES})

catkin_add_gtest(parser_test src/parser_test.cpp)
target_link_libraries(parser_test matlab_rosbag ${catkin_LIBRARIES})
