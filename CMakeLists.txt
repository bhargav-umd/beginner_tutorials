cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
add_service_files(FILES service.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(BEFORE ${GTEST_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
if(CATKIN_ENABLE_TESTING)
      find_package(rostest REQUIRED)
      add_rostest_gtest(talker_test launch/test.launch test/talker_test.cpp)
      target_link_libraries(talker_test ${catkin_LIBRARIES})
      endif()
