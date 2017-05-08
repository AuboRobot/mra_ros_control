cmake_minimum_required(VERSION 2.8.3)
project(mra_joint_state_publisher_control)

#support C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

        mra_basic
        mra_core_msgs)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS mra_core_msgs mra_basic#config.h
)


###########
## Build ##
###########



include_directories(${catkin_INCLUDE_DIRS})

ADD_EXECUTABLE(joint_state_publisher_control src/joint_state_publisher_control.cpp)
TARGET_LINK_LIBRARIES(joint_state_publisher_control ${catkin_LIBRARIES} ${libOURAPI_LIBS})
add_dependencies(joint_state_publisher_control ${catkin_EXPORTED_TARGETS}) # don't build until necessary msgs are finish


