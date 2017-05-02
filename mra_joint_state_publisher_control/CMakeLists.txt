cmake_minimum_required(VERSION 2.8.3)
project(mra_joint_state_publisher_control)

#support C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        cmake_modules
        mra_basic
        mra_core_msgs)



#generate_messages(
 # DEPENDENCIES actionlib_msgs std_msgs 
#)#action_server需要的# Or other packages containing msgs

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

#ADD_EXECUTABLE(OUR_jointcontrol ${LIBJOINTCONTROL_SRC})
#TARGET_LINK_LIBRARIES(OUR_jointcontrol ${LIBPCAN_LIB} -DNO_RT)
