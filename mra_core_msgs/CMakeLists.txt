cmake_minimum_required(VERSION 2.8.3)
project(mra_core_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  AssemblyState.msg
  AssemblyStates.msg
  CollisionAvoidanceState.msg
  CollisionDetectionState.msg
  EndEffectorCommand.msg
  EndEffectorProperties.msg
  EndEffectorState.msg
  EndpointState.msg
  EndpointStates.msg
  JointCommand.msg

)

add_service_files(DIRECTORY srv FILES

  SolvePositionIK.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
)
