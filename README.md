# The mra_ros_control respository
**This repository is used to control real aubo's MRA(module robot amr) by ROS.**<br>
1. The **mra_basic** package is the driver to commuicate with the real MRA hardware and provides the some related topic of the ros control.<br>
2. The **mra_control** package is the abstraction of real MRA hardware.<br>
3. The **mra_core_msgs** package is the communication messages between mra_control and mra_basic.<br>
4. The **mra_joint_state_publisher** package is used to simulate MRA_API to send joint state and mra state<br>
5. The **ros_control** stack :See [ros_control documentation](http://ros.org/wiki/ros_control) on ros.org<br>
6. The **ros_controllers** stack is the implement of related ros controllers.<br>
7. The **moveit_visual_tools** package is the helper for displaying and debugging MoveIt! data in Rviz<br>