# The mra_ros_control respository

## Travis - Continuous Integration

Indigo | Kinetic
------ | -------
[![Build Status](https://travis-ci.org/ros-planning/moveit.svg?branch=indigo-devel)](https://travis-ci.org/ros-planning/moveit) | [![Build Status](https://travis-ci.org/ros-planning/moveit.svg?branch=kinetic-devel)](https://travis-ci.org/ros-planning/moveit) |

**This repository is used to control real aubo's MRA(module robot arm) by ROS.**<br>
1. The **mra_basic** package is the driver to commuicate with the real MRA hardware and provides the some related topic of the ros control.<br>
2. The **mra_control** package is the abstraction of real MRA hardware.<br>
3. The **mra_core_msgs** package is the communication messages between mra_control and mra_basic.<br>
4. The **mra_joint_state_publisher** package is used to simulate MRA_API to send joint state and mra state<br>
5. The **ros_control** stack :See [ros_control documentation](http://ros.org/wiki/ros_control) on ros.org<br>
6. The **ros_controllers** stack is the implement of related ros controllers.<br>
7. The **moveit_visual_tools** package is the helper for displaying and debugging MoveIt! data in Rviz<br>
8. The **mra_joint_state_publisher** package is used to control mra joints by subscribing the "/joint_state" topic published by starting demo.launch in the *_moveit_config.

## Features
<img align="right" src="./resources/mra7a.png" width=500 height=300/>

* The MRA has kinds of arm configurations composed of the module joins, such as dual arms, 7Dofs arm, 6Dofs arm, etc.
* You can only modify the configure file to suit different Dof arms, and don't need to modify the mra_basic source code.
* Support the trajectory msgs planning by move_group in MoveIt!. 
* Support safe operations in real-time.
*  Multiple kinds of control mode
    * basic position, velocity, effort.
    * controlling signle joint or arms by ros_control.
    
## Prerequisites
* A kind of MRA description with moveit configure.(If you need to plan to control the mra.) For example [mra7a](https://github.com/auboROS/mra7a).
* A real MRA and gripper(Optional).
* [ROS Indigo](http://wiki.ros.org/ROS/Installation) on (recommended) Ubuntu 14.04 of 32 bit or 64bit, or ROS Kinetic on Ubuntu 16.04 (under development)
   These packages may work with other configurations as well, but they have only been tested for the one recommended above.
* [MoveIt!](http://moveit.ros.org/install/): sudo apt-get install ros-<indigo\>-moveit

## Using instruction
* Create a catkin workspace if you don't already have one (we recommend a separate one for MRA)

        mkdir -p ~/ros_ws/mra_ros_ws/src  	
        cd ~/ros_ws/mra_ros_ws/src  
        catkin_init_workspace 
	
* Clone this respository to your catkin workspace

        cd ~/ros_ws/mra_ros_ws/src  
        git clone https://github.com/auboROS/mra_ros_control.git
        
* Compiling

        cd ~/ros_ws/mra_ros_ws
        rosdep install --from-paths src --ignore-src --rosdistro indigo -y
        catkin_make
        
* Running

    Firstly, startup the your MRA hardware.
  
    The three control way as below:

    * Control single joint by ros_control.
     
            roslaunch mra_control mra7a_hw_position_bingup.launch
            rostopic pub /mra7a/joint7_position_controller/command std_msgs/Float64 1.5 //control the seventh joint move 1.5 radio.
               
    * Control the whole arm by planning.
    
            roslaunch mra_control mra7a_trajectory_rviz.launch 
            
    * Contorl single joint by QT control panel.(You need install [QT](http://download.qt.io/archive/qt/))<br>
        In the mra_basic/src/control_panel/CMakeLists.txt
    
            set(CMAKE_PREFIX_PATH "/home/lmn/Qt5.3.2/5.3/gcc/lib/cmake") //Set your QT path.
            
        In the mra_basic/CMakeLists.txt, cancel the comment below.
    
            //add_subdirectory(src/control_panel)
        Running in three terminal
        
            first terminal: roscore
            second terminal: rosrun mra_basic joint_control
            third terminal: rosrun mra_basic control_panel



	
	
	
	
	
	
