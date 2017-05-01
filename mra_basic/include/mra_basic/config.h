#ifndef CONFIG_H_
#define CONFIG_H_

#include <iostream>
#include <vector>

namespace mra_basic_config {
//const std::string DEFAULT_NODE = "/dev/pcanusb32"; //use:DEFAULT_NODE.c_str()
#define DEFAULT_NODE "/dev/pcanusb32"
const std::vector<int> jointID{21,22,23,24,25,26,27};
const std::vector<std::string> joint_names{"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6","Joint7"};

#define CONTROL_RATE 100 //100HZ

/*Topic name definition*/
#define JOINT_STATE_TOPIC "/mra/joint_states"  //-->pub joint state
#define STATE_TOPIC "/mra/state"               //Robot State Related-->pub
#define JOINT_COMMAND_TOPIC "/mra/joint_command"    //-->sub joint control command
#define JOINT_POSITION_COMMAND_TOPIC "/moveJ"    //-->sub joint control command
#define RESET_MRA_API_TOPIC "/mra/reset_MRA_API"    //-->sub, When canbus interrupts, we need to reset MRA_API after reconnecting the usb-can.


/*Position controll topic*/
#define JOINT1_POSITION_CONTROLLER "/mra7a/joint1_position_controller/command"
#define JOINT2_POSITION_CONTROLLER "/mra7a/joint2_position_controller/command"
#define JOINT3_POSITION_CONTROLLER "/mra7a/joint3_position_controller/command"
#define JOINT4_POSITION_CONTROLLER "/mra7a/joint4_position_controller/command"
#define JOINT5_POSITION_CONTROLLER "/mra7a/joint5_position_controller/command"
#define JOINT6_POSITION_CONTROLLER "/mra7a/joint6_position_controller/command"
#define JOINT7_POSITION_CONTROLLER "/mra7a/joint7_position_controller/command"

/*For Dual Arms*/
const bool isDualArm = false;//When you using dual arms, please set the value to true.
#define L_ARM_DEV "/dev/pcanusb32" //Setting this according to your computer.
#define R_ARM_DEV "/dev/pcanusb33"
const std::vector<int> L_jointID{21,22,23,24,25,26,27};
const std::vector<int> R_jointID{31,32,33,34,35,36,37};
const std::vector<std::string> R_joint_names{"rJoint1","rJoint2","rJoint3","rJoint4","rJoint5","rJoint6","rJoint7"};
const std::vector<std::string> L_joint_names{"lJoint1","lJoint2","lJoint3","lJoint4","lJoint5","lJoint6","lJoint7"};


#define L_ARM_COMMAND_TOPIC "/mra/L_ARM/joint_command"    //-->sub joint control command
#define R_ARM_COMMAND_TOPIC "/mra/R_ARM/joint_command"    //-->sub joint control command

}

#endif
