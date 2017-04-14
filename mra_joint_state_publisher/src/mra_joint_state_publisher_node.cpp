#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <mra_core_msgs/AssemblyState.h>
#include <mra_core_msgs/JointCommand.h>


sensor_msgs::JointState joints;

void callBack(const mra_core_msgs::JointCommand &msg) {
    for (int i=0; i<7; i++) {
        joints.position[i] = msg.command[i];
        joints.velocity[i] = 0.00;
        joints.effort[i] = 0.000;
    }
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "mra_joint_state_publisher");
  ros::NodeHandle n;

  ros::Rate loop_rate(100);


  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState> ("/mra/joint_states", 1000);
  ros::Publisher state_pub = n.advertise<mra_core_msgs::AssemblyState> ("/mra/state", 1);


  joints.position.resize(7);
  joints.velocity.resize(7);
  joints.effort.resize(7);
  joints.name.resize(7);
  joints.name[0] = "Joint1";
  joints.name[1] = "Joint2";
  joints.name[2] = "Joint3";
  joints.name[3] = "Joint4";
  joints.name[4] = "Joint5";
  joints.name[5] = "Joint6";
  joints.name[6] = "Joint7";

  for (int i=0; i<7; i++) {
      joints.position[i] = 1.0;
      joints.velocity[i] = 0.00;
      joints.effort[i] = 0.000;
  }

  ros::Subscriber command_sub = n.subscribe("/mra/joint_command",1,&callBack);


  mra_core_msgs::AssemblyState state;
  state.enabled = true;


  while(ros::ok()) {
    joint_pub.publish(joints);
    state_pub.publish(state);
    loop_rate.sleep();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}
