#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <mra_basic/config.h>
#include <mra_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>


mra_core_msgs::JointCommand joints;

ros::Publisher *ptr_pub;
ros::Rate *ptr_loop_rate;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
      for(std::size_t i=0; i < mra_basic_config::jointID.size(); ++i)
      {
        joints.command[i] = msg->position[i];
      }
      ptr_pub->publish(joints);
   
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "joint_state_publisher_control"); 
  ros::NodeHandle n;

  ros::Publisher command_pub = n.advertise<mra_core_msgs::JointCommand> (JOINT_COMMAND_TOPIC, 1000);
  ptr_pub = &command_pub;
  joints.command.resize(mra_basic_config::jointID.size());

 
  ros::Subscriber sub = n.subscribe("joint_states", 1000, chatterCallback);

  ros::spin();

  return 0;
}
