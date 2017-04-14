/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, AUBO Inc.
 *  All rights reserved.
 *  Author: LMN
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <mra_api.h>
#include <mra_basic/config.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <mra_core_msgs/AssemblyState.h>
#include <mra_core_msgs/JointCommand.h>
#include <std_msgs/Bool.h>

using namespace mra_basic_config;

UserControlOnCan *left_arm_dev;
UserControlOnCan *right_arm_dev;

mra_core_msgs::AssemblyState mra_state;

void L_ARM_command_callback(const mra_core_msgs::JointCommandConstPtr &msg)
{
    for(int i=0; i<msg->command.size(); i++) {
        std::cout<<msg->command[i]<<"#";
    }
    std::cout<<std::endl;

    if(mra_state.canbus_state==mra_core_msgs::AssemblyState::CANBUS_STATE_NORMAL) {
        for(int i=0; i<L_jointID.size(); i++) {
            bool isSent = left_arm_dev->setJointTagPos(L_jointID[i],msg->command[i]);
            if (isSent==false) {
                ROS_ERROR("Sent is failure");
                //set canbus state = CANBUS_STATE_INTERRUPT
                mra_state.canbus_state = mra_core_msgs::AssemblyState::CANBUS_STATE_INTERRUPT;
            }
        }
    }
}
void R_ARM_command_callback(const mra_core_msgs::JointCommandConstPtr &msg)
{
    for(int i=0; i<msg->command.size(); i++) {
        std::cout<<msg->command[i]<<"#";
    }
    std::cout<<std::endl;

    if(mra_state.canbus_state==mra_core_msgs::AssemblyState::CANBUS_STATE_NORMAL) {
        for(int i=0; i<R_jointID.size(); i++) {
            bool isSent = right_arm_dev->setJointTagPos(R_jointID[i],msg->command[i]);
            if (isSent==false) {
                ROS_ERROR("Sent is failure");
                //set canbus state = CANBUS_STATE_INTERRUPT
                mra_state.canbus_state = mra_core_msgs::AssemblyState::CANBUS_STATE_INTERRUPT;
            }
        }
    }
}

void MRA_API_INIT(const std_msgs::Bool &reset) {
    /*dual arms all init*/
    if(reset.data == 3) {
        int ok = 0;
        left_arm_dev = new UserControlOnCan();
        right_arm_dev = new UserControlOnCan();
        if(left_arm_dev->Init(L_ARM_DEV)) {
            ok++;
            for(int i=0; i<L_jointID.size(); i++) {
                left_arm_dev->setJointAutoUpdateCurPos(L_jointID[i],true);
            }
        } else {
            ROS_ERROR("Can't Open the L_ARM_DEV");
        }
        if(right_arm_dev->Init(R_ARM_DEV)) {
            ok++;
            for(int i=0; i<R_jointID.size(); i++) {
                right_arm_dev->setJointAutoUpdateCurPos(R_jointID[i],true);
            }
        } else {
            ROS_ERROR("Can't Open the R_ARM_DEV");
        }
        if(ok == 2) {
            mra_state.canbus_state = mra_core_msgs::AssemblyState::CANBUS_STATE_NORMAL;
            mra_state.enabled = true;
        }
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "joint_control_for_dual_arm");
    ros::NodeHandle n;
    ros::Rate loop_rate(CONTROL_RATE);//default 100Hz

    /*mra's state information*/
    mra_state.enabled = false;
    mra_state.ready = true;
    mra_state.error = false;
    mra_state.canbus_state = mra_core_msgs::AssemblyState::CANBUS_STATE_INTERRUPT;
    mra_state.interruptJoints.resize(L_jointID.size()+R_jointID.size());

    /*MRA_API first init*/
    std_msgs::Bool reset;
    reset.data = 3;
    MRA_API_INIT(reset);

    /*sub other node's joint control command*/
    ros::Subscriber sub_L_ARM_command = n.subscribe(L_ARM_COMMAND_TOPIC, 1, &L_ARM_command_callback);
    ros::Subscriber sub_R_ARM_command = n.subscribe(R_ARM_COMMAND_TOPIC, 1, &R_ARM_command_callback);
    /*pub joint state*/
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState> (JOINT_STATE_TOPIC, 1000);
    /*pub mra's state, such as whether canbus is normal or not*/
    ros::Publisher state_pub = n.advertise<mra_core_msgs::AssemblyState> (STATE_TOPIC, 1);
    /*sub reset */
    ros::Subscriber sub_reset_MRA_API = n.subscribe(RESET_MRA_API_TOPIC, 10, &MRA_API_INIT);


    sensor_msgs::JointState joint_state;
    joint_state.position.resize(L_jointID.size()+R_jointID.size());
    joint_state.velocity.resize(L_jointID.size()+R_jointID.size());
    joint_state.effort.resize(L_jointID.size()+R_jointID.size());
    joint_state.name.resize(L_jointID.size()+R_jointID.size());
    std::vector<std::string> dual_arm_joint_names;
    dual_arm_joint_names.insert(dual_arm_joint_names.end(),
                                L_joint_names.begin(),L_joint_names.end());
    dual_arm_joint_names.insert(dual_arm_joint_names.end(),
                                R_joint_names.begin(),R_joint_names.end());
    joint_state.name = dual_arm_joint_names;

    while (ros::ok()) {

        if(mra_state.canbus_state==mra_core_msgs::AssemblyState::CANBUS_STATE_NORMAL) {
            /*get joint states and publish it*/
                for(int i=0; i<L_jointID.size(); i++) {
                    joint_state.position[i] = left_arm_dev->readJointCurPos(L_jointID[i]);
                    joint_state.velocity[i] = left_arm_dev->readJointCurSpd(L_jointID[i]);
                    joint_state.effort[i] = left_arm_dev->readJointCurI(L_jointID[i]);
                }
                for(int i=0; i<R_jointID.size(); i++) {
                    joint_state.position[i+L_jointID.size()] = right_arm_dev->readJointCurPos(R_jointID[i]);
                    joint_state.velocity[i+L_jointID.size()] = right_arm_dev->readJointCurSpd(R_jointID[i]);
                    joint_state.effort[i+L_jointID.size()] = right_arm_dev->readJointCurI(R_jointID[i]);
                }
            joint_state_pub.publish(joint_state);

            /*pub mra_state*/
            for(int i=0; i<L_jointID.size(); i++) {
                //If any joint position>0.01 radio, the mra's joints are not in home position.Set ready = false.
                joint_state.position[i] > 0.01 ? mra_state.ready=false : mra_state.ready=true;
            }
            for(int i=0; i<R_jointID.size(); i++) {
                //If any joint position>0.01 radio, the mra's joints are not in home position.Set ready = false.
                joint_state.position[i+L_jointID.size()] > 0.01 ? mra_state.ready=false : mra_state.ready=true;
            }
            state_pub.publish(mra_state);
        }

        /*loop_rate default 100HZ*/
        loop_rate.sleep();
        ros::spinOnce();

    }

    ros::spin();

    return 0;
}
