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
#include <std_msgs/Int8.h>
#include <vector>

using namespace mra_basic_config;
using namespace std;


UserControlOnCan *userControlOnCan;
Gripper *gripper;
mra_core_msgs::AssemblyState mra_state;

void copy_ID_in_current_canbus()
{
    jointID.clear();
    cout << "Joint ID in CANBUS:";
    for (std::vector<Joint>::iterator iter = userControlOnCan->controller.allJoint.begin();
         iter != userControlOnCan->controller.allJoint.end();
         ++iter) {
        cout << " --- " << iter.base()->ID;
        jointID.push_back(iter.base()->ID);
    }
    cout << endl;

    cout<<Color_light_cyan<< "Sort Joint ID:";
    std::sort(jointID.begin(),jointID.end());
    for (std::vector<int>::iterator iter = jointID.begin();
         iter != jointID.end();
         ++iter) {
        cout << " --- " << *iter;
    }
    cout <<Color_end<< endl;


    cout << "Finger ID in CANBUS:";
    for (std::vector<Gripper>::iterator iter = userControlOnCan->controller.allGripper.begin();
         iter != userControlOnCan->controller.allGripper.end();
         ++iter) {
        cout << " --- " << iter.base()->ID;
        GRIPPER_ID = iter.base()->ID;
    }
    cout << endl;
}

void joint_command_callback(const mra_core_msgs::JointCommandConstPtr &msg)
{
    //    for(int i=0; i<msg->command.size(); i++) {
    //        std::cout<<msg->command[i]<<" || ";
    //    }
    //    std::cout<<std::endl;

    if(mra_state.canbus_state==mra_core_msgs::AssemblyState::CANBUS_STATE_NORMAL) {
        for(int i=0; i<jointID.size(); i++) {
            bool isSent = userControlOnCan->setJointTagPos(jointID[i],msg->command[i]);
            if (isSent==false) {
                ROS_ERROR("Senting is failure in ID:%d",jointID[i]);
                //set canbus state = CANBUS_STATE_INTERRUPT
                mra_state.canbus_state = mra_core_msgs::AssemblyState::CANBUS_STATE_INTERRUPT;
            }
        }
    }
}
void moveJ_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    //    ROS_INFO("[%f,%f,%f,%f,%f,%f,%f]",
    //             msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6]);

    if(mra_state.canbus_state==mra_core_msgs::AssemblyState::CANBUS_STATE_NORMAL) {
        for(int i=0; i<jointID.size(); i++) {
            bool isSent = userControlOnCan->setJointTagPos(jointID[i],msg->data[i]);
            if (isSent==false) {
                ROS_ERROR("Senting is failure in ID:%d",jointID[i]);
                //set canbus state = CANBUS_STATE_INTERRUPT
                mra_state.canbus_state = mra_core_msgs::AssemblyState::CANBUS_STATE_INTERRUPT;
            }
        }
    }
}

void gripper_command_callback(const std_msgs::Int8ConstPtr &msg){
    if(gripper != NULL){
        ROS_INFO("Gripper work on %d", msg->data);
        gripper->setFingerTagOpenStatus(msg->data);
    }

}

void MRA_API_INIT(const std_msgs::Bool &reset) {

    userControlOnCan = new UserControlOnCan();
    if(userControlOnCan->Init(CAN_NODE_DEV.c_str())) {
        copy_ID_in_current_canbus();
        mra_state.canbus_state = mra_core_msgs::AssemblyState::CANBUS_STATE_NORMAL;
        mra_state.enabled = true;
        for(int i=0; i<jointID.size(); i++) {
            userControlOnCan->setJointAutoUpdateCurPos(jointID[i],true);
        }
    } else {
        std::string s;
        userControlOnCan->controller.GetErrorText(s);
        ROS_ERROR("Can't Open the peak can driver:%s",s.c_str());
    }
    if(userControlOnCan->controller.allGripper.size() != 0){
        gripper = userControlOnCan->findGripperID(mra_basic_config::GRIPPER_ID);
        gripper->setFingerLimTorque(450);
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "joint_control");
    ros::NodeHandle n;

    sleep(1);
    get_param(n);

    ros::Rate loop_rate(CONTROL_RATE);//default 100Hz

    /*mra's state information*/
    mra_state.enabled = false;
    mra_state.ready = true;
    mra_state.error = false;
    mra_state.canbus_state = mra_core_msgs::AssemblyState::CANBUS_STATE_INTERRUPT;
    mra_state.interruptJoints.resize(jointID.size());

    /*MRA_API first init*/
    std_msgs::Bool reset;
    reset.data = 0;
    MRA_API_INIT(reset);

    /*sub other node's joint control command*/
    ros::Subscriber sub_joint_command = n.subscribe(JOINT_COMMAND_TOPIC, 1, &joint_command_callback);
    /*sub the simple position control command, android app will send the topic*/
    ros::Subscriber sub_moveJ = n.subscribe(JOINT_POSITION_COMMAND_TOPIC, 20, &moveJ_callback);
    /*pub joint state*/
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState> (JOINT_STATE_TOPIC, 1000);
    /*pub mra's state, such as whether canbus is normal or not*/
    ros::Publisher state_pub = n.advertise<mra_core_msgs::AssemblyState> (STATE_TOPIC, 1);
    /*sub reset */
    ros::Subscriber sub_reset_MRA_API = n.subscribe(RESET_MRA_API_TOPIC, 10, &MRA_API_INIT);
    /*sub gripper command*/
    ros::Subscriber sub_gripper_command = n.subscribe(GRIPPER_COMMAND, 1, &gripper_command_callback);


    sensor_msgs::JointState joint_state;
    joint_state.position.resize(jointID.size());
    joint_state.velocity.resize(jointID.size());
    joint_state.effort.resize(jointID.size());
    joint_state.name.resize(jointID.size());
    joint_state.name = joint_names;

    while (ros::ok()) {

        if(mra_state.canbus_state==mra_core_msgs::AssemblyState::CANBUS_STATE_NORMAL) {
            /*get joint states and publish it*/
            for(int i=0; i<jointID.size(); i++) {
                joint_state.position[i] = userControlOnCan->readJointCurPos(jointID[i]);
                joint_state.velocity[i] = userControlOnCan->readJointCurSpd(jointID[i]);
                joint_state.effort[i] = userControlOnCan->readJointCurI(jointID[i]);
            }
            joint_state_pub.publish(joint_state);

            /*pub mra_state*/
            for(int i=0; i<jointID.size(); i++) {
                //If any joint position>0.01 radio, the mra's joints are not in home position.Set ready = false.
                joint_state.position[i] > 0.01 ? mra_state.ready=false : mra_state.ready=true;
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
