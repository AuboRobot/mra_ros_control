/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, AUBO Inc.
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
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

/* Author: LMN(refer to baxter_cpp by Dave Coleman)
   Desc:   ros_control hardware interface layer for MRA
*/

#include <mra_control/mra_hardware_interface.h>
#include <print_color/print_color.h>
#include <mra_basic/config.h>

namespace mra_control
{

MRAHardwareInterface::MRAHardwareInterface():
    joint_mode_(1),
    loop_hz_(100)
{

    if(mra_basic_config::isDualArm){
        ROS_INFO("\033[22;34m[hardware_interface]: Reset dual arms' hardware' %s",Color_end);
        num_mra_joints = mra_basic_config::L_joint_names.size() + mra_basic_config::R_joint_names.size();
        L_mra_hw_.reset(new mra_control::ArmHardwareInterface(mra_basic_config::L_joint_names,loop_hz_,L_ARM_COMMAND_TOPIC));
        R_mra_hw_.reset(new mra_control::ArmHardwareInterface(mra_basic_config::R_joint_names,loop_hz_,R_ARM_COMMAND_TOPIC));
    } else{
        ROS_INFO("\033[22;34m[hardware_interface]: Reset single arm's' hardware' %s",Color_end);
        num_mra_joints = mra_basic_config::joint_names.size();
        mra_hw_.reset(new mra_control::ArmHardwareInterface(mra_basic_config::joint_names,loop_hz_,JOINT_COMMAND_TOPIC));
    }

    // Set the joint mode interface data
    jm_interface_.registerHandle(hardware_interface::JointModeHandle("joint_mode", &joint_mode_));

    // Start the shared joint state subscriber
    sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 1,
                                                              &MRAHardwareInterface::stateCallback, this);

    // Wait for first state message from real robot
    do
    {
        // Loop until we get our first joint_state message from MRA
        while(ros::ok() && state_msg_timestamp_.toSec() == 0)
        {
            ROS_INFO_STREAM_NAMED("hardware_interface","Waiting for first state message to be recieved");
            ros::spinOnce();
            ros::Duration(0.25).sleep();
        }
    } while (state_msg_->name.size() != this->num_mra_joints);

    // Initialize MRA's arm hardware
    if(mra_basic_config::isDualArm){
        L_mra_hw_->init(js_interface_, ej_interface_, vj_interface_, pj_interface_, &joint_mode_, state_msg_);
        R_mra_hw_->init(js_interface_, ej_interface_, vj_interface_, pj_interface_, &joint_mode_, state_msg_);
    } else{
        mra_hw_->init(js_interface_, ej_interface_, vj_interface_, pj_interface_, &joint_mode_, state_msg_);
    }

    // Register interfaces
    registerInterface(&js_interface_);
    registerInterface(&jm_interface_);
    registerInterface(&ej_interface_);
    registerInterface(&vj_interface_);
    registerInterface(&pj_interface_);

    // Enable MRA
    bool enabled = false;
    while(!enabled)
    {
        if( !mra_util_.enableMRA() )
        {
            ROS_WARN_STREAM_NAMED("hardware_interface","Unable to enable mra, retrying...");
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
        else
        {
            enabled = true;
        }
    }

    // Set callback for mra being disabled
    if(mra_basic_config::isDualArm){
        mra_util_.setDisabledCallback(boost::bind( &ArmInterface::robotDisabledCallback,  L_mra_hw_ ));
        mra_util_.setDisabledCallback(boost::bind( &ArmInterface::robotDisabledCallback,  R_mra_hw_ ));
    } else{
        mra_util_.setDisabledCallback(boost::bind( &ArmInterface::robotDisabledCallback,  mra_hw_ ));
    }

    // Create the controller manager
    ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    non_realtime_loop_ = nh_.createTimer(update_freq, &MRAHardwareInterface::update, this);

    ROS_INFO_NAMED("hardware_interface", "Loaded mra_hardware_interface.");
}

MRAHardwareInterface::~MRAHardwareInterface()
{
    //mra_util_.disableMRA();
}

bool MRAHardwareInterface::stateExpired()
{
    // Check that we have a non-expired state message
    // \todo lower the expiration duration
    if( ros::Time::now() > state_msg_timestamp_ + ros::Duration(STATE_EXPIRED_TIMEOUT)) // check that the message timestamp is no older than 1 second
    {

        ROS_WARN_STREAM_THROTTLE_NAMED(1,"hardware_interface","State expired. Last recieved state " << (ros::Time::now() - state_msg_timestamp_).toSec() << " seconds ago." );
        return true;
    }
    return false;
}

void MRAHardwareInterface::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    // Check if this message has the correct number of joints
    if( msg->name.size() != this->num_mra_joints)
    {
        ROS_ERROR("the size[%d] of joint state is not equal %d",msg->name.size(),this->num_mra_joints);
        return;
    }

    // Copy the latest message into a buffer
    state_msg_ = msg;
    state_msg_timestamp_ = ros::Time::now();
}

void MRAHardwareInterface::update(const ros::TimerEvent& e)
{
    // Check if state msg from mra is expired
    if( stateExpired() )
        return;

    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    if(mra_basic_config::isDualArm){
        // Input
        L_mra_hw_->read(state_msg_);
        R_mra_hw_->read(state_msg_);
        // Control
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        // Output
        L_mra_hw_->write(elapsed_time_);
        R_mra_hw_->write(elapsed_time_);
    } else{

        // Input
        mra_hw_->read(state_msg_);

        // Control
        controller_manager_->update(ros::Time::now(), elapsed_time_);

        // Output
        mra_hw_->write(elapsed_time_);
    }



}

} // namespace

int main(int argc, char** argv)
{
    ROS_INFO("\033[22;34m[hardware_interface]: Starting hardware interface... %s",Color_end);
    ros::init(argc, argv, "mra_hardware_interface");

    // Allow the action server to recieve and send ros messages(spin,spinOnce,AsyncSpinner,MultiThreadedSpinner)
    ros::AsyncSpinner spinner(4);//using 4 threads
    spinner.start();

    ros::NodeHandle nh;//node registration

    mra_control::MRAHardwareInterface *mra = new mra_control::MRAHardwareInterface();

    ros::spin();

    ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down.");

    return 0;
}



