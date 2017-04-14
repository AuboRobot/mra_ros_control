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
   Desc:   ros_control hardware interface for the MRA
*/

#include <mra_control/arm_hardware_interface.h>
#include <mra_basic/config.h>

namespace mra_control
{

ArmHardwareInterface::ArmHardwareInterface(const std::vector<std::string> &joint_names,
                                           double loop_hz,
                                           std::string joint_command_topic)
    : ArmInterface(joint_names, loop_hz)
{
    // Populate joints in this arm
    joint_names_ = joint_names;
    n_dof_ = joint_names_.size();
    joint_command_topic_ = joint_command_topic;

    first_read = true;

    // Resize vectors
    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_position_command_.resize(n_dof_);
    joint_effort_command_.resize(n_dof_);
    joint_velocity_command_.resize(n_dof_);
    output_msg_.command.resize(n_dof_);
    output_msg_.names.resize(n_dof_);
    trajectory_command_msg_.joint_names.resize(n_dof_);
    joint_id_to_joint_states_id_.resize(n_dof_);

    for (std::size_t i = 0; i < n_dof_; ++i)
    {
        joint_position_[i] = 0.0;
        joint_velocity_[i] = 0.0;
        joint_effort_[i] = 0.0;
        joint_position_command_[i] = 0.0;
        joint_effort_command_[i] = 0.0;
        joint_velocity_command_[i] = 0.0;
        trajectory_command_msg_.joint_names[i] = joint_names_[i];
    }

    // Set trajectory to have two point
    trajectory_msgs::JointTrajectoryPoint single_pt;
    single_pt.positions.resize(n_dof_);
    single_pt.time_from_start = ros::Duration(0);
    trajectory_command_msg_.points.push_back(single_pt);

    trajectory_msgs::JointTrajectoryPoint single_pt2;
    single_pt2.positions.resize(n_dof_);
    single_pt2.time_from_start = ros::Duration(0.5);
    trajectory_command_msg_.points.push_back(single_pt2);
}

ArmHardwareInterface::~ArmHardwareInterface()
{
}

bool ArmHardwareInterface::init(
        hardware_interface::JointStateInterface&    js_interface,
        hardware_interface::EffortJointInterface&   ej_interface,
        hardware_interface::VelocityJointInterface& vj_interface,
        hardware_interface::PositionJointInterface& pj_interface,
        int* joint_mode,
        sensor_msgs::JointStateConstPtr state_msg)
{
    joint_mode_ = joint_mode;

    for (std::size_t i = 0; i < n_dof_; ++i)
    {
        // Create joint state interface for all joints
        js_interface.registerHandle(hardware_interface::JointStateHandle(
                                        joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

        // Create position joint interface
        pj_interface.registerHandle(hardware_interface::JointHandle(
                                        js_interface.getHandle(joint_names_[i]),&joint_position_command_[i]));

        // Create velocity joint interface
        vj_interface.registerHandle(hardware_interface::JointHandle(
                                        js_interface.getHandle(joint_names_[i]),&joint_velocity_command_[i]));

        // Create effort joint interface
        ej_interface.registerHandle(hardware_interface::JointHandle(
                                        js_interface.getHandle(joint_names_[i]),&joint_effort_command_[i]));
    }

    // Start publishers
    pub_joint_command_ = nh_.advertise<mra_core_msgs::JointCommand>(joint_command_topic_,10);

    pub_trajectory_command_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mra/_trajectory_controller/command",10);

    // Make a mapping of joint names to indexes in the joint_states message
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
        std::vector<std::string>::const_iterator iter = std::find(state_msg->name.begin(), state_msg->name.end(), joint_names_[i]);
        size_t joint_states_id = std::distance(state_msg->name.begin(), iter);
        if(joint_states_id == state_msg->name.size())
        {
            ROS_ERROR_STREAM_NAMED("arm","Unable to find joint " << i << " named " << joint_names_[i] << " in joint state message");
        }

        joint_id_to_joint_states_id_[i] = joint_states_id;

        ROS_DEBUG_STREAM_NAMED("arm_hardware_interface","Found joint " << i << " at " << joint_states_id << " named " << joint_names_[i]);
    }

    // Set the initial command values based on current state
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
        joint_position_command_[i] = state_msg->position[joint_id_to_joint_states_id_[i]];

        // Pre-load the joint names into the output messages just once
        output_msg_.names[i] = joint_names_[i];
    }

    ROS_INFO_NAMED("arm", "Loaded mra_hardware_interface.");
    return true;
}

void ArmHardwareInterface::read( sensor_msgs::JointStateConstPtr &state_msg )
{
    // Copy state message to our datastructures
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
        //ROS_INFO_STREAM_NAMED("arm_hardware_interface","Joint " << i << "("<< joint_names_[i] << ") -> " << joint_id_to_joint_states_id_[i] << " position= " << state_msg->position[joint_id_to_joint_states_id_[i]]);
        joint_position_[i] = state_msg->position[joint_id_to_joint_states_id_[i]];
        joint_velocity_[i] = state_msg->velocity[joint_id_to_joint_states_id_[i]];
        joint_effort_[i] = state_msg->effort[joint_id_to_joint_states_id_[i]];
    }
    if(first_read == true){
        joint_position_command_ = joint_position_;
        joint_velocity_command_ = joint_velocity_;
        joint_effort_command_ = joint_effort_;
        first_read = false;

    }
}

void ArmHardwareInterface::write(ros::Duration elapsed_time)
{
    // Send commands to MRA in different modes
    switch (*joint_mode_)
    {
    case hardware_interface::MODE_POSITION:
        output_msg_.command = joint_position_command_;
        output_msg_.mode = mra_core_msgs::JointCommand::POSITION_MODE;
        break;
    case hardware_interface::MODE_VELOCITY:
        output_msg_.command = joint_velocity_command_;
        output_msg_.mode = mra_core_msgs::JointCommand::VELOCITY_MODE;
        break;
    case hardware_interface::MODE_EFFORT:
        output_msg_.command = joint_effort_command_;
        output_msg_.mode = mra_core_msgs::JointCommand::TORQUE_MODE;
        break;
    }

    // Publish
    pub_joint_command_.publish(output_msg_);
}

void ArmHardwareInterface::publishCurrentLocation()
{
    // Publish this new trajectory just once, on cuff release
    ROS_INFO_STREAM_NAMED("arm", "Sent updated trajectory to trajectory controller");

    // Update the trajectory message with the current positions
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
        trajectory_command_msg_.points[0].positions[i] = joint_position_[i];
        trajectory_command_msg_.points[1].positions[i] = joint_position_[i];
    }

    // Send a trajectory
    pub_trajectory_command_.publish(trajectory_command_msg_);
}

void ArmHardwareInterface::robotDisabledCallback()
{
    publishCurrentLocation();
}

} // namespace
