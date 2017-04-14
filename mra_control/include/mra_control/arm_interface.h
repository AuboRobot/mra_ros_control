/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Author: Dave Coleman
   Desc:   ros_control hardware interface layer for mra
*/

#ifndef MRA_CONTROL__ARM_INTERFACE_
#define MRA_CONTROL__ARM_INTERFACE_

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_mode_interface.h>

// mra
#include <mra_core_msgs/JointCommand.h>
#include <mra_core_msgs/JointCommand.h>

#include <vector>
#include <iostream>

namespace mra_control
{

enum MRAControlMode { POSITION, VELOCITY, TORQUE };

class ArmInterface
{
protected:

  // Node Handles
  ros::NodeHandle nh_; // no namespace



  // Track current hardware interface mode we are in
  int* joint_mode_;

  // Speed of hardware loop
  double loop_hz_;


public:

  // Number of joints we are using
  unsigned int n_dof_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_velocity_command_;

  /**
   * \brief Constructor/Descructor
   */
  ArmInterface(const std::vector<std::string> &arm_name, double loop_hz)
    :loop_hz_(loop_hz)
  {};

  ~ArmInterface()
  {};

  /**
   * \brief Initialice hardware interface
   * \return false if an error occurred during initialization
   */
  virtual bool init(
    hardware_interface::JointStateInterface&    js_interface,
    hardware_interface::EffortJointInterface&   ej_interface,
    hardware_interface::VelocityJointInterface& vj_interface,
    hardware_interface::PositionJointInterface& pj_interface,
    int* joint_mode,
    sensor_msgs::JointStateConstPtr state_msg
  )
  { return true; };

  /**
   * \brief Copy the joint state message into our hardware interface datastructures
   */
  virtual void read( sensor_msgs::JointStateConstPtr &state_msg )
  {};

  /**
   * \brief Publish our hardware interface datastructures commands to mra hardware
   */
  virtual void write(ros::Duration elapsed_time)
  {};

  /**
   * \brief This is called when mra is disabled, so that we can update the desired positions
   */
  virtual void robotDisabledCallback()
  {};

};

typedef boost::shared_ptr<mra_control::ArmInterface> ArmInterfacePtr;
typedef boost::shared_ptr<const mra_control::ArmInterface> ArmInterfaceConstPtr;

} // namespace

#endif
