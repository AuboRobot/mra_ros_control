/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, CU Boulder
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
 *   * Neither the name of CU Boulder nor the names of its
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

/**
 * \brief   Helper functions for controlling mra
 * \author  Dave Coleman
 */

#ifndef MRA_CONTROL__MRA_UTILITIES_
#define MRA_CONTROL__MRA_UTILITIES_

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// Boost
#include <boost/scoped_ptr.hpp>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kinematic_constraints/utils.h>

// Msgs
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <mra_core_msgs/AssemblyState.h>
#include <moveit_msgs/MoveGroupGoal.h>
#include <moveit_msgs/MoveGroupAction.h>


namespace mra_control
{

static const std::string MRA_STATE_TOPIC = "/mra/state";

class MRAUtilities
{
public:

  // ROS Messages
  ros::Publisher pub_mra_enable_; //when MRA meet some troubles disable it etc.
  ros::Publisher pub_mra_reset_;
  ros::Subscriber sub_mra_state_;

  // Remember the last mra state and time
  mra_core_msgs::AssemblyStateConstPtr mra_state_;
  ros::Time mra_state_timestamp_;

  // Cache messages
  std_msgs::Bool enable_msg_;
  std_msgs::Bool disable_msg_;
  std_msgs::Empty empty_msg_;

  std::size_t state_counter_;
  bool disabled_callback_called_;

  // Optional function callback for when mra is disabled
  typedef boost::function<void ()> DisabledCallback;
  DisabledCallback disabled_callback_; //   f1( boost::bind( &myclass::fun1, this ) );
  
  MRAUtilities();
  
  /**
   * \brief Allow classes that uses MRAUtilities to add a hook for when mra is disabled
   * \param callback - the function to call when mra is disabled
   */
  void setDisabledCallback(DisabledCallback callback);

  /**
   * \brief Wait for initial state to be recieved from mra
   * \return true if communication is ok
   */
  bool communicationActive();

  /**
   * \brief Check if there is no error, is not stopped, and is enabled
   * \return true if mra is ready to use
   */
  bool isEnabled(bool verbose = false);

  void stateCallback(const mra_core_msgs::AssemblyStateConstPtr& msg);

  bool enableMRA();

  bool disableMRA();

  bool resetMRA();

};

} //namespace

#endif
