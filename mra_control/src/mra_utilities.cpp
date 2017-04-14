/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, AUBO Inc.
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
 * \Author: LMN(refer to baxter_cpp by Dave Coleman)
 */

#include <mra_control/mra_utilities.h>

namespace mra_control
{
MRAUtilities::MRAUtilities()
  : disabled_callback_called_(false),
    state_counter_(1)
{
  ROS_INFO_STREAM_NAMED("mra_utilities","Loading mra utilities");
  ros::NodeHandle nh;

  // Preload Messages
  disable_msg_.data = false;
  enable_msg_.data = true;

  // ---------------------------------------------------------------------------------------------
  // Advertise services
  pub_mra_enable_ = nh.advertise<std_msgs::Bool>("/robot/set_super_enable",10);
  pub_mra_reset_ = nh.advertise<std_msgs::Empty>("reset_MRA_API",10);
  ROS_DEBUG_STREAM_NAMED("mra_utilities","Publishing on topics /robot/set_super_enable and robot/set_super_reset");

  // ---------------------------------------------------------------------------------------------
  // Start the state subscriber
  sub_mra_state_ = nh.subscribe<mra_core_msgs::AssemblyState>(MRA_STATE_TOPIC,
                      1, &MRAUtilities::stateCallback, this);

}


void MRAUtilities::setDisabledCallback(DisabledCallback callback)
{
  disabled_callback_ = callback;
}

bool MRAUtilities::communicationActive()
{
  int count = 0;
  while( ros::ok() && mra_state_timestamp_.toSec() == 0 )
  {
    if( count > 40 ) // 40 is an arbitrary number for when to assume no state is being published
    {
      ROS_WARN_STREAM_NAMED("utilities","No state message has been recieved on topic "
        << MRA_STATE_TOPIC);
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }

  // Check that the message timestamp is no older than 1 second
  if(ros::Time::now() > mra_state_timestamp_ + ros::Duration(1.0))
  {
    ROS_ERROR_STREAM_NAMED("utilities","mra state expired.");
    return false;
  }

  return true;
}

bool MRAUtilities::isEnabled(bool verbose)
{
  // Check communication
  if( !communicationActive() )
  {
    // Error message aready outputed
    return false;
  }


  // Check for error
  if( mra_state_->error == true )
  {
    if(verbose)
      ROS_ERROR_STREAM_NAMED("utilities","mra has an error :(  State: \n" << *mra_state_ );
    return false;
  }

  // Check enabled
  if( mra_state_->enabled == false )
  {
    if(verbose)
      ROS_ERROR_STREAM_NAMED("utilities","mra is not enabled.  State: \n" << *mra_state_ );

    return false;
  }

  return true;
}

void MRAUtilities::stateCallback(const mra_core_msgs::AssemblyStateConstPtr& msg)
{
  mra_state_ = msg;
  mra_state_timestamp_ = ros::Time::now();

  // Check for errors every CHECK_FREQ refreshes to save computation
  static const int CHECK_FREQ = 25; //50;
  if( state_counter_ % CHECK_FREQ == 0 )
  {
    if( !isEnabled() ) // mra is disabled
    {
      if (disabled_callback_called_ == false)
      {
        // Call the parent classes' callback if they've provided one
        if (disabled_callback_)
          disabled_callback_();

        disabled_callback_called_ = true;
      }
    }
    else // mra is not disabled
    {
      disabled_callback_called_ = false;
    }

    // Reset the counter so it doesn't overflow
    state_counter_ = 0;
  }

  state_counter_++;
}


bool MRAUtilities::enableMRA()
{
  ROS_INFO_STREAM_NAMED("utility","Enabling mra");

  // Check if we need to do anything
  if( isEnabled(false) )
    return true;

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  // Reset mra
  if( !resetMRA() )
    return false;

  // Attempt to enable mra
  pub_mra_enable_.publish(enable_msg_);
  ros::Duration(0.5).sleep();

  // Check if enabled
  int count = 0;
  while( ros::ok() && !isEnabled(true) )
  {
    if( count > 20 ) // 20 is an arbitrary number for when to assume its not going to enable
    {
      ROS_ERROR_STREAM_NAMED("utilities","Giving up on waiting");
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }

  return true;
}

bool MRAUtilities::disableMRA()
{
  ROS_INFO_STREAM_NAMED("utility","Disabling mra");

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  pub_mra_enable_.publish(disable_msg_);
  ros::Duration(0.5).sleep();

  // Check it enabled
  int count = 0;
  while( ros::ok() && mra_state_->enabled == true )
  {
    if( count > 20 ) // 20 is an arbitrary number for when to assume its not going to enable
    {
      ROS_ERROR_STREAM_NAMED("utilities","Failed to disable mra");
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }

  return true;
}

bool MRAUtilities::resetMRA()
{
  ROS_INFO_STREAM_NAMED("utility","Resetting mra");

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  // Attempt to reset and enable robot
  pub_mra_reset_.publish(empty_msg_);
  ros::Duration(0.5).sleep();

  return true;
}



} //namespace

