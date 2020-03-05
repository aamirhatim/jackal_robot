/**
 *
 *  \file
 *  \brief      Class representing Jackal hardware
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <boost/assign.hpp>
#include "jackal_base/jackal_hardware.h"
#include <math.h>
#include <ros/master.h>

namespace jackal_base
{

JackalHardware::JackalHardware()
{
  ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");

  for (unsigned int i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  feedback_sub_ = nh_.subscribe("feedback", 1, &JackalHardware::feedbackCallback, this);

  // Heartbeat subscriber
  connected_ = false;
  time_last_connected_ = ros::Time::now();
  heartbeat_sub_ = nh_.subscribe("/mec_connection", 1, &JackalHardware::heartbeatCallback, this);

  // Realtime publisher, initializes differently from regular ros::Publisher
  cmd_drive_pub_.init(nh_, "cmd_drive", 1);
}

/**
 * Populates the internal joint state struct from the most recent Feedback message
 * received from the MCU.
 *
 * Called from the controller thread.
 */
void JackalHardware::copyJointsFromHardware()
{
  boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  if (feedback_msg_ && feedback_msg_lock)
  {
    for (int i = 0; i < 4; i++)
    {
      joints_[i].position = feedback_msg_->drivers[i % 2].measured_travel;
      joints_[i].velocity = feedback_msg_->drivers[i % 2].measured_velocity;
      joints_[i].effort = 0;  // TODO(mikepurvis): determine this from amperage data.
    }
  }
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void JackalHardware::publishDriveFromController()
{
  if (cmd_drive_pub_.trylock())
  {
    // Get current time
    ros::Time time_now = ros::Time::now();

    // Get elapsed time since last heartbeat
    double time_elapsed = time_now.toSec() - time_last_connected_.toSec();
    // std::cout << time_elapsed << std::endl;

    // Check if elapsed time is greater than timeout
    if (time_elapsed > 0.5)
    {
      std::cout << 'disconnect' << std::endl;
      std::cout << vels << std::endl;
      // Get current velocities
      double left_vel = vels / 10.0;
      double right_vel = vels / 10.0;
      // std::cout << left_vel << std::endl;

      // Calculate deceleration
      double v_left = std::max(0.0, fabs(left_vel) - 0.002);
      // std::cout << v_left << std::endl;
      double v_right = std::max(0.0, fabs(right_vel) - 0.002);
      if (left_vel < 0.0)
      {
        v_left = -v_left;
      }
      if (right_vel < 0.0)
      {
        v_right = -v_right;
      }

      // Create Drive message
      // double v = sin(time_now.toSec());
      // std::cout << v_left << std::endl << std::endl;
      cmd_drive_pub_.msg_.mode = jackal_msgs::Drive::MODE_VELOCITY;
      cmd_drive_pub_.msg_.drivers[jackal_msgs::Drive::LEFT] = v_left * 10;
      cmd_drive_pub_.msg_.drivers[jackal_msgs::Drive::RIGHT] = v_right * 10;
      cmd_drive_pub_.unlockAndPublish();
      // std::cout << "timeout" << std::endl;
      // std::cout << v_left * 10.0 << std::endl << std::endl;

      vels = v_left;
      // std::cout << vels << std::endl;
    } else
    {
      cmd_drive_pub_.msg_.mode = jackal_msgs::Drive::MODE_VELOCITY;
      cmd_drive_pub_.msg_.drivers[jackal_msgs::Drive::LEFT] = joints_[0].velocity_command;
      cmd_drive_pub_.msg_.drivers[jackal_msgs::Drive::RIGHT] = joints_[1].velocity_command;
      cmd_drive_pub_.unlockAndPublish();
      // std::cout << "connected" << std::endl;
    }    
  }
}

/**
 * Populates and publishes Drive message based on a slow deceleration speed profile.
 * Run only when remote connection to master is lost.
 *
 * Called from the controller thread.
 */
void JackalHardware::publishSafeStop()
{
  if (cmd_drive_pub_.trylock())
  {
    // Get current velocities
    double left_vel = joints_[0].velocity / 10.0;
    double right_vel = joints_[1].velocity / 10.0;

    // Calculate deceleration
    double v_left = std::max(0.0, fabs(left_vel) - (1.5/50.0));
    double v_right = std::max(0.0, fabs(right_vel) - (1.5/50.0));
    if (left_vel < 0.0)
    {
      v_left = -v_left;
    }
    if (right_vel < 0.0)
    {
      v_right = -v_right;
    }
    // std::cout << sign_right << std::endl;

    // Create Drive message
    cmd_drive_pub_.msg_.mode = jackal_msgs::Drive::MODE_VELOCITY;
    cmd_drive_pub_.msg_.drivers[jackal_msgs::Drive::LEFT] = v_left * 10.0;
    cmd_drive_pub_.msg_.drivers[jackal_msgs::Drive::RIGHT] = v_right * 10.0;

    // Publish
    // cmd_drive_pub_.unlockAndPublish();
  }
}

void JackalHardware::feedbackCallback(const jackal_msgs::Feedback::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  feedback_msg_ = msg;
}

void JackalHardware::heartbeatCallback(const std_msgs::Empty::ConstPtr& msg)
{
  // connected_ = true;
  time_last_connected_ = ros::Time::now();

  // Get current vels and add to buffer
  vel_buffer[2] = vel_buffer[1];
  vel_buffer[1] = vel_buffer[0];
  vel_buffer[0] = joints_[0].velocity;

  vels = (vel_buffer[0] + vel_buffer[1] + vel_buffer[2]) / 3.0;
  std::cout << vels << std::endl;
}

bool JackalHardware::checkTimeout()
{
  bool timeout = false;

  // Get current time
  ros::Time time_now = ros::Time::now();

  // Get elapsed time since last heartbeat
  double time_elapsed = time_now.toSec() - time_last_connected_.toSec();
  // std::cout << time_elapsed << std::endl;

  // Check if elapsed time is greater than timeout
  if (time_elapsed > 0.5)
  {
    timeout = true;
  }

  return timeout;
}

}  // namespace jackal_base
