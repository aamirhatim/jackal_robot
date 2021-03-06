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

  // Realtime publisher, initializes differently from regular ros::Publisher
  cmd_drive_pub_.init(nh_, "cmd_drive", 1);

  // Custom config
  acc_rate_ = 0.08;
  connected_ = true;
  cmd_vel_reached_ = true;
  time_last_connected_ = ros::Time::now();

  heartbeat_sub_ = nh_.subscribe("/mec_connection", 1, &JackalHardware::heartbeatCallback, this);
  user_cmd_sub_ = nh_.subscribe("user_cmd", 1, &JackalHardware::updateCommandCallback, this);
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
    // Check if elapsed time is greater than timeout
    if (!connected_)
    {
      double *acc;
      acc = getAcceleration(0.0);

      // Set left and right speeds
      left_vel_ += acc[0];
      right_vel_ += acc[1];
    } else if (!cmd_vel_reached_)
    {
      const double cmd = user_cmd_.linear.x * 10;
      double *acc;
      acc = getAcceleration(cmd);

      // Set left and right speeds
      left_vel_ += acc[0];
      right_vel_ += acc[1];
    } else
    {
      left_vel_ = joints_[0].velocity_command;
      right_vel_ = joints_[1].velocity_command;
    }

    // Publish drive command
    cmd_drive_pub_.msg_.mode = jackal_msgs::Drive::MODE_VELOCITY;
    cmd_drive_pub_.msg_.drivers[jackal_msgs::Drive::LEFT] = left_vel_;
    cmd_drive_pub_.msg_.drivers[jackal_msgs::Drive::RIGHT] = right_vel_;
    cmd_drive_pub_.unlockAndPublish(); 
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
  time_last_connected_ = ros::Time::now();
}

void JackalHardware::updateCommandCallback(const geometry_msgs::Twist& msg)
{
  user_cmd_ = msg;
}

void JackalHardware::checkTimeout()
{
  // Get current time
  ros::Time time_now = ros::Time::now();

  // Get elapsed time since last heartbeat
  double time_elapsed = time_now.toSec() - time_last_connected_.toSec();

  // Check if elapsed time is greater than timeout
  if (time_elapsed > 0.2)
  {
    // Switch status to disconnected and reset cmd_vel_reached flag
    if (connected_)
    {
      connected_ = false;
      cmd_vel_reached_ = false;
    }
  } else
  {
    connected_ = true;
  }
}

double* JackalHardware::getAcceleration(const double cmd_desired)
{
  // If desired and actual speed are close enough to each other, set cmd_vel_reached_ flag to true
  double delta_left = cmd_desired - left_vel_;
  double delta_right = cmd_desired - right_vel_;
      
  // Calculate acceleration needed to get current actual speed to current desired speed
  double acc_left = delta_left / 50.0;
  double acc_right = delta_right / 50.0;

  // Saturate acceleration if needed and add that to current speed
  if (fabs(acc_left) > acc_rate_)
  {
    acc_left = (fabs(acc_left) / acc_left) * acc_rate_;
  }
  if (fabs(acc_right) > acc_rate_)
  {
    acc_right = (fabs(acc_right) / acc_right) * acc_rate_;
  }

  static double acc[2];
  acc[0] = acc_left;
  acc[1] = acc_right;

  // If reconnecting, check if desired cmd_vel has been reached
  if (!cmd_vel_reached_ && connected_)
  {
    if (fabs(delta_left) <= 0.5 && fabs(delta_right) <= 0.5)
    {
      cmd_vel_reached_ = true;
    }
  }
  return acc;
}

}  // namespace jackal_base
