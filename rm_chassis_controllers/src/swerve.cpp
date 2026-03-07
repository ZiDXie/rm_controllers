/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 4/23/21.
//

#include "rm_chassis_controllers/swerve.h"
#include "rm_common/math_utilities.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers
{
bool SwerveController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                            ros::NodeHandle& controller_nh)
{
  if (!ChassisBase::init(robot_hw, root_nh, controller_nh))
    return false;
  XmlRpc::XmlRpcValue modules;
  controller_nh.getParam("modules", modules);
  ROS_ASSERT(modules.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (const auto& module : modules)
  {
    ROS_ASSERT(module.second.hasMember("position"));
    ROS_ASSERT(module.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(module.second["position"].size() == 2);
    ROS_ASSERT(module.second.hasMember("pivot"));
    ROS_ASSERT(module.second["pivot"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(module.second.hasMember("wheel"));
    ROS_ASSERT(module.second["wheel"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(module.second["wheel"].hasMember("radius"));

    Module m{ .position_ = Vec2<double>((double)module.second["position"][0], (double)module.second["position"][1]),
              .pivot_offset_ = module.second["pivot"]["offset"],
              .wheel_radius_ = module.second["wheel"]["radius"],
              .ctrl_pivot_ = new effort_controllers::JointPositionController(),
              .ctrl_wheel_ = new effort_controllers::JointVelocityController() };
    ros::NodeHandle nh_pivot = ros::NodeHandle(controller_nh, "modules/" + module.first + "/pivot");
    ros::NodeHandle nh_wheel = ros::NodeHandle(controller_nh, "modules/" + module.first + "/wheel");
    if (!m.ctrl_pivot_->init(effort_joint_interface_, nh_pivot) ||
        !m.ctrl_wheel_->init(effort_joint_interface_, nh_wheel))
      return false;
    if (module.second["pivot"].hasMember("offset"))
      m.pivot_offset_ = module.second["pivot"]["offset"];
    pivot_joint_handles_.push_back(m.ctrl_pivot_->joint_);
    wheel_joint_handles_.push_back(m.ctrl_wheel_->joint_);
    modules_.push_back(m);
  }
  // power limit
  if (!controller_nh.getParam("power/pivot_max_power", pivot_max_power_) ||
      !controller_nh.getParam("power/pivot_effort_coeff", pivot_effort_coeff_) ||
      !controller_nh.getParam("power/pivot_velocity_coeff", pivot_velocity_coeff_))
  {
    ROS_ERROR("Some pivot power limit params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

  return true;
}

// Ref: https://dominik.win/blog/programming-swerve-drive/

void SwerveController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  Vec2<double> vel_center(vel_cmd_.x, vel_cmd_.y);
  for (auto& module : modules_)
  {
    Vec2<double> vel = vel_center + vel_cmd_.z * Vec2<double>(-module.position_.y(), module.position_.x());
    double vel_angle = std::atan2(vel.y(), vel.x()) + module.pivot_offset_;
    // Direction flipping and Stray module mitigation
    double a = angles::shortest_angular_distance(module.ctrl_pivot_->joint_.getPosition(), vel_angle);
    double b = angles::shortest_angular_distance(module.ctrl_pivot_->joint_.getPosition(), vel_angle + M_PI);
    module.ctrl_pivot_->setCommand(std::abs(a) < std::abs(b) ? vel_angle : vel_angle + M_PI);
    module.ctrl_wheel_->setCommand(vel.norm() / module.wheel_radius_ * std::cos(a));
    module.ctrl_pivot_->update(time, period);
    module.ctrl_wheel_->update(time, period);
  }
}

geometry_msgs::Twist SwerveController::odometry()
{
  geometry_msgs::Twist vel_data{};
  geometry_msgs::Twist vel_modules{};
  for (auto& module : modules_)
  {
    geometry_msgs::Twist vel;
    vel.linear.x = module.ctrl_wheel_->joint_.getVelocity() * module.wheel_radius_ *
                   std::cos(module.ctrl_pivot_->joint_.getPosition());
    vel.linear.y = module.ctrl_wheel_->joint_.getVelocity() * module.wheel_radius_ *
                   std::sin(module.ctrl_pivot_->joint_.getPosition());
    vel.angular.z =
        module.ctrl_wheel_->joint_.getVelocity() * module.wheel_radius_ *
        std::cos(module.ctrl_pivot_->joint_.getPosition() - std::atan2(module.position_.x(), -module.position_.y()));
    vel_modules.linear.x += vel.linear.x;
    vel_modules.linear.y += vel.linear.y;
    vel_modules.angular.z += vel.angular.z;
  }
  vel_data.linear.x = vel_modules.linear.x / modules_.size();
  vel_data.linear.y = vel_modules.linear.y / modules_.size();
  vel_data.angular.z =
      vel_modules.angular.z / modules_.size() /
      std::sqrt(std::pow(modules_.begin()->position_.x(), 2) + std::pow(modules_.begin()->position_.y(), 2));
  return vel_data;
}

void SwerveController::powerLimit()
{
  double power_limit = cmd_rt_buffer_.readFromRT()->cmd_chassis_.power_limit;
  double wheel_power_limit{};

  // Avoid too much power allocated to pivot joints
  double pivot_power_limit = std::min(power_limit / 2.0, pivot_max_power_);
  // Three coefficients of a quadratic equation for pivot joints
  double a_p{}, b_p{}, c_p = {};
  for (const auto& joint : pivot_joint_handles_)
  {
    double cmd_effort = joint.getCommand();
    double real_vel = joint.getVelocity();
    a_p += square(cmd_effort);
    b_p += cmd_effort * real_vel;
    c_p += square(real_vel);
  }
  a_p *= pivot_effort_coeff_;
  c_p = c_p * pivot_velocity_coeff_ - pivot_power_limit;
  // Root formula for quadratic equation in one variable
  double zoom_pivot =
      (square(b_p) - 4.0 * a_p * c_p) > 0.0 ? ((-b_p + sqrt(square(b_p) - 4.0 * a_p * c_p)) / (2.0 * a_p)) : 0.0;
  double actual_zoom = zoom_pivot > 1.0 ? 1.0 : zoom_pivot;
  double pivot_power = square(actual_zoom) * a_p + actual_zoom * b_p + (c_p + pivot_power_limit);
  for (auto joint : pivot_joint_handles_)
  {
    joint.setCommand(actual_zoom * joint.getCommand());
  }
  // Remaining power for wheels
  wheel_power_limit = power_limit + power_offset_ - pivot_power;

  // Three coefficients of a quadratic equation for wheel joints
  double a_w{}, b_w{}, c_w{};
  for (const auto& joint : wheel_joint_handles_)
  {
    double cmd_effort = joint.getCommand();
    double real_vel = joint.getVelocity();
    a_w += square(cmd_effort);
    b_w += cmd_effort * real_vel;
    c_w += square(real_vel);
  }
  a_w *= effort_coeff_;
  c_w = c_w * velocity_coeff_ - wheel_power_limit;
  // Root formula for quadratic equation in one variable
  double zoom_coeff = (square(b_w) - 4 * a_w * c_w) > 0 ? ((-b_w + sqrt(square(b_w) - 4 * a_w * c_w)) / (2 * a_w)) : 0.;
  for (auto joint : wheel_joint_handles_)
  {
    joint.setCommand(zoom_coeff > 1 ? joint.getCommand() : joint.getCommand() * zoom_coeff);
  }
}

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SwerveController, controller_interface::ControllerBase)
}  // namespace rm_chassis_controllers
