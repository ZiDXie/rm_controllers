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

  // 0 for pivot, 1 for wheel. Each module has 4 joints at most.
  for (auto& filter_group : motor_lp_filters_)
  {
    for (auto& filter : filter_group)
      filter = new LowPassFilter(20);
  }

  auto epower_publisher =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64>>(controller_nh, "power/estimated", 100);
  this->epower_pub_ = std::move(epower_publisher);
  auto cpower_publisher =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64>>(controller_nh, "power/commanded", 100);
  this->cpower_pub_ = std::move(cpower_publisher);
  auto base_gyro_publisher = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>>(
      controller_nh, "base_gyro", 100);
  this->base_gyro_pub_ = std::move(base_gyro_publisher);

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

// Ref: https://gitee.com/cod_-control/rmcod2026_-sentry/tree/dev
// https://github.com/hkustenterprize/RM2024-PowerModule

void SwerveController::powerLimit()
{
  updatePowerStatus();

  // multiply K to limit power for wheel joints.
  if (wheel_power_limitor_.err_sum > wheel_power_limitor_.err_upper)
  {
    wheel_power_limitor_.K = 1;
  }
  else if (wheel_power_limitor_.err_sum < wheel_power_limitor_.err_lower)
  {
    wheel_power_limitor_.K = 0;
  }
  else
  {
    wheel_power_limitor_.K = 1 - (wheel_power_limitor_.err_sum - wheel_power_limitor_.err_lower) /
                                     (wheel_power_limitor_.err_upper - wheel_power_limitor_.err_lower);
  }
  // Set power limit to each joint according to K.
  for (int i = 0; i < 4; i++)
  {
    double pivot_zoom = abs(pivot_power_limitor_.power_in[i]) / pivot_power_limitor_.power_sum;
    pivot_zoom = limit(pivot_zoom, 0.0, 1.0);
    double wheel_zoom =
        (wheel_power_limitor_.K * abs(wheel_power_limitor_.err[i]) / wheel_power_limitor_.err_sum) +
        (1 - wheel_power_limitor_.K) * (abs(wheel_power_limitor_.power_in[i]) / wheel_power_limitor_.power_sum);
    wheel_zoom = limit(wheel_zoom, 0.0, 1.0);
    pivot_power_limitor_.power_limit[i] = pivot_zoom * pivot_power_limitor_.max_power;
    wheel_power_limitor_.power_limit[i] = wheel_zoom * wheel_power_limitor_.max_power;
  }
  // Set command limit according to power limit.
  if (pivot_power_limitor_.power_sum > pivot_power_limitor_.max_power)
  {
    for (size_t i = 0; i < modules_.size() && i < 4; ++i)
    {
      auto& module = modules_[i];
      auto& joint = module.ctrl_pivot_->joint_;
      double A = pivot_power_limitor_.effort_coeff;
      double B = pivot_power_limitor_.omiga[i];
      double C = square(pivot_power_limitor_.omiga[i]) * pivot_power_limitor_.vel_coeff +
                 pivot_power_limitor_.power_offset / 4 - pivot_power_limitor_.power_limit[i];
      double Delta = square(B) - 4 * A * C;
      if (!std::isfinite(Delta) || Delta < 0.0)
        Delta = 0.0;
      if (Delta >= 0)
      {
        double Sqrt = sqrtf(Delta);
        if (pivot_power_limitor_.torque[i] >= 0)
          joint.setCommand((-B + Sqrt) / (2 * A));
        else
          joint.setCommand((-B - Sqrt) / (2 * A));
      }
      else
      {
        joint.setCommand((-B) / (2 * A));
      }
    }
  }
  if (wheel_power_limitor_.power_sum > wheel_power_limitor_.max_power)
  {
    for (size_t i = 0; i < modules_.size() && i < 4; ++i)
    {
      auto& module = modules_[i];
      auto& joint = module.ctrl_wheel_->joint_;
      double A = wheel_power_limitor_.effort_coeff;
      double B = wheel_power_limitor_.omiga[i];
      double C = square(wheel_power_limitor_.omiga[i]) * wheel_power_limitor_.vel_coeff +
                 wheel_power_limitor_.power_offset / 4 - wheel_power_limitor_.power_limit[i];
      double Delta = square(B) - 4 * A * C;
      if (!std::isfinite(Delta) || Delta < 0.0)
        Delta = 0.0;
      if (Delta >= 0)
      {
        double Sqrt = sqrtf(Delta);
        if (wheel_power_limitor_.torque[i] >= 0)
          joint.setCommand((-B + Sqrt) / (2 * A));
        else
          joint.setCommand((-B - Sqrt) / (2 * A));
      }
      else
      {
        joint.setCommand((-B) / (2 * A));
      }
    }
  }
}

void SwerveController::updatePowerStatus()
{
  double power_limit = cmd_rt_buffer_.readFromRT()->cmd_chassis_.power_limit;
  const auto& power_config = *power_limit_rt_buffer_.readFromRT();

  pivot_power_limitor_ = { .vel_coeff = power_config.pivot_vel_coeff,
                           .effort_coeff = power_config.pivot_effort_coeff,
                           .power_offset = 0,
                           .max_power =
                               std::min(power_config.pivot_max_power, power_limit * power_config.pivot_power_ratio),
                           .ratio = power_config.pivot_power_ratio };

  double epivot_power{}, cpivot_power{};
  for (size_t i = 0; i < modules_.size() && i < 4; ++i)
  {
    auto& module = modules_[i];

    double cmd_torque = pivot_power_limitor_.torque[i] = module.ctrl_pivot_->joint_.getCommand();
    double real_vel = pivot_power_limitor_.omiga[i] = module.ctrl_pivot_->joint_.getVelocity();
    motor_lp_filters_[0][i]->input(module.ctrl_pivot_->joint_.getEffort());
    double real_torque = motor_lp_filters_[0][i]->output();

    epivot_power += real_torque * real_vel + pivot_power_limitor_.effort_coeff * square(real_torque) +
                    pivot_power_limitor_.vel_coeff * square(real_vel);
    cpivot_power += cmd_torque * real_vel + pivot_power_limitor_.effort_coeff * square(cmd_torque) +
                    pivot_power_limitor_.vel_coeff * square(real_vel);

    pivot_power_limitor_.power_in[i] = cpivot_power;
  }

  pivot_power_limitor_.power_sum = cpivot_power + pivot_power_limitor_.power_offset;

  wheel_power_limitor_ = { .vel_coeff = power_config.vel_coeff,
                           .effort_coeff = power_config.effort_coeff,
                           .power_offset = power_config.power_offset,
                           .max_power = power_limit - std::abs(pivot_power_limitor_.power_sum),
                           .err_upper = 1e8,
                           .err_lower = 1e6,
                           .err_sum = 0 };

  double ewheel_power{}, cwheel_power{};
  for (size_t i = 0; i < modules_.size() && i < 4; ++i)
  {
    auto& module = modules_[i];

    double cmd_torque = wheel_power_limitor_.torque[i] = module.ctrl_wheel_->joint_.getCommand();
    double cmd_vel{};
    module.ctrl_wheel_->getCommand(cmd_vel);
    double real_vel = wheel_power_limitor_.omiga[i] = module.ctrl_wheel_->joint_.getVelocity();
    motor_lp_filters_[1][i]->input(module.ctrl_wheel_->joint_.getEffort());
    double real_torque = motor_lp_filters_[1][i]->output();

    wheel_power_limitor_.err[i] = cmd_vel - real_vel;
    wheel_power_limitor_.err_sum += abs(wheel_power_limitor_.err[i]);

    ewheel_power += real_torque * real_vel + wheel_power_limitor_.effort_coeff * square(real_torque) +
                    wheel_power_limitor_.vel_coeff * square(real_vel);
    cwheel_power += cmd_torque * real_vel + wheel_power_limitor_.effort_coeff * square(cmd_torque) +
                    wheel_power_limitor_.vel_coeff * square(real_vel);

    wheel_power_limitor_.power_in[i] = cwheel_power;
  }
  wheel_power_limitor_.power_sum = cwheel_power + wheel_power_limitor_.power_offset;

  // Publish power status.
  auto publishPower = [](auto& pub, const double power) {
    if (pub && pub->trylock())
    {
      pub->msg_.data = power;
      pub->unlockAndPublish();
    }
  };

  publishPower(epower_pub_, epivot_power + ewheel_power);
  publishPower(cpower_pub_, cpivot_power + cwheel_power);
}

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SwerveController, controller_interface::ControllerBase)
}  // namespace rm_chassis_controllers
