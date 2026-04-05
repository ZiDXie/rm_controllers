//
// Created by qiayuan on 2022/7/29.
//

#include <string>
#include <Eigen/QR>

#include <rm_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>

#include "rm_chassis_controllers/omni.h"

namespace rm_chassis_controllers
{
bool OmniController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  auto ewheel_power_publisher =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64>>(controller_nh, "power/ewheel_power", 100);
  this->ewheel_power_pub_ = std::move(ewheel_power_publisher);
  auto cwheel_power_publisher =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64>>(controller_nh, "power/cwheel_power", 100);
  this->cwheel_power_pub_ = std::move(cwheel_power_publisher);

  XmlRpc::XmlRpcValue wheels;
  controller_nh.getParam("wheels", wheels);
  chassis2joints_.resize(wheels.size(), 3);

  size_t i = 0;
  for (const auto& wheel : wheels)
  {
    ROS_ASSERT(wheel.second.hasMember("pose"));
    ROS_ASSERT(wheel.second["pose"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(wheel.second["pose"].size() == 3);
    ROS_ASSERT(wheel.second.hasMember("roller_angle"));
    ROS_ASSERT(wheel.second.hasMember("radius"));

    // Ref: Modern Robotics, Chapter 13.2: Omnidirectional Wheeled Mobile Robots
    Eigen::MatrixXd direction(1, 2), in_wheel(2, 2), in_chassis(2, 3);
    double beta = (double)wheel.second["pose"][2];
    double roller_angle = (double)wheel.second["roller_angle"];
    direction << 1, tan(roller_angle);
    in_wheel << cos(beta), sin(beta), -sin(beta), cos(beta);
    in_chassis << -(double)wheel.second["pose"][1], 1., 0., (double)wheel.second["pose"][0], 0., 1.;
    Eigen::MatrixXd chassis2joint = 1. / (double)wheel.second["radius"] * direction * in_wheel * in_chassis;
    chassis2joints_.block<1, 3>(i, 0) = chassis2joint;

    ros::NodeHandle nh_wheel = ros::NodeHandle(controller_nh, "wheels/" + wheel.first);
    joints_.push_back(std::make_shared<effort_controllers::JointVelocityController>());
    if (!joints_.back()->init(effort_joint_interface_, nh_wheel))
      return false;
    wheel_joint_handles_.push_back(joints_[i]->joint_);

    i++;
  }
  return true;
}

void OmniController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  Eigen::Vector3d vel_chassis;
  vel_chassis << vel_cmd_.z, vel_cmd_.x, vel_cmd_.y;
  Eigen::VectorXd vel_joints = chassis2joints_ * vel_chassis;
  for (size_t i = 0; i < joints_.size(); i++)
  {
    joints_[i]->setCommand(vel_joints(i));
    joints_[i]->update(time, period);
  }
}

geometry_msgs::Twist OmniController::odometry()
{
  Eigen::VectorXd vel_joints(joints_.size());
  for (size_t i = 0; i < joints_.size(); i++)
    vel_joints[i] = joints_[i]->joint_.getVelocity();
  Eigen::Vector3d vel_chassis =
      (chassis2joints_.transpose() * chassis2joints_).inverse() * chassis2joints_.transpose() * vel_joints;
  geometry_msgs::Twist twist;
  twist.angular.z = vel_chassis(0);
  twist.linear.x = vel_chassis(1);
  twist.linear.y = vel_chassis(2);
  return twist;
}

void OmniController::powerLimit()
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
    double wheel_zoom =
        (wheel_power_limitor_.K * abs(wheel_power_limitor_.err[i]) / wheel_power_limitor_.err_sum) +
        (1 - wheel_power_limitor_.K) * (abs(wheel_power_limitor_.power_in[i]) / wheel_power_limitor_.power_sum);
    wheel_zoom = limit(wheel_zoom, 0.0, 1.0);
    wheel_power_limitor_.power_limit[i] = wheel_zoom * wheel_power_limitor_.max_power;
  }
  if (wheel_power_limitor_.power_sum > wheel_power_limitor_.max_power)
  {
    for (size_t i = 0; i < joints_.size() && i < 4; ++i)
    {
      auto& ctl = joints_[i];
      auto& joint = ctl->joint_;
      double A = wheel_power_limitor_.effort_coeff;
      double B = wheel_power_limitor_.omiga[i] / 9.55f;
      double C = square(wheel_power_limitor_.omiga[i]) * wheel_power_limitor_.vel_coeff +
                 wheel_power_limitor_.power_offset - wheel_power_limitor_.power_limit[i];
      double Delta = square(B) - 4 * A * C;
      if (!std::isfinite(Delta) || Delta < 0.0)
        Delta = 0.0;
      if (Delta >= 0)
      {
        double Sqrt = sqrtf(Delta);
        double cmd{};
        ctl->getCommand(cmd);
        if (cmd >= 0)
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

void OmniController::updatePowerStatus()
{
  double power_limit = cmd_rt_buffer_.readFromRT()->cmd_chassis_.power_limit;
  const auto& power_config = *power_limit_rt_buffer_.readFromRT();

  wheel_power_limitor_ = {
    .vel_coeff = power_config.vel_coeff,
    .effort_coeff = power_config.effort_coeff,
    .power_offset = power_config.power_offset,
    .max_power = power_limit,
    .err_upper = 4000,
    .err_lower = 10,
  };

  double ewheel_power{}, cwheel_power{};
  for (size_t i = 0; i < joints_.size() && i < 4; ++i)
  {
    auto& ctl = joints_[i];
    double cmd_torque = wheel_power_limitor_.torque[i] = ctl->joint_.getCommand();
    double cmd_vel{};
    ctl->getCommand(cmd_vel);
    double real_vel = wheel_power_limitor_.omiga[i] = ctl->joint_.getVelocity();
    double real_torque = ctl->joint_.getEffort();
    wheel_power_limitor_.err[i] = cmd_vel - real_vel;
    wheel_power_limitor_.err_sum += abs(wheel_power_limitor_.err[i]);
    wheel_power_limitor_.power_in[i] = real_torque * real_vel / 9.55f +
                                       wheel_power_limitor_.effort_coeff * square(real_torque) +
                                       wheel_power_limitor_.vel_coeff * square(real_vel);
    ewheel_power += wheel_power_limitor_.power_in[i];
    cwheel_power += cmd_torque * real_vel / 9.55f + wheel_power_limitor_.effort_coeff * square(cmd_torque) +
                    wheel_power_limitor_.vel_coeff * square(real_vel);
  }
  wheel_power_limitor_.power_sum = ewheel_power + wheel_power_limitor_.power_offset;

  // Publish power status.
  auto publishPower = [](auto& pub, const double power) {
    if (pub && pub->trylock())
    {
      pub->msg_.data = power;
      pub->unlockAndPublish();
    }
  };
  publishPower(ewheel_power_pub_, ewheel_power);
  publishPower(cwheel_power_pub_, cwheel_power);
}

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::OmniController, controller_interface::ControllerBase)
