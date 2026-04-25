//
// Created by qiayuan on 2022/7/29.
//

#include "rm_chassis_controllers/active_suspension.h"

namespace rm_chassis_controllers
{
bool ActiveSuspensionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                      ros::NodeHandle& controller_nh)
{
  if (!OmniController::init(robot_hw, root_nh, controller_nh))
    return false;

  auto pid_init = [&](const std::string& ns, std::shared_ptr<control_toolbox::Pid>& pid) {
    if (controller_nh.hasParam(ns))
    {
      pid = std::make_shared<control_toolbox::Pid>();
      if (!pid->init(ros::NodeHandle(controller_nh, ns)))
        ROS_ERROR_STREAM("Failed to init " << ns);
    }
  };
  pid_init("pid_roll", pid_roll_);
  pid_init("pid_pitch", pid_pitch_);
  pid_init("pid_legs", pid_legs_);

  try
  {
    left_hip_joint_handle_ =
        std::make_shared<hardware_interface::JointHandle>(effort_joint_interface_->getHandle("left_hip_joint"));
    right_hip_joint_handle_ =
        std::make_shared<hardware_interface::JointHandle>(effort_joint_interface_->getHandle("right_hip_joint"));
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Failed to get leg joint handle(s): " << e.what());
    return false;
  }

  controller_nh.getParam("phi4_offset", phi4_offset_);
  controller_nh.getParam("l1", l1_);
  controller_nh.getParam("l2", l2_);
  controller_nh.getParam("l3", l3_);
  controller_nh.getParam("l4", l4_);
  controller_nh.getParam("l5", l5_);

  active_suspension_sub_ = controller_nh.subscribe("/cmd_active_suspension", 10,
                                                   &ActiveSuspensionController::ActiveSuspensionCallBack, this);
  return true;
}

void ActiveSuspensionController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  OmniController::moveJoint(time, period);

  switch (current_state_)
  {
    case State::DOWN:
      target_leg_length_ = 0.18;
      target_pitch = 0;
      break;
    case State::MID:
      target_leg_length_ = 0.30;
      target_pitch = -0.25;
      break;
    case State::UP:
      target_leg_length_ = 0.40;
      target_pitch = -0.3;
      break;
  }

  // 0 is phi1, 1 is phi4
  double left_angle[2]{ 0. }, right_angle[2]{ 0. };
  left_angle[0] = right_angle[0] = 0;  // Todo:add real angle
  left_angle[1] = left_hip_joint_handle_->getPosition() + phi4_offset_;
  right_angle[1] = right_hip_joint_handle_->getPosition() + phi4_offset_;

  calculateLegPos(left_angle[0], left_angle[1], left_pos_);
  calculateLegPos(right_angle[0], right_angle[1], right_pos_);

  double current_leg_length = (left_pos_[0] + right_pos_[0]) / 2.0;
  double F_roll = pid_roll_->computeCommand(0 - roll_, period);
  double F_pitch = pid_pitch_->computeCommand(target_pitch - pitch_, period);
  double F_leg = pid_legs_->computeCommand(target_leg_length_ - current_leg_length, period);
  double F_spring = calculateSpringForce();
  double F_gravity = calculateFx_gravity();
  double F_left{}, F_right{}, T_left{}, T_right{};

  F_left = F_leg - F_roll - F_pitch - F_spring + F_gravity;
  F_right = F_leg + F_roll - F_pitch - F_spring + F_gravity;

  calculateTorque(F_left, left_angle[0], left_angle[1], T_left);
  calculateTorque(F_right, right_angle[0], right_angle[1], T_right);

  left_hip_joint_handle_->setCommand(T_left);
  right_hip_joint_handle_->setCommand(T_right);
}

void ActiveSuspensionController::calculateLegPos(double phi1, double phi4, double pos[2])
{
  // Four link vmc
  double YD, YB, XD, XB, lBD, A0, B0, C0, XC, YC;
  

}

void ActiveSuspensionController::calculateTorque(double F, double phi1, double phi4, double T)
{
  double J[1][2]{};
  calculateJacobian(phi1, phi4, J);

  // T=J^T * F
  T = J[0][0] * F;
}

void ActiveSuspensionController::calculateJacobian(double phi1, double phi4, double J[1][2])
{
  // Four link vmc
  double YD, YB, XD, XB, lBD, A0, B0, C0, XC, YC;
  double phi2, phi3;
  double L0, phi0;
  double j01, j11;

  YD = l4_ * sin(phi4);
  YB = l1_ * sin(phi1);
  XD = l4_ * cos(phi4);
  XB = l1_ * cos(phi1);
  lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
  A0 = 2 * l2_ * (XD - XB);
  B0 = 2 * l2_ * (YD - YB);
  C0 = l2_ * l2_ + lBD * lBD - l3_ * l3_;
  phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
  phi3 = atan2(YB - YD + l2_ * sin(phi2), XB - XD + l2_ * cos(phi2));
  XC = l1_ * cos(phi1) + l2_ * cos(phi2);
  YC = l1_ * sin(phi1) + l2_ * sin(phi2);
  L0 = sqrt(XC * XC + YC * YC);
  phi0 = atan2(YC, XC);

  j01 = (l4_ * sin(phi0 - phi2) * sin(phi3 - phi4)) / sin(phi3 - phi2);
  j11 = (l4_ * cos(phi0 - phi2) * sin(phi3 - phi4)) / (L0 * sin(phi3 - phi2));

  J[0][1] = j01;
  J[1][1] = j11;
}

double ActiveSuspensionController::calculateSpringForce()
{ return 0.; }

double ActiveSuspensionController::calculateFx_gravity()
{ return 0; }

void ActiveSuspensionController::ActiveSuspensionCallBack(const rm_msgs::ChassisActiveSusCmd& msg)
{ current_state_ = static_cast<State>(msg.mode); }
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::ActiveSuspensionController, controller_interface::ControllerBase)
