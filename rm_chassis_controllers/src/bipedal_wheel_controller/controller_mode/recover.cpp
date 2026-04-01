//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/recover.h"
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
Recover::Recover(const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas, control_toolbox::Pid* pid_theta_diff)
  : joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas), pid_theta_diff_(pid_theta_diff)
{
}

void Recover::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter RECOVER");
    detectd_flag = false;
    controller->setStateChange(true);
  }

  // until chassis
  if (!detectd_flag && abs(x_left_[1]) < 0.1 && abs(x_left_[5]) < 0.1)
  {
    detectChassisStateToRecover();
    detectd_flag = true;
    leg_recovery_velocity_ =
        recovery_chassis_state_ == BackwardSlip ? -leg_recovery_velocity_const_ : leg_recovery_velocity_const_;
  }

  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  leg_theta_diff_ = angles::shortest_angular_distance(left_pos_[1], right_pos_[1]);
  double T_theta_diff{ 0.0 }, feedforward_force{ 0.0 };
  if (controller->getBaseState() != 4)
  {
    left_cmd.force = pid_legs_[0]->computeCommand(desired_leg_length_ - left_pos_[0], period) + feedforward_force;
    right_cmd.force = pid_legs_[1]->computeCommand(desired_leg_length_ - right_pos_[0], period) + feedforward_force;
    if (roll_ < -0.5)
    {
      left_cmd.torque = pid_thetas_[2]->computeCommand(leg_recovery_velocity_ - left_spd_[1], period);
      right_cmd.torque = pid_thetas_[3]->computeCommand(0 - right_spd_[1], period);
      controller->getVMCPtr()->leg_conv(left_cmd.force, 5 * leg_recovery_velocity_ + left_cmd.torque, left_angle_[0],
                                        left_angle_[1], left_cmd.input);
      controller->getVMCPtr()->leg_conv(right_cmd.force, 5 * leg_recovery_velocity_ + right_cmd.torque, right_angle_[0],
                                        right_angle_[1], right_cmd.input);
    }
    else if (roll_ > 0.5)
    {
      left_cmd.torque = pid_thetas_[2]->computeCommand(0 - left_spd_[1], period);
      right_cmd.torque = pid_thetas_[3]->computeCommand(leg_recovery_velocity_ - right_spd_[1], period);
      controller->getVMCPtr()->leg_conv(left_cmd.force, 5 * leg_recovery_velocity_ + left_cmd.torque, left_angle_[0],
                                        left_angle_[1], left_cmd.input);
      controller->getVMCPtr()->leg_conv(right_cmd.force, 5 * leg_recovery_velocity_ + right_cmd.torque, right_angle_[0],
                                        right_angle_[1], right_cmd.input);
    }
    else
    {
      if (left_recovery_leg == NotReady && right_recovery_leg == Ready)
      {
        detectLegRecoveryState(left_recovery_leg, left_pos_[1]);
        detectLegRecoveryState(right_recovery_leg, right_pos_[1]);
        left_cmd.torque = pid_thetas_[2]->computeCommand(leg_recovery_velocity_ - left_spd_[1], period);
        right_cmd.torque = pid_thetas_[3]->computeCommand(0 - right_spd_[1], period);
        controller->getVMCPtr()->leg_conv(left_cmd.force, 5 * leg_recovery_velocity_ + left_cmd.torque, left_angle_[0],
                                          left_angle_[1], left_cmd.input);
        controller->getVMCPtr()->leg_conv(right_cmd.force, 5 * leg_recovery_velocity_ + right_cmd.torque,
                                          right_angle_[0], right_angle_[1], right_cmd.input);
      }
      if (left_recovery_leg == Ready && right_recovery_leg == NotReady)
      {
        detectLegRecoveryState(left_recovery_leg, left_pos_[1]);
        detectLegRecoveryState(right_recovery_leg, right_pos_[1]);
        left_cmd.torque = pid_thetas_[2]->computeCommand(0 - left_spd_[1], period);
        right_cmd.torque = pid_thetas_[3]->computeCommand(leg_recovery_velocity_ - right_spd_[1], period);
        controller->getVMCPtr()->leg_conv(left_cmd.force, 5 * leg_recovery_velocity_ + left_cmd.torque, left_angle_[0],
                                          left_angle_[1], left_cmd.input);
        controller->getVMCPtr()->leg_conv(right_cmd.force, 5 * leg_recovery_velocity_ + right_cmd.torque,
                                          right_angle_[0], right_angle_[1], right_cmd.input);
      }
      if (abs(leg_theta_diff_) < 0.4)
      {
        if ((left_recovery_leg == Ready && right_recovery_leg == Ready) ||
            (left_recovery_leg == NotReady && right_recovery_leg == NotReady))
        {
          T_theta_diff = pid_theta_diff_->computeCommand(leg_theta_diff_, period);
          detectLegRecoveryState(left_recovery_leg, left_pos_[1]);
          detectLegRecoveryState(right_recovery_leg, right_pos_[1]);
          left_cmd.torque = pid_thetas_[2]->computeCommand(leg_recovery_velocity_ - left_spd_[1], period);
          right_cmd.torque = pid_thetas_[3]->computeCommand(leg_recovery_velocity_ - right_spd_[1], period);
          controller->getVMCPtr()->leg_conv(left_cmd.force, 5 * leg_recovery_velocity_ + left_cmd.torque + T_theta_diff,
                                            left_angle_[0], left_angle_[1], left_cmd.input);
          controller->getVMCPtr()->leg_conv(right_cmd.force,
                                            5 * leg_recovery_velocity_ + right_cmd.torque - T_theta_diff,
                                            right_angle_[0], right_angle_[1], right_cmd.input);
        }
      }
    }
  }
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if (abs(pitch_) < 0.2 && linear_acc_base_.z > 5.0 && !controller->getOverturn())
  {
    controller->setMode(BalanceMode::SIT_DOWN);
    controller->setStateChange(false);
    controller->clearRecoveryFlag();
    ROS_INFO("[balance] Exit RECOVER");
  }
}

void Recover::detectChassisStateToRecover()
{
  // pitch_ is base_link pitch not model pitch
  if (pitch_ > 0.45 && pitch_ < M_PI)
  {
    ROS_INFO("forward");
    recovery_chassis_state_ = RecoveryChassisState::ForwardSlip;
  }
  else if (pitch_ < -0.45 && pitch_ > -M_PI)
  {
    ROS_INFO("back");
    recovery_chassis_state_ = RecoveryChassisState::BackwardSlip;
  }
  detectLegRecoveryState(left_recovery_leg, left_pos_[1]);
  detectLegRecoveryState(right_recovery_leg, right_pos_[1]);
}

inline void Recover::detectLegRecoveryState(LegRecoveryState& leg_recovery_state, const double& leg_pos)
{
  if (recovery_chassis_state_ == RecoveryChassisState::ForwardSlip)
  {
    if ((leg_pos < M_PI && leg_pos > M_PI - 0.3) || (leg_pos < (-M_PI_2 + 0.4) && leg_pos > -M_PI))
    {
      leg_recovery_state = Ready;
    }
    else
    {
      leg_recovery_state = NotReady;
    }
  }
  else
  {
    if ((leg_pos > 0.5 && leg_pos < M_PI) || (leg_pos < (-M_PI_2 + 1.0) && leg_pos > -M_PI))
    {
      leg_recovery_state = Ready;
    }
    else
    {
      leg_recovery_state = NotReady;
    }
  }
  ROS_INFO("%d", leg_recovery_state);
}
}  // namespace rm_chassis_controllers
