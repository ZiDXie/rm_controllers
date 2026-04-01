//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/stand_up.h"
#include "bipedal_wheel_controller/controller.h"
#include "bipedal_wheel_controller/helper_functions.h"

namespace rm_chassis_controllers
{
StandUp::StandUp(const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas)
  : joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas)
{
}

void StandUp::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter STAND_UP");
    controller->setStateChange(true);
    controller->setCompleteStand(false);
    leg_state_threshold_ = controller->getLegThresholdParams();
    vmcPtr_ = controller->getVMCPtr();
    StandUp::detectLegState(x_left_, left_leg_state);
    StandUp::detectLegState(x_right_, right_leg_state);
  }

  auto model_params_ = controller->getModelParams();
  spring_force_ = -model_params_->f_spring;
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  setUpLegMotion(x_left_, right_leg_state, left_pos_[0], left_pos_[1], left_leg_state, left_leg_command_, left_stop_);
  setUpLegMotion(x_right_, left_leg_state, right_pos_[0], right_pos_[1], right_leg_state, right_leg_command_,
                 right_stop_);

  if (!left_stop_)
  {
    left_cmd = computePidLegCommand(left_leg_command_, left_pos_, left_spd_, *pid_legs_[0], *pid_thetas_[0],
                                    *pid_thetas_[2], left_angle_, left_leg_state, period, spring_force_);
  }
  if (!right_stop_)
  {
    right_cmd = computePidLegCommand(right_leg_command_, right_pos_, right_spd_, *pid_legs_[1], *pid_thetas_[1],
                                     *pid_thetas_[3], right_angle_, right_leg_state, period, spring_force_);
  }

  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  //  if (((left_pos_[1] < 0.3 && left_leg_state == LegState::BEHIND) ||
  //       (left_pos_[1] > -0.3 && left_leg_state == LegState::UNDER)) &&
  //      ((right_pos_[1] < 0.3 && right_leg_state == LegState::BEHIND) ||
  //       (right_pos_[1] > -0.3 && right_leg_state == LegState::UNDER)))
  if (((left_pos_[1] < 0.2 && left_leg_state == LegState::BEHIND)) &&
      ((right_pos_[1] < 0.2 && right_leg_state == LegState::BEHIND)))
  {
    controller->setMode(BalanceMode::NORMAL);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit STAND_UP");
  }
  if (controller->getOverturn())
  {
    controller->setMode(BalanceMode::RECOVER);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit STAND_UP");
  }
}

void StandUp::setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const int& other_leg_state,
                             const double& leg_length, const double& leg_theta, int& leg_state,
                             StandUpLegCommand& legCommand, bool& stop_flag)
{
  switch (leg_state)
  {
    case LegState::UNDER:
      stop_flag = false;
      legCommand.desired_angle = -M_PI_2;
      legCommand.desired_length = 0.36;
      if (leg_length > 0.33)
      {
        leg_state = LegState::FRONT;
      }
      break;
    case LegState::FRONT:
      stop_flag = false;
      legCommand.desired_angle = M_PI_2 - 0.35;
      legCommand.desired_length = 0.36;
      legCommand.desired_angle_vel = -5.0;
      if (abs(legCommand.desired_angle - leg_theta) < 0.4)
      {
        legCommand.desired_angle_vel = -1.0;
      }
      if (abs(x[1]) < 0.2)
      {
        if (x[0] > 0 && x[0] < M_PI_2)
        {
          leg_state = LegState::BEHIND;
        }
      }
      break;
    case LegState::BEHIND:
      stop_flag = true;
      legCommand.desired_angle = leg_theta;
      legCommand.desired_length = leg_length;
      if (other_leg_state == LegState::BEHIND)
      {
        stop_flag = false;
        double h = 0.125;
        //        legCommand.desired_length = 0.12;
        //        legCommand.desired_angle = acos(h / leg_length);
        legCommand.desired_length = h / cos(leg_theta);
        legCommand.desired_angle = 0.0;
        //        legCommand.desired_length = 0.18;
        //        if (leg_length < 0.3)
        //        {
        //          if (leg_theta < M_PI_2)
        //            legCommand.desired_angle = 0.3;
        //        }
        //        if (leg_length < 0.15)
        //          legCommand.desired_angle = 0.0;
      }
      break;
  }
}

inline void StandUp::detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, int& leg_state)
{
  if (!leg_state_threshold_)
  {
    ROS_ERROR_THROTTLE(1.0, "LegUtils threshold params not initialized!");
    return;
  }
  if (x[0] > leg_state_threshold_->under_lower && x[0] < leg_state_threshold_->under_upper)
    leg_state = LegState::UNDER;
  else if ((x[0] < leg_state_threshold_->front_lower && x[0] > -M_PI) ||
           (x[0] < M_PI && x[0] > leg_state_threshold_->front_upper))
    leg_state = LegState::FRONT;
  else if (x[0] > leg_state_threshold_->behind_lower && x[0] < leg_state_threshold_->behind_upper)
    leg_state = LegState::BEHIND;
  switch (leg_state)
  {
    case LegState::UNDER:
      ROS_INFO("[balance] x[0]: %.3f Leg state: UNDER", x[0]);
      break;
    case LegState::FRONT:
      ROS_INFO("[balance] x[0]: %.3f Leg state: FRONT", x[0]);
      break;
    case LegState::BEHIND:
      ROS_INFO("[balance] x[0]: %.3f Leg state: BEHIND", x[0]);
      break;
  }
}

inline LegCommand StandUp::computePidLegCommand(const StandUpLegCommand& leg_command, double leg_pos[2],
                                                double leg_spd[2], control_toolbox::Pid& length_pid,
                                                control_toolbox::Pid& angle_pid, control_toolbox::Pid& angle_vel_pid,
                                                const double* leg_angle, const int& leg_state,
                                                const ros::Duration& period, double feedforward_force)
{
  LegCommand cmd{ 0.0, 0.0, { 0.0, 0.0 } };
  cmd.force = length_pid.computeCommand(leg_command.desired_length - leg_pos[0], period) + feedforward_force;
  cmd.force = abs(cmd.force) > 250 ? std::copysign(1, cmd.force) * 250 : cmd.force;
  if (leg_state == LegState::BEHIND || leg_state == LegState::UNDER)
  {
    cmd.torque =
        angle_pid.computeCommand(-angles::shortest_angular_distance(leg_command.desired_angle, leg_pos[1]), period);
  }
  else
  {
    cmd.torque = angle_vel_pid.computeCommand(leg_command.desired_angle_vel - leg_spd[1], period);
  }
  vmcPtr_->leg_conv(cmd.force, cmd.torque, leg_angle[0], leg_angle[1], cmd.input);
  return cmd;
}
}  // namespace rm_chassis_controllers
