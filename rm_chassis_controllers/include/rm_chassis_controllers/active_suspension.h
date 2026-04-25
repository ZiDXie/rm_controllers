//
// Created by xuncheng on 2025/11/29.
//

#include "rm_chassis_controllers/chassis_base.h"
#include "rm_chassis_controllers/omni.h"
#include <effort_controllers/joint_position_controller.h>
#include <rm_msgs/ChassisActiveSusCmd.h>
#include <std_msgs/Bool.h>
#include <rm_common/ros_utilities.h>
#include <string>
#include <Eigen/QR>
#include <pluginlib/class_list_macros.hpp>

namespace rm_chassis_controllers
{
enum class State
{
  DOWN = 0,
  MID = 1,
  UP = 2
};

class ActiveSuspensionController : public OmniController
{
public:
  ActiveSuspensionController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void ActiveSuspensionCallBack(const rm_msgs::ChassisActiveSusCmd& msg);

private:
  double calculateFx_gravity();

  /**
   * @brief Compute leg polar coordinates (length and angle) from the four-link suspension geometry.
   *
   * Given the linkage angles, this helper solves the four-bar configuration and outputs the
   * equivalent leg length and leg angle used by the controller.
   *
   * @param[in]  phi1 Upper/hip link angle (rad).
   * @param[in]  phi4 Actuated link angle (rad).
   * @param[out] pos  Output array where:
   *                 - @c pos[0] is the leg length $L$ (m)
   *                 - @c pos[1] is the leg angle $\theta$ (rad)
   *                 The buffer must provide at least 2 elements.
   *
   * @note Member link lengths @c l1_..@c l5_ must be configured before calling.
   */
  void calculateLegPos(double phi1, double phi4, double pos[2]);
  double calculateSpringForce();

  /**
   * @brief Compute joint torque from an equivalent axial/virtual force using $\tau = J^T F$.
   *
   * The Jacobian is obtained from calculateJacobian() for the current linkage angles, then the
   * commanded force is mapped into an actuator torque.
   *
   * @param[in]  F    Virtual/axial force applied along the leg (N).
   * @param[in]  phi1 Hip/upper link angle (rad).
   * @param[in]  phi4 Actuated link angle (rad).
   * @param[out] T    Resulting actuator torque (N·m).
   *
   * @note The current function signature passes @p T by value; if the intention is to return the
   *       computed torque to the caller, change it to a reference (e.g. @c double& T).
   */
  void calculateTorque(double F, double phi1, double phi4, double T);

  /**
   * @brief Compute Jacobian terms of the suspension four-link (four-bar) mechanism.
   *
   * This helper solves the internal linkage geometry and computes Jacobian elements that are
   * later used for force/velocity mapping in the virtual-model-control (VMC) formulation.
   *
   * @param[in]  phi1 Hip/upper link angle (rad).
   * @param[in]  phi4 Actuated link angle (rad).
   * @param[out] J    Output Jacobian storage.
   *                The current implementation writes to @c J[0][1] and @c J[1][1], therefore the
   *                provided buffer must be valid for indices at least up to @c [1][1] (i.e. a
   *                2x2 array or larger backing storage).
   *
   * @note Member link lengths @c l1_, @c l2_, @c l3_, @c l4_ must be configured before calling.
   */
  void calculateJacobian(double phi1, double phi4, double J[1][2]);
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;

  std::shared_ptr<hardware_interface::JointHandle> right_hip_joint_handle_;
  std::shared_ptr<hardware_interface::JointHandle> left_hip_joint_handle_;

  ros::Subscriber active_suspension_sub_;
  double l1_{ 0. }, l2_{ 0. }, l3_{ 0. }, l4_{ 0. }, l5_{ 0. };
  double phi4_offset_{ 0. };
  double left_pos_[2]{ 0. }, right_pos_[2]{ 0. };
  double target_leg_length_{ 0. };
  double target_pitch{ 0. };

  std::shared_ptr<control_toolbox::Pid> pid_roll_{}, pid_pitch_{}, pid_legs_{};

  State current_state_{ State::DOWN };
  State last_state_{ State::DOWN };
};
}  // namespace rm_chassis_controllers
