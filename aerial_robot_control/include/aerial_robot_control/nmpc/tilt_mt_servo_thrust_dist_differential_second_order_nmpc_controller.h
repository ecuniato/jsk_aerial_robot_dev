#ifndef TILT_MT_SERVO_THRUST_DIST_DIFFERENTIAL_SECOND_ORDER_NMPC_CONTROLLER_H
#define TILT_MT_SERVO_THRUST_DIST_DIFFERENTIAL_SECOND_ORDER_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_dist_nmpc_controller.h"

#include "spinal/ESCTelemetryArray.h"

namespace aerial_robot_control
{

namespace nmpc
{

class TiltMtServoThrustDistDifferentialSecondOrderNMPC : public nmpc::TiltMtServoDistNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  double krpm_square_to_thrust_ratio_;
  double krpm_square_to_thrust_bias_;
  std::vector<double> thrust_meas_;
  ros::Subscriber sub_esc_telem_;

  Eigen::VectorXd internal_wrench_b_;

  std::vector<double> prev_joint_angle_;
  std::vector<double> prev_joint_angle_vel_estimate_;
  std::vector<double> prev_thrust_meas_;
  std::vector<double> prev_thrust_vel_estimate_;

  double servo_angle_velocity_min_, servo_angle_velocity_max_;
  double thrust_velocity_min_, thrust_velocity_max_;
  double servo_angle_c_velocity_min_, servo_angle_c_velocity_max_;
  double thrust_c_velocity_min_, thrust_c_velocity_max_;

  std::vector<double> uo_prev_;

  inline void initActuatorStates() override
  {
    nmpc::TiltMtServoNMPC::initActuatorStates();
    thrust_meas_.resize(motor_num_, 0.0);

    internal_wrench_b_ = Eigen::VectorXd::Zero(6);

    prev_joint_angle_.resize(joint_num_, 0.0);
    prev_joint_angle_vel_estimate_.resize(joint_num_, 0.0);
    prev_thrust_meas_.resize(motor_num_, 0.0);
    prev_thrust_vel_estimate_.resize(motor_num_, 0.0);

    uo_prev_.resize(motor_num_ + joint_num_, 0.0);
  }

  void initGeneralParams() override;

  void initNMPCCostW() override;

  void initNMPCConstraints() override;

  void callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg);

  std::vector<double> meas2VecX(bool is_modified_by_traj_frame) override;

  void allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i, const tf::Quaternion& ref_quat_ib,
                    const tf::Vector3& ref_omega_b, const VectorXd& ref_wrench_b, vector<double>& x,
                    vector<double>& u) override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;

  void computeInternalWrenchB();

  double getCommand(int idx_u, double T_horizon = 0.0) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_THRUST_DIST_DIFFERENTIAL_SECOND_ORDER_NMPC_CONTROLLER_H
