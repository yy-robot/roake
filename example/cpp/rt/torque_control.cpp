/**
 * @file torque_control.cpp
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "Eigen/Geometry"
#include "../print_helper.hpp"

using namespace rokae;

void torqueControl(XMateErProRobot &robot) {
  using namespace RtSupportedFields;
  error_code ec;
  auto rtCon = robot.getRtMotionController().lock();
  rtCon->automaticErrorRecovery(ec);
  auto model = robot.model();
  std::array<double,7> q_init {};
  std::array<double,7> q_drag = {0, M_PI/6, 0, M_PI/3, 0, M_PI_2,0 };

  rtCon->stopReceiveRobotState();
  rtCon->startReceiveRobotState({jointPos_m, jointVel_m, jointAcc_c, tcpPose_m});

  // 运动到拖拽位置
  rtCon->getStateData("q_m", q_init);
  rtCon->MoveJ(0.2, q_init, q_drag);

  // 控制模式为力矩控制
  rtCon->startMove(RtControllerMode::torque);

  // Compliance parameters
  const double translational_stiffness{200.0};
  const double rotational_stiffness{5.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 0.0 * sqrt(translational_stiffness) *
    Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 0.0 * sqrt(rotational_stiffness) *
    Eigen::MatrixXd::Identity(3, 3);

  std::array<double, 16> init_position {};
  static bool init = true;
  Eigen::Matrix<double, 6, 7> jacobian;

  auto callback = [&]() {
    using namespace RtSupportedFields;
    static double time=0;
    time += 0.001;
    if(init){
      rtCon->getStateData(tcpPose_m, init_position);
      init = false;
    }
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(init_position.data()).transpose());
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    std::array<double, 7> q{}, dq_m{}, ddq_c{};
    std::array<double, 16> pos_m {};
    rtCon->getStateData(jointPos_m, q);
    rtCon->getStateData(jointVel_m, dq_m);
    rtCon->getStateData(jointAcc_c, ddq_c);

    std::array<double, 42> jacobian_array = model.jacobian(q);
    std::array<double, 7> gravity_array = model.getTorque(q, dq_m, ddq_c, TorqueType::gravity);
    std::array<double, 7> friction_array = model.getTorque(q, dq_m, ddq_c, TorqueType::friction);

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> friction(friction_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 6>> jacobian_(jacobian_array.data());
    jacobian = jacobian_.transpose();
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_mat(q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_mat(dq_m.data());
    rtCon->getStateData(tcpPose_m, pos_m);
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(pos_m.data()).transpose());
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired equilibrium pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d;

    // orientation error
    // "difference" quaternion
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    // compute control
    Eigen::VectorXd tau_d(7);

    // cartesion space impedance calculate && map to joint space
    tau_d << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq_mat));

    constexpr double kRadius = 3;
    double angle = M_PI / 4 * (1 - std::cos(M_PI / 4 * time));

    Torque cmd {};
    cmd.tau.resize(7);
    Eigen::VectorXd::Map(cmd.tau.data(), 7) = tau_d;

    if(time>200){
      cmd.setFinished();
    }
    rtCon->appendCommand(cmd);
  };

  rtCon->setControlLoop(callback);
  rtCon->startLoop();
}

int main() {
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::XMateErProRobot robot(ip, "192.168.0.180"); // ****   XMate 7-axis
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    // 开始力矩模式控制
    torqueControl(robot);

    // 关闭实时模式
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);

  } catch (const std::exception &e) {
    std::cout << e.what();
  }
  return 0;
}
