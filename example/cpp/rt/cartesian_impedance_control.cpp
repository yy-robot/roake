/**
 * @file cartesian_impedance_control.cpp
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "Eigen/Geometry"

using namespace rokae;

int main() {
  using namespace std;
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::XMateErProRobot robot(ip, "192.168.0.100"); // 本机地址192.168.0.100

    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();

    // 设置要接收数据。其中jointPos_m是本示例程序会用到的
    rtCon->startReceiveRobotState({RtSupportedFields::jointPos_m, RtSupportedFields::tauVel_c,
                                   RtSupportedFields::tcpPose_m, RtSupportedFields::elbow_m});

    // 读取实时状态数据: 力矩微分
    rtCon->updateRobotState();
    std::array<double,7> array7 {};
    rtCon->getStateData(RtSupportedFields::tauVel_c, array7);

    std::array<double, 16> init_position {};
    static bool init = true;
    double time = 0;

    rtCon->getStateData(RtSupportedFields::jointPos_m, array7);
    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};

    // 获取机器人当前轴角度(需要设置"jointPos_m"为要接收的状态数据)
    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.5, array7, q_drag_xm7p);

    // 设置笛卡尔空间阻抗系数，开始笛卡尔空间阻抗运动
    rtCon->setCartesianImpedance({1000, 1000, 1000, 100, 100, 100}, ec);
    rtCon->startMove(RtControllerMode::cartesianImpedance);
    std::atomic<bool> stopManually { true };

    auto callback = [&, rtCon]()->void{
      time += 0.001;
      if(init) {
        rtCon->getStateData(RtSupportedFields::tcpPose_m, init_position);
        init = false;
      }
      constexpr double kRadius = 0.2;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * time));
      double delta_z = kRadius * (std::cos(angle) - 1);

      CartesianPosition output{};
      output.pos = init_position;
      output.pos[7] += delta_z;

      if(time > 20) {
        output.setFinished(); // 20秒后结束
      }
      rtCon->appendCommand(output); // 发送运动指令
      if(output.isFinished()) {
        stopManually.store(false);  // loop为非阻塞，和主线程同步停止状态
      }
    };

    rtCon->setControlLoop(callback);
    // 非阻塞loop
    rtCon->startLoop(false);
    while(stopManually.load());
    rtCon->stopLoop();
  } catch (const std::exception &e) {
    std::cout << e.what();
  }
  return 0;
}
