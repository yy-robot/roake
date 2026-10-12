/**
 * @file move_commands.cpp
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <cmath>
#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "../print_helper.hpp"

using namespace rokae;

int main() {
  using namespace std;
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::XMateErProRobot robot(ip, "192.168.0.180"); // ****   XMate 7-axis
    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    // 若程序运行时控制器已经是实时模式，需要先切换到非实时模式后再更改网络延迟阈值，否则不生效
    robot.setRtNetworkTolerance(20, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();
    rtCon->startReceiveRobotState({RtSupportedFields::jointPos_m, RtSupportedFields::elbow_m, RtSupportedFields::tcpPose_m});

    // 示例程序使用机型: xMateER7 Pro
    // 1. 从当前位置MoveJ运动到拖拽位置
    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0}, q_init;
    rtCon->getStateData(RtSupportedFields::jointPos_m, q_init);
    rtCon->MoveJ(0.5, q_init, q_drag_xm7p);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    // 2. 圆弧运动 (X-Y平面上)
    CartesianPosition start, aux, target;
    rtCon->updateRobotState();
    rtCon->getStateData(RtSupportedFields::tcpPose_m, start.pos);
    Eigen::Matrix3d rot_start;
    Eigen::Vector3d trans_start, trans_aux, trans_end;
    Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
    trans_end = trans_start; trans_aux = trans_start;
    trans_aux[0] -= 0.3;
    trans_aux[1] -= 0.05;
    trans_end[1] -= 0.15;

    Utils::transMatrixToArray(rot_start, trans_aux, aux.pos);
    Utils::transMatrixToArray(rot_start, trans_end, target.pos);
    rtCon->MoveC(0.2, start, aux, target);

    // 3. 直线运动
    rtCon->updateRobotState();
    rtCon->getStateData(RtSupportedFields::tcpPose_m, start.pos); // 读取机器人当前位姿作为起始位姿
    Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
    start.pos = target.pos;
    trans_end = trans_start;
    // 沿 x-0.25m, z-0.3
    trans_end[0] -= 0.25;
    trans_end[2] -= 0.3;
    Utils::transMatrixToArray(rot_start, trans_end, target.pos);

    rtCon->MoveL(0.2, start, target);

    // 4. 关闭实时模式
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);

  } catch (const std::exception &e) {
    std::cout << "Final catch exception\n";
    std::cout << e.what();
  }
  return 0;
}