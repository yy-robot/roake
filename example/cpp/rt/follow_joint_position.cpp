/**
 * @file follow_joint_position.cpp
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <thread>
#include "rokae/robot.h"
#include "../print_helper.hpp"
#include "rokae/utility.h"

std::atomic_bool running = true;
std::ostream &os = std::cout;
std::array<double, 7> q_drag_xm7p = {0,M_PI/6,0,M_PI/3,0,M_PI/2,0};
std::array<double, 6> q_drag_cr7 = { 0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0 };

template <unsigned short DoF>
void updatePose(rokae::FollowPosition<DoF> &fp){
  using namespace std::chrono;
  std::this_thread::sleep_for(std::chrono::seconds(5));
  long count = 1;
  std::array<double, DoF> desired = q_drag_cr7;
  double step = rokae::Utils::degToRad(0.3);
  fp.setScale(2);
  while(running) {
    // 模拟每50ms更新一次位置
    std::this_thread::sleep_for(std::chrono::milliseconds (50));
    if(count++ % 300 == 0) {
      step = -step;
    }
    for(auto &d: desired) {
      d += step;
    }
    fp.update(desired);
  }
}

int main() {
  using namespace rokae;
  using namespace std;
  using namespace rokae::RtSupportedFields;

  string remoteIP = "192.168.0.160";
  string localIP = "192.168.0.180";
  error_code ec;
  std::thread updater;

  try {
    XMateRobot robot(remoteIP, localIP);
    auto model = robot.model();
    robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);

    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setPowerState(true, ec);
    auto rtCon = robot.getRtMotionController().lock();
    rtCon->startReceiveRobotState({jointPos_m});

    array<double, 6> init {};
    rtCon->updateRobotState();
    rtCon->getStateData(jointPos_m, init);
    rtCon->MoveJ(0.4, init, q_drag_cr7);
    this_thread::sleep_for(chrono::seconds(2));

    FollowPosition follow_pose(robot, model);

    print(os, "开始跟随");
    Eigen::Transform<double, 3, Eigen::Isometry> bMe_desire = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
    bMe_desire.rotate(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0));
    bMe_desire.pretranslate(Eigen::Vector3d(0.563, 0, 0.2));
    follow_pose.start(bMe_desire);
    updater = std::thread([&]() {
      updatePose(follow_pose);
    });

    // press 'q' to stop
    while (getchar() != 'q');
    running = false;

    follow_pose.stop();
    updater.join();
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
    if(updater.joinable()) {
      running = false;
      updater.join();
    }
  }
  return 0;
}

