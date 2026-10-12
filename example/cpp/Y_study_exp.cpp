/**
 * @file left_right_move.cpp
 * @brief Simple left/right motion example for a Rokae robot.
 */
#ifdef _MSC_VER
#pragma execution_character_set("utf-8")
#endif

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

#include "rokae/robot.h"

using namespace rokae;
using namespace std;



namespace {
    
constexpr const char *kRobotIp = "192.168.2.160";

const char* operationStateName(OperationState state) {
    switch (state) {  // 将 SDK 枚举状态转换为中英混合文本。
    case OperationState::idle: return "idle / 空闲";
    case OperationState::jog: return "jog / 点动";
    case OperationState::rtControlling: return "rtControlling / 实时控制";
    case OperationState::drag: return "drag / 拖动模式";
    case OperationState::rlProgram: return "rlProgram / RL 程序运行";
    case OperationState::demo: return "demo / 演示程序运行";
    case OperationState::dynamicIdentify: return "dynamicIdentify / 动力学辨识";
    case OperationState::frictionIdentify: return "frictionIdentify / 摩擦辨识";
    case OperationState::loadIdentify: return "loadIdentify / 负载辨识";
    case OperationState::moving: return "moving / 运动中";
    case OperationState::unknown: return "unknown / 未知";
    default: return "unhandled / 未处理状态";
    }
}
void checkError(const error_code &ec, const char *step) {
  if(ec) {
    throw std::runtime_error(std::string(step) + ": " + ec.message());
  }
}

void prepareRobotForAutomatic(XMateRobot& robot) {
    error_code ec;  // 本函数内复用的 SDK 错误码变量。
    const auto state = robot.operationState(ec);
    checkError(ec,"读取状态");

    }
void WaitRobot(BaseRobot* robot) {
    bool running = true;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        error_code ec;
        auto st = robot->operationState(ec);
        if (st == OperationState::idle || st == OperationState::unknown) {
            running = false;
        }
    }
}
} // namespace

int main() {
    std::error_code ec;
  try {
      std::string ip = "192.168.2.160";
      double deg2rad = M_PI / 180.0;
      rokae::XMateRobot robot(ip);   
      robot.connectToRobot(ec);
      int state =(int)robot.operationState(ec);
      std::cout << "-------------" << std::endl;
      std::cout <<state<< std::endl;
      std::cout << "-------------" << std::endl;
      cout <<ec.message()<< "robotInfo(),H\n";
      auto info = robot.robotInfo(ec);
      if (ec.value() == 0){
          std::cout << "robotInfo success" << std::endl;
          std::cout << "机器人 ID: " << info.id << std::endl;
          std::cout << "机器人版本: " << info.version << std::endl;
          std::cout << "机器人型号: " << info.type << std::endl;
          std::cout << "机器人轴数: " << info.joint_num << std::endl;
      }
      else {
          cout << ec.message() << endl;
      }
      if (ec.value() == 0) {
          std::cout << ec.message() << std::endl;
      }
      else {
          std::cout << ec.message() << std::endl;
      }
      robot.moveReset(ec);
      robot.setOperateMode(OperateMode::automatic, ec);
      robot.setPowerState(true, ec);
      robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
      robot.setDefaultSpeed(100, ec);
      CartesianPosition target{
       0.400, 0.100, 0.200,
        180 * M_PI / 180,   // A，弧度
              0 * M_PI / 180,
              -90 * M_PI / 180
      };

      CartesianPosition aux{
          0.200, 0.150, 0.300,
          180 * M_PI / 180,   // A，弧度
              0 * M_PI / 180,
              -90 * M_PI / 180
      };

      MoveCCommand arc(
          target,  // 最终目标点在前
          aux,     // 圆弧辅助点在后
          100,
          0
      );
/*      MoveAbsJCommand home(
          {
              90 * deg2rad,
              0 * deg2rad,
              90 * deg2rad,
              0 * deg2rad,
              60 * deg2rad,
              0 * deg2rad
          },
          300,
          0
      );
      robot.executeCommand({ home }, ec);

     MoveJCommand command(
          {
              0.300,              // X，米
              0.300,              // Y，米
              0.300,              // Z，米
              200 * M_PI / 180,   // A，弧度
              0 * M_PI / 180,
              -90 * M_PI / 180
          },
          300,
          50
      );
      robot.executeCommand({ command }, ec);
*/
      WaitRobot(&robot);
  }
  catch (const rokae::Exception& e) {
      std::cout << e.what();
  }
  

  return 0;
}
