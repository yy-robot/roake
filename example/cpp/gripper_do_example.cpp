/**
 * @file gripper_do_example.cpp
 * @brief Gripper DO control example. / 夹爪数字输出控制示例。
 *
 * This file is a cleaned and renamed version of old_backup/test5.cpp.
 * It demonstrates opening and closing a simple gripper using two digital outputs.
 */

#ifdef _MSC_VER
#pragma execution_character_set("utf-8")
#endif

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "rokae/robot.h"

using namespace rokae;

namespace {

constexpr const char *kRobotIp = "192.168.2.160";
constexpr unsigned kIoBoard = 0;
constexpr unsigned kOpenPin = 0;
constexpr unsigned kClosePin = 1;
constexpr auto kActionDelay = std::chrono::seconds(1);
constexpr int kStartupRetryCount = 5;
constexpr auto kStateChangeWait = std::chrono::milliseconds(500);
constexpr auto kStartupRetryWait = std::chrono::milliseconds(1500);
constexpr int kStateWaitLoops = 20;

void setConsoleUtf8() {
#ifdef _WIN32
  SetConsoleOutputCP(CP_UTF8);  // 将 Windows 控制台输出切到 UTF-8，避免中文提示乱码。
  SetConsoleCP(CP_UTF8);        // 将 Windows 控制台输入也切到 UTF-8。
#endif
}

void checkError(const error_code &ec, const char *step) {
  if(ec) {  // SDK 返回错误码时，抛出异常并停止当前流程。
    throw std::runtime_error(std::string(step) + ": " + ec.message());
  }
}

void warnError(const error_code &ec, const char *step) {
  if(ec) {  // 可恢复步骤只打印警告，后续由五次轮询继续尝试。
    std::cerr << "[警告 Warning] " << step << ": " << ec.message() << std::endl;
  }
}

const char *operationStateName(OperationState state) {
  switch(state) {  // 将 SDK 运行状态转换为中英混合文本，方便观察控制器状态。
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

OperationState readOperationState(XMateRobot &robot, const char *step) {
  error_code ec;                               // 接收本次 SDK 状态读取的错误码。
  const auto state = robot.operationState(ec); // 查询控制器当前运行状态。
  checkError(ec, step);                        // 读取失败时停止流程。
  return state;                                // 返回当前运行状态。
}

bool waitForIdleOrUnknown(XMateRobot &robot, const char *reason) {
  for(int i = 0; i < kStateWaitLoops; ++i) {  // 最多等待约 10 秒，让控制器回到空闲。
    error_code ec;
    const auto state = robot.operationState(ec);
    if(ec) {
      std::cerr << reason << " state read warning / 读取状态警告: "
                << ec.message() << std::endl;
      return false;
    }

    if(state == OperationState::idle || state == OperationState::unknown) {
      return true;
    }

    std::cout << reason << " / 等待控制器空闲: "
              << operationStateName(state) << std::endl;
    std::this_thread::sleep_for(kStateChangeWait);
  }
  return false;
}

void prepareRobotForNrtAutomatic(XMateRobot &robot) {
  error_code ec;
  auto state = readOperationState(robot, "operationState before preparation / 准备前读取状态");
  std::cout << "\n[准备 Prepare] 当前运行状态 current operationState: "
            << operationStateName(state) << std::endl;

  if(state == OperationState::drag) {  // 如果当前在拖动模式，先退出拖动。
    std::cout << "[准备 Prepare] 退出拖动模式 disableDrag()" << std::endl;
    robot.disableDrag(ec);
    warnError(ec, "disableDrag / 退出拖动模式");
    ec.clear();
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  std::cout << "[准备 Prepare] 停止残留 Jog/运动 stop()" << std::endl;
  robot.stop(ec);                       // 停止可能残留的 Jog 或运动。
  warnError(ec, "stop / 停止运动");
  ec.clear();
  std::this_thread::sleep_for(kStateChangeWait);

  if(!waitForIdleOrUnknown(robot, "waiting for controller idle")) {
    state = readOperationState(robot, "operationState after stop / 停止后读取状态");
    throw std::runtime_error(std::string("Controller is still busy / 控制器仍忙: ") +
                             operationStateName(state));
  }

  std::cout << "[准备 Prepare] 请求伺服下电 setPowerState(false)" << std::endl;
  robot.setPowerState(false, ec);        // 切模式前先请求下电，失败也允许后面重试。
  warnError(ec, "setPowerState(false) / 请求下电");
  ec.clear();

  std::cout << "[准备 Prepare] 清除伺服报警 clearServoAlarm()" << std::endl;
  robot.clearServoAlarm(ec);             // 清除可由软件恢复的伺服报警。
  warnError(ec, "clearServoAlarm / 清除伺服报警");
  ec.clear();

  std::cout << "[准备 Prepare] 切换非实时命令模式 setMotionControlMode(NrtCommand)" << std::endl;
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);  // 切换到普通命令运动模式。
  checkError(ec, "setMotionControlMode(NrtCommand) / 切换非实时命令模式");

  std::cout << "[准备 Prepare] 清空运动队列 moveReset()" << std::endl;
  robot.moveReset(ec);                   // 清空可能残留的运动队列。
  warnError(ec, "moveReset before automatic / 自动模式前清空队列");
}

bool prepareModeAndPowerOnOnce(XMateRobot &robot, int attempt) {
  error_code ec;
  std::cout << "\n[启动 Startup] 第 " << attempt << "/" << kStartupRetryCount
            << " 次准备控制器 / startup preparation attempt "
            << attempt << "/" << kStartupRetryCount << std::endl;

  try {
    prepareRobotForNrtAutomatic(robot);  // 先整理为 NRT、空闲、可上电状态。
  } catch(const std::exception &e) {
    std::cerr << "[启动 Startup] 准备控制器失败 / preparation failed: "
              << e.what() << std::endl;
    return false;
  }

  std::cout << "[模式 Mode] 切换自动模式 setOperateMode(automatic)" << std::endl;
  robot.setOperateMode(OperateMode::automatic, ec);
  if(ec) {
    std::cerr << "[启动 Startup] setOperateMode(automatic) 暂未成功 / not ready yet: "
              << ec.message() << std::endl;
    return false;
  }

  std::this_thread::sleep_for(kStateChangeWait);
  std::cout << "[上电 Power] 伺服上电 setPowerState(true)" << std::endl;
  robot.setPowerState(true, ec);
  if(ec) {
    std::cerr << "[启动 Startup] setPowerState(true) 暂未成功 / not ready yet: "
              << ec.message() << std::endl;
    return false;
  }

  std::this_thread::sleep_for(kStateChangeWait);
  std::cout << "[队列 Queue] 再次清空运动队列 moveReset()" << std::endl;
  robot.moveReset(ec);
  if(ec) {
    std::cerr << "[启动 Startup] moveReset 暂未成功 / not ready yet: "
              << ec.message() << std::endl;
    return false;
  }

  return true;
}

bool prepareModeAndPowerOnWithRetry(XMateRobot &robot) {
  for(int attempt = 1; attempt <= kStartupRetryCount; ++attempt) {  // 最多轮询/重试 5 次。
    if(prepareModeAndPowerOnOnce(robot, attempt)) {
      std::cout << "[启动 Startup] 控制器已可用 / Controller is ready." << std::endl;
      return true;
    }

    if(attempt < kStartupRetryCount) {
      std::cout << "[启动 Startup] 等待状态稳定后重试 / Wait for state to settle, then retry."
                << std::endl;
      std::this_thread::sleep_for(kStartupRetryWait);
    }
  }
  return false;
}

void printPowerFailureHint(const char *where) {
  std::cerr << "\n[失败 Failed] " << where << std::endl;
  std::cerr << "程序不能绕过控制器安全限制。/ The program cannot bypass controller safety interlocks." << std::endl;
  std::cerr << "请先确认: / Please check first:" << std::endl;
  std::cerr << "1. 释放实体/外部急停。/ Release physical or external emergency stop." << std::endl;
  std::cerr << "2. 清除报警。/ Clear alarms." << std::endl;
  std::cerr << "3. 退出拖动、力矩控制或实时控制状态。/ Exit drag, torque-control, or real-time-control state." << std::endl;
  std::cerr << "4. 切换到允许上电的控制模式。/ Switch to a mode that permits servo power." << std::endl;
}

class GripperController {
 public:
  GripperController(BaseRobot *robot, unsigned board, unsigned openPin, unsigned closePin)
      : robot_(robot), board_(board), openPin_(openPin), closePin_(closePin) {}

  void open(error_code &ec) {
    std::cout << "[Gripper] Open gripper / 打开夹爪" << std::endl;
    robot_->setDO(board_, openPin_, true, ec);
    checkError(ec, "setDO open pin true");
    std::cout << "setDO open pin true" << std::endl;
    robot_->setDO(board_, closePin_, false, ec);
    checkError(ec, "setDO close pin false");
    std::this_thread::sleep_for(kActionDelay);
    opened_ = true;
  }

  void close(error_code &ec) {
    std::cout << "[Gripper] Close gripper / 关闭夹爪" << std::endl;
    robot_->setDO(board_, openPin_, false, ec);
    checkError(ec, "setDO open pin false");
    robot_->setDO(board_, closePin_, true, ec);
    checkError(ec, "setDO close pin true");
    std::this_thread::sleep_for(kActionDelay);
    opened_ = false;
  }

  void open_pin_ture(error_code& ec) {
      robot_->setDO(board_, openPin_, true, ec);//打开半行程
      checkError(ec, "setDO open pin true");
      std::cout << "setDO open pin true" << std::endl;
  }

  void close_pin_ture(error_code& ec) {
      robot_->setDO(board_, closePin_, true, ec);//关闭半行程
      checkError(ec, "setDO close pin true");
      std::cout << "setDO close pin true" << std::endl;
  }
  void open_pin_false(error_code& ec) {
      robot_->setDO(board_, openPin_, false, ec);//打开全行程
      checkError(ec, "setDO open pin false");
      std::cout << "setDO open pin false" << std::endl;
  }

  void close_pin_false(error_code& ec) {
      robot_->setDO(board_, closePin_, false , ec);//关闭全行程
      checkError(ec, "setDO close pin false");
      std::cout << "setDO close pin false" << std::endl;
  }

  void toggle(error_code &ec) {
    if(opened_) {
      close(ec);
    } else {
      open(ec);
    }
  }

 private:
  BaseRobot *robot_;
  unsigned board_;
  unsigned openPin_;
  unsigned closePin_;
  bool opened_ {false};
};

} // namespace

int main() {
  try {
    setConsoleUtf8();
    error_code ec;
    XMateRobot robot(kRobotIp);

    std::cout << "========== Gripper DO Example / 夹爪 DO 控制示例 ==========" << std::endl;
    std::cout << "Robot IP / 机器人 IP: " << kRobotIp << std::endl;
    std::cout << "Open pin / 打开端口: DO" << kOpenPin
              << ", close pin / 关闭端口: DO" << kClosePin << std::endl;

    std::cout << "[连接 Connect] 连接控制器 connectToRobot(): " << kRobotIp << std::endl;
    robot.connectToRobot(ec);
    checkError(ec, "connectToRobot / 连接控制器");

    if(!prepareModeAndPowerOnWithRetry(robot)) {
      printPowerFailureHint("startup retry / 启动重试");
      throw std::runtime_error("startup preparation failed / 启动准备失败");
    }

    GripperController gripper(&robot, kIoBoard, kOpenPin, kClosePin);
    
    gripper.open_pin_ture(ec);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    gripper.open_pin_false(ec);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    gripper.close_pin_ture(ec);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    gripper.close_pin_false(ec);
    std::this_thread::sleep_for(std::chrono::seconds(2));



/*
    gripper.open(ec);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    gripper.close(ec);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    gripper.toggle(ec);
*/
    std::cout << "[Done] Gripper DO example finished / 夹爪 DO 示例结束" << std::endl;

    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);
    return 0;
  } catch(const std::exception &e) {
    std::cerr << "[Error / 错误] " << e.what() << std::endl;
    return 1;
  }
}
