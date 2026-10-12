/**
 * @file study_example.cpp
 * @brief xMate ER3 study motion example. / xMate ER3 学习用运动示例。
 *
 * 程序流程 / Program flow:
 * 1. 自动整理控制器状态，让机器人进入可执行非实时命令的状态。
 * 2. 移动到 HOME 初始姿态。
 * 3. J1 到 J6 依次演示：单关节旋转 60 度，再回到 HOME。
 * 4. 从 HOME 的法兰位姿开始，执行一个小范围螺旋下落。
 * 5. 最后回到 HOME 初始姿态。
 *
 * 安全说明 / Safety notes:
 * - 代码不能绕过急停、外部安全输入、伺服硬报警或控制器安全限制。
 * - 每个关节演示会检查软限位。若 +60 度超限，会尝试 -60 度。
 * - 螺旋轨迹半径和下降高度设置得很小，仍需确认现场空间安全。
 */

#ifdef _MSC_VER
#pragma execution_character_set("utf-8")  // 告诉 MSVC 当前源码字符串按 UTF-8 处理。
#endif

#include <algorithm>  // std::max/std::min 用于限幅速度和计算最大差角。
#include <array>      // std::array 用于保存 6 个关节角和 6 维笛卡尔位姿。
#include <chrono>     // std::chrono 用于毫秒、秒等时间单位。
#include <cmath>      // std::sin、std::cos 用于生成螺旋轨迹。
#include <iomanip>    // std::setprecision 用于控制控制台小数位。
#include <iostream>   // std::cout/std::cerr/std::cin 用于控制台交互。
#include <stdexcept>  // std::runtime_error 用于把错误码变成异常。
#include <string>     // std::string 用于拼接提示文字。
#include <system_error> // std::errc 用于构造通用错误码。
#include <thread>     // std::this_thread::sleep_for 用于等待控制器状态稳定。
#include <vector>     // std::vector 用于构造 JointPosition。

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX      // 禁止 windows.h 定义 min/max 宏，避免和 std::min/std::max 冲突。
#endif
#include <windows.h>  // Windows 下设置控制台 UTF-8 编码。
#endif

#include "rokae/robot.h"  // 珞石 SDK 主头文件，提供 XMateRobot 和运动命令。

using namespace rokae;  // 直接使用 rokae 命名空间中的 SDK 类型。

namespace {

constexpr const char *kRobotIp = "192.168.2.160";  // 机器人控制器 IP 地址。
constexpr double kPi = 3.14159265358979323846;     // 圆周率，用于角度/弧度转换。
constexpr int kJointSpeed = 800;                    // 关节演示速度，低速便于观察。
constexpr int kCartesianSpeed = 200;                 // 螺旋 MoveL 速度，单位按 SDK 定义。
constexpr int kFineZone = 5;                        // zone=0 表示尽量精确停到目标点。
constexpr auto kPollPeriod = std::chrono::milliseconds(1000);       // 运动中每 1s 打印状态。
constexpr auto kStateChangeWait = std::chrono::milliseconds(500);  // 状态切换后等待 0.5 秒。
constexpr auto kStartupRetryWait = std::chrono::milliseconds(1500); // 启动失败后等 1.5 秒再重试。
constexpr int kStateWaitLoops = 20;                                // 等待空闲最多 20 次。
constexpr int kStartupRetryCount = 5;                               // 启动准备最多重试 5 次。
constexpr double kJointDemoDeg = 60.0;                              // 每个关节演示旋转 60 度。
constexpr int kHomeReturnMinSpeed = 100;                            // 回 HOME 固定慢速档，SDK 中约对应 30% 关节速度档。
constexpr int kHomeReturnMaxSpeed = 800;                            // 回 HOME 最高速度档，SDK 中约对应 80% 关节速度档。
constexpr double kHomeReturnSpeedGain = 12.0;                       // 回 HOME 速度增益：最大差角 1 deg 约对应 12 个 speed 单位。
constexpr double kHomeReturnSlowZoneDeg = 2.0;                      // 差角小于 2 deg 后才使用固定慢速直达 HOME。
constexpr double kHomeReturnSegmentRatio = 0.80;                    // 每段走剩余距离的 80%，快速接近 HOME。
constexpr double kHomeReturnArrivedDeg = 0.5;                       // 距离 HOME 小于 0.5 deg 时认为已到位。
constexpr int kHomeReturnMaxSegments = 3;                           // 变速回 HOME 最多分 5 段，防止分段过多导致拖慢。
constexpr double kSoftLimitMarginDeg = 3.0;                         // 距离软限位至少保留 3 度余量。
constexpr double kSpiralRadiusM = 0.025;                            // 螺旋半径 25mm。
constexpr double kSpiralDropM = 0.060;                              // 螺旋总下降 60mm。
constexpr int kSpiralPoints = 12;                                   // 螺旋分成 12 个小直线段。
constexpr double kSpiralTurns = 1.0;                                // 螺旋旋转 1 圈。

// HOME 初始姿态，单位是角度。执行前会转换成 SDK 使用的弧度。
constexpr std::array<double, 6> kHomeDeg = {140.0, 0.0, 90.0, 0.0, 90.0, 0.0};

// 根据 Robot Assist 软限位截图整理的 xMate ER3 关节软限位，单位是角度。
constexpr std::array<double, 6> kSoftMinDeg = {-165.0, -115.0, -115.0, -165.0, -115.0, -355.0};
constexpr std::array<double, 6> kSoftMaxDeg = { 165.0,  115.0,  115.0,  165.0,  115.0,  355.0};

void setConsoleUtf8() {
#ifdef _WIN32
  SetConsoleOutputCP(CP_UTF8);  // 设置 Windows 控制台输出为 UTF-8，减少中文乱码。
  SetConsoleCP(CP_UTF8);        // 设置 Windows 控制台输入为 UTF-8。
#endif
}

double degToRad(double deg) {
  return deg * kPi / 180.0;  // 角度转换为弧度，SDK 运动命令使用弧度。
}

double radToDeg(double rad) {
  return rad * 180.0 / kPi;  // 弧度转换为角度，方便控制台阅读。
}

void checkError(const error_code &ec, const char *step) {
  if(ec) {  // SDK 错误码非空时，把错误转换为异常并停止当前流程。
    throw std::runtime_error(std::string(step) + ": " + ec.message());
  }
}

void warnError(const error_code &ec, const char *step) {
  if(ec) {  // 可恢复步骤失败时只打印警告，由后续重试处理。
    std::cerr << "[警告 Warning] " << step << ": " << ec.message() << std::endl;
  }
}

template <typename Array>
void printArray(const char *name, const Array &values) {
  std::cout << name << " [";  // 打印数组名称。
  for(size_t i = 0; i < values.size(); ++i) {  // 遍历数组中的每个元素。
    if(i != 0) {  // 从第二个元素开始添加逗号。
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(6) << values[i];  // 使用固定小数格式输出。
  }
  std::cout << "]" << std::endl;  // 数组打印结束并换行。
}

template <typename Array>
void printArrayRadAndDeg(const char *name, const Array &values) {
  printArray((std::string(name) + " rad / 弧度").c_str(), values);  // 先打印 SDK 原始弧度值。

  std::cout << name << " deg / 角度 [";  // 再打印角度值。
  for(size_t i = 0; i < values.size(); ++i) {  // 遍历每个关节角。
    if(i != 0) {  // 添加分隔符。
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(3) << radToDeg(values[i]);  // 弧度转角度。
  }
  std::cout << "]" << std::endl;  // 角度数组打印结束。
}

const char *operationStateName(OperationState state) {
  switch(state) {  // 将 SDK 枚举状态转换为中英混合文本。
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

void printRobotSnapshot(XMateRobot &robot, const char *label) {
  error_code ec;  // 用于接收每次 SDK 调用的错误码。

  std::cout << "\n---- " << label << " ----" << std::endl;  // 打印状态快照标题。

  const auto state = robot.operationState(ec);  // 读取当前运行状态。
  if(!ec) {  // 运行状态读取成功。
    std::cout << "operationState / 运行状态: " << operationStateName(state) << std::endl;
  } else {  // 运行状态读取失败。
    std::cout << "operationState error / 读取运行状态失败: " << ec.message() << std::endl;
    ec.clear();  // 清除错误码，继续尝试读取其他信息。
  }

  const auto flange = robot.flangePos(ec);  // 读取法兰相对基坐标系位姿 [X,Y,Z,A,B,C]。
  if(!ec) {  // 法兰位姿读取成功。
    printArray("flange in base [m,rad] / 法兰相对基坐标系", flange);
  } else {  // 法兰位姿读取失败。
    std::cout << "flangePos error / 读取法兰位姿失败: " << ec.message() << std::endl;
    ec.clear();  // 清除错误码。
  }

  const auto jointPosition = robot.jointPos(ec);  // 读取 6 个关节角，单位 rad。
  if(!ec) {  // 关节角读取成功。
    printArrayRadAndDeg("joint position / 关节角", jointPosition);
  } else {  // 关节角读取失败。
    std::cout << "jointPos error / 读取关节角失败: " << ec.message() << std::endl;
    ec.clear();  // 清除错误码。
  }

  const auto jointVelocity = robot.jointVel(ec);  // 读取 6 个关节角速度，单位 rad/s。
  if(!ec) {  // 关节角速度读取成功。
    printArray("joint velocity [rad/s] / 关节角速度", jointVelocity);
  } else {  // 关节角速度读取失败。
    std::cout << "jointVel error / 读取关节角速度失败: " << ec.message() << std::endl;
  }
}

OperationState readOperationState(XMateRobot &robot, const char *label) {
  error_code ec;  // 接收 SDK 错误码。
  const auto state = robot.operationState(ec);  // 读取运行状态。
  checkError(ec, label);  // 读取失败就抛异常。
  return state;  // 返回当前运行状态。
}

bool waitForIdleOrUnknown(XMateRobot &robot, const char *reason) {
  for(int i = 0; i < kStateWaitLoops; ++i) {  // 最多等待约 10 秒。
    error_code ec;  // 每次循环单独接收错误码。
    const auto state = robot.operationState(ec);  // 读取当前运行状态。
    if(ec) {  // 状态读取失败。
      std::cerr << reason << " state read warning / 读取状态警告: "
                << ec.message() << std::endl;
      return false;  // 读取失败时不继续等待。
    }

    if(state == OperationState::idle || state == OperationState::unknown) {  // 控制器已空闲或可继续判断。
      return true;
    }

    std::cout << reason << " / 等待控制器空闲: "
              << operationStateName(state) << std::endl;
    std::this_thread::sleep_for(kStateChangeWait);  // 等待状态刷新。
  }

  return false;  // 超时仍未空闲。
}

void prepareRobotForNrtAutomatic(XMateRobot &robot) {
  error_code ec;  // 本函数内复用的 SDK 错误码变量。

  printRobotSnapshot(robot, "initial state before preparation / 准备前初始状态");

  auto state = readOperationState(robot, "operationState before preparation / 准备前读取状态");
  std::cout << "\n[准备 Prepare] 当前运行状态 current operationState: "
            << operationStateName(state) << std::endl;

  if(state == OperationState::drag) {  // 如果处于拖动模式。
    std::cout << "[准备 Prepare] 退出拖动模式 disableDrag()" << std::endl;
    robot.disableDrag(ec);  // 请求退出拖动模式。
    warnError(ec, "disableDrag / 退出拖动模式");
    ec.clear();  // 清除错误码，后续继续尝试。
    std::this_thread::sleep_for(std::chrono::seconds(2));  // 等待拖动模式退出。
  }

  std::cout << "[准备 Prepare] 停止可能残留的 Jog/运动 stop()" << std::endl;
  robot.stop(ec);  // 尝试停止可能残留的 Jog 或运动。
  warnError(ec, "stop / 停止运动");
  ec.clear();  // 清除错误码。
  std::this_thread::sleep_for(kStateChangeWait);  // 等待控制器状态更新。

  if(!waitForIdleOrUnknown(robot, "waiting for controller idle")) {  // 等待控制器变为空闲。
    state = readOperationState(robot, "operationState after stop/disableDrag / 停止后读取状态");
    throw std::runtime_error(
      std::string("Controller is still busy / 控制器仍忙: ") + operationStateName(state));
  }

  std::cout << "[准备 Prepare] 请求伺服下电 setPowerState(false)" << std::endl;
  robot.setPowerState(false, ec);  // 请求下电，方便后续切换模式。
  warnError(ec, "setPowerState(false) / 请求下电");
  ec.clear();  // 清除错误码。

  std::cout << "[准备 Prepare] 清除伺服报警 clearServoAlarm()" << std::endl;
  robot.clearServoAlarm(ec);  // 尝试清除可软件恢复的伺服报警。
  warnError(ec, "clearServoAlarm / 清除伺服报警");
  ec.clear();  // 清除错误码。

  std::cout << "[准备 Prepare] 切换非实时命令模式 setMotionControlMode(NrtCommand)" << std::endl;
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);  // 切换到非实时命令模式。
  checkError(ec, "setMotionControlMode(NrtCommand) / 切换非实时命令模式");

  std::cout << "[准备 Prepare] 清空运动队列 moveReset()" << std::endl;
  robot.moveReset(ec);  // 清空可能残留的运动队列。
  warnError(ec, "moveReset before automatic / 自动模式前清空队列");
  ec.clear();  // 清除错误码。
}

bool prepareModeAndPowerOnOnce(XMateRobot &robot, int attempt) {
  error_code ec;  // 本轮启动尝试使用的错误码。

  std::cout << "\n[启动 Startup] 第 " << attempt << "/" << kStartupRetryCount
            << " 次准备控制器。/ Startup preparation attempt "
            << attempt << "/" << kStartupRetryCount << std::endl;

  try {  // 尝试执行软恢复。
    prepareRobotForNrtAutomatic(robot);  // 退出拖动/Jog、清报警、切 NRT、清队列。
  } catch(const std::exception &e) {  // 软恢复失败。
    std::cerr << "[启动 Startup] 准备控制器失败 / preparation failed: "
              << e.what() << std::endl;
    return false;  // 返回 false，让外层决定是否重试。
  }

  std::cout << "[模式 Mode] 切换自动模式 setOperateMode(automatic)" << std::endl;
  robot.setOperateMode(OperateMode::automatic, ec);  // 请求进入自动模式。
  if(ec) {  // 自动模式切换失败。
    std::cerr << "[启动 Startup] setOperateMode(automatic) 暂未成功 / not ready yet: "
              << ec.message() << std::endl;
    return false;
  }

  std::this_thread::sleep_for(kStateChangeWait);  // 等待自动模式生效。

  std::cout << "[上电 Power] 伺服上电 setPowerState(true)" << std::endl;
  robot.setPowerState(true, ec);  // 请求伺服上电。
  if(ec) {  // 上电失败。
    std::cerr << "[启动 Startup] setPowerState(true) 暂未成功 / not ready yet: "
              << ec.message() << std::endl;
    return false;
  }

  std::this_thread::sleep_for(kStateChangeWait);  // 等待上电状态稳定。

  std::cout << "[队列 Queue] 再次清空运动队列 moveReset()" << std::endl;
  robot.moveReset(ec);  // 上电后再次清空运动队列。
  if(ec) {  // 清队列失败。
    std::cerr << "[启动 Startup] moveReset 暂未成功 / not ready yet: "
              << ec.message() << std::endl;
    return false;
  }

  return true;  // 本轮启动准备成功。
}

bool prepareModeAndPowerOnWithRetry(XMateRobot &robot) {
  for(int attempt = 1; attempt <= kStartupRetryCount; ++attempt) {  // 最多重试 5 次。
    if(prepareModeAndPowerOnOnce(robot, attempt)) {  // 本轮成功。
      std::cout << "[启动 Startup] 控制器已可用。/ Controller is ready." << std::endl;
      return true;
    }

    if(attempt < kStartupRetryCount) {  // 还有重试机会。
      std::cout << "[启动 Startup] 等待状态稳定后重试。/ Wait for state to settle, then retry."
                << std::endl;
      std::this_thread::sleep_for(kStartupRetryWait);  // 等待异步状态切换完成。
    }
  }

  return false;  // 多次重试仍失败。
}

void waitRobotAndPrint(XMateRobot &robot, const char *commandName) {
  while(true) {  // 循环等待运动结束。
    std::this_thread::sleep_for(kPollPeriod);  // 等待一小段时间，控制打印频率。
    printRobotSnapshot(robot, commandName);  // 打印当前状态、法兰位姿、关节角、角速度。

    error_code ec;  // 接收 operationState 错误码。
    const auto state = robot.operationState(ec);  // 查询当前运行状态。
    checkError(ec, "operationState while waiting / 等待运动时读取状态");

    if(state == OperationState::idle || state == OperationState::unknown) {  // 运动结束。
      const auto last = robot.lastErrorCode();  // 查询最后一次运动是否有错误。
      checkError(last, "motion execution result / 运动执行结果");
      return;  // 正常返回。
    }
  }
}

std::array<double, 6> homeTargetRad() {
  std::array<double, 6> target {};  // 创建 HOME 弧度数组。
  for(size_t i = 0; i < target.size(); ++i) {  // 遍历每个关节。
    target[i] = degToRad(kHomeDeg[i]);  // 把角度转换为弧度。
  }
  return target;  // 返回 HOME 目标。
}

MoveAbsJCommand makeJointCommand(const std::array<double, 6> &target, int speed) {
  std::vector<double> joints(target.begin(), target.end());  // 把 array 转成 vector。
  return MoveAbsJCommand(JointPosition(joints), speed, kFineZone);  // 构造 MoveAbsJ 命令。
}

void executeJointMove(XMateRobot &robot, const std::array<double, 6> &target, const char *name, int speed) {
  error_code ec;  // 接收 SDK 错误码。

  std::cout << "\n>>> " << name << std::endl;  // 打印动作标题。
  printArrayRadAndDeg("target joint / 目标关节", target);  // 打印目标关节角。
  std::cout << "speed / 速度: " << speed << std::endl;  // 打印本次 MoveAbsJ 使用的速度。

  auto command = makeJointCommand(target, speed);  // 创建绝对关节运动命令。
  robot.executeCommand({command}, ec);  // 异步下发运动命令。
  checkError(ec, name);  // 下发失败则抛异常。

  waitRobotAndPrint(robot, name);  // 等待运动完成并打印实时反馈。
}

double maxAbsJointDeltaDeg(const std::array<double, 6> &current, const std::array<double, 6> &target) {
  double maxDeltaDeg = 0.0;  // 保存 6 个关节中最大的绝对差角。
  for(size_t i = 0; i < current.size(); ++i) {  // 遍历 J1 到 J6。
    const double deltaDeg = std::abs(radToDeg(target[i] - current[i]));  // 计算当前关节离目标还差多少度。
    maxDeltaDeg = std::max(maxDeltaDeg, deltaDeg);  // 更新最大差角。
  }
  return maxDeltaDeg;  // 返回最大绝对差角，后续用它决定回 HOME 速度。
}

int calculateHomeReturnSpeed(double maxDeltaDeg) {
  if(maxDeltaDeg <= kHomeReturnSlowZoneDeg) {  // 进入慢速区后不再继续按比例降速。
    return kHomeReturnMinSpeed;  // 使用固定慢速，避免速度越来越接近 0。
  }

  const int proportionalSpeed = static_cast<int>(maxDeltaDeg * kHomeReturnSpeedGain);  // 按最大差角线性估算速度。
  return std::max(kHomeReturnMinSpeed, std::min(kHomeReturnMaxSpeed, proportionalSpeed));  // 限制在最低和最高速度之间。
}

std::array<double, 6> makeHomeReturnSegmentTarget(const std::array<double, 6> &current,
                                                  const std::array<double, 6> &home,
                                                  bool finalSegment) {
  if(finalSegment) {  // 最后一段必须直接指向 HOME，保证最终能精确复位。
    return home;
  }

  std::array<double, 6> target {};  // 保存本段的中间目标。
  for(size_t i = 0; i < target.size(); ++i) {  // 遍历每个关节。
    target[i] = current[i] + (home[i] - current[i]) * kHomeReturnSegmentRatio;  // 走完剩余差角的一部分。
  }
  return target;  // 返回中间目标。
}

void executeVariableSpeedHomeReturn(XMateRobot &robot, const char *name) {
  error_code ec;  // 接收 jointPos 的错误码。
  const auto home = homeTargetRad();  // 读取当前 HOME 姿态，尊重你修改过的 kHomeDeg。

  for(int segment = 1; segment <= kHomeReturnMaxSegments; ++segment) {  // 最多分 5 段回 HOME。
    const auto current = robot.jointPos(ec);  // 每一段开始前重新读取当前关节角。
    checkError(ec, "jointPos before variable HOME return / 变速回 HOME 前读取关节角");

    const double maxDeltaDeg = maxAbsJointDeltaDeg(current, home);  // 计算离 HOME 最远的关节差角。
    if(maxDeltaDeg <= kHomeReturnArrivedDeg) {  // 已经足够接近 HOME。
      std::cout << "[HOME 变速 Variable HOME] 已接近 HOME，最大差角 "
                << std::fixed << std::setprecision(3) << maxDeltaDeg
                << " deg。/ Already close to HOME." << std::endl;
      return;
    }

    const int speed = calculateHomeReturnSpeed(maxDeltaDeg);  // 根据最大差角计算本段速度。
    const bool finalSegment = maxDeltaDeg <= kHomeReturnSlowZoneDeg || segment == kHomeReturnMaxSegments;  // 慢速区或最后一段直达 HOME。
    const auto target = makeHomeReturnSegmentTarget(current, home, finalSegment);  // 生成本段目标。

    std::cout << "\n[HOME 变速 Variable HOME] 第 " << segment << "/" << kHomeReturnMaxSegments
              << " 段，最大差角 " << std::fixed << std::setprecision(3) << maxDeltaDeg
              << " deg，速度 " << speed
              << "。/ Segment " << segment << ", max delta " << maxDeltaDeg
              << " deg, speed " << speed << "." << std::endl;

    executeJointMove(robot, target, name, speed);  // 按本段速度执行本段 MoveAbsJ。

    if(finalSegment) {  // 最后一段已经直接到 HOME。
      return;
    }
  }
}

bool withinSoftLimitDeg(size_t jointIndex, double deg) {
  return deg >= kSoftMinDeg[jointIndex] + kSoftLimitMarginDeg &&
         deg <= kSoftMaxDeg[jointIndex] - kSoftLimitMarginDeg;  // 判断是否在软限位安全余量内。
}

bool makeJointDemoTarget(size_t jointIndex, std::array<double, 6> &target, double &deltaDeg) {
  target = homeTargetRad();  // 每个关节演示都从 HOME 开始构造目标。

  const double homeDeg = kHomeDeg[jointIndex];  // 读取该关节 HOME 角度。
  const double positiveDeg = homeDeg + kJointDemoDeg;  // 优先尝试正向旋转 60 度。
  const double negativeDeg = homeDeg - kJointDemoDeg;  // 如果正向超限，则尝试负向旋转 60 度。

  if(jointIndex == 1 || jointIndex == 2) {  // J2/J3 正向容易碰撞，因此固定只做 -60 度。
    if(withinSoftLimitDeg(jointIndex, negativeDeg)) {  // 即使固定负向，也仍然检查软限位。
      target[jointIndex] = degToRad(negativeDeg);  // 写入 J2/J3 的负向演示目标。
      deltaDeg = -kJointDemoDeg;  // 记录本次旋转方向为 -60 度。
      return true;
    }
    return false;  // 如果 -60 度也不安全，则跳过该关节。
  }

  if(withinSoftLimitDeg(jointIndex, positiveDeg)) {  // 正向目标安全。
    target[jointIndex] = degToRad(positiveDeg);  // 写入正向目标。
    deltaDeg = kJointDemoDeg;  // 记录本次旋转方向。
    return true;
  }

  if(withinSoftLimitDeg(jointIndex, negativeDeg)) {  // 负向目标安全。
    target[jointIndex] = degToRad(negativeDeg);  // 写入负向目标。
    deltaDeg = -kJointDemoDeg;  // 记录本次旋转方向。
    return true;
  }

  return false;  // 正负 60 度都不安全，跳过该关节演示。
}

void runJointDemonstration(XMateRobot &robot) {
  for(size_t joint = 0; joint < 6; ++joint) {  // 从 J1 到 J6 依次演示。
    std::array<double, 6> target {};  // 保存当前关节演示目标。
    double deltaDeg = 0.0;  // 保存本关节实际旋转方向。

    if(!makeJointDemoTarget(joint, target, deltaDeg)) {  // 检查软限位并生成目标。
      std::cout << "\n[跳过 Skip] J" << (joint + 1)
                << " 指定演示方向接近软限位，跳过。/ The selected demo direction is close to soft limits, skipped."
                << std::endl;
      continue;
    }

    std::cout << "\n[关节演示 Joint Demo] 当前移动 J" << (joint + 1)
              << "，旋转 " << deltaDeg << " deg。/ Moving J" << (joint + 1)
              << " by " << deltaDeg << " deg." << std::endl;

    executeJointMove(robot, target, "Joint demo target / 单关节演示目标", kJointSpeed);  // 运动到单关节偏转目标。

    std::cout << "[关节演示 Joint Demo] J" << (joint + 1)
              << " 复位到 HOME。/ Reset J" << (joint + 1) << " back to HOME."
              << std::endl;
    executeVariableSpeedHomeReturn(robot, "Variable return HOME after joint demo / 单关节演示后变速回 HOME");  // 使用变速算法回 HOME。
  }
}

void executeLinearMove(XMateRobot &robot, const std::array<double, 6> &target, const char *name) {
  error_code ec;  // 接收 SDK 错误码。

  std::cout << "\n>>> " << name << std::endl;  // 打印当前直线动作名称。
  printArray("target flange [m,rad] / 目标法兰位姿", target);  // 打印笛卡尔目标。

  MoveLCommand command(CartesianPosition(target), kCartesianSpeed, kFineZone);  // 构造 MoveL 直线运动命令。
  robot.executeCommand({command}, ec);  // 下发 MoveL 命令。
  checkError(ec, name);  // 下发失败则抛异常。

  waitRobotAndPrint(robot, name);  // 等待运动完成。
}

void runSpiralDescent(XMateRobot &robot) {
  error_code ec;  // 接收 SDK 错误码。

  std::cout << "\n[螺旋 Spiral] 读取 HOME 法兰位姿作为螺旋起点。/ Read HOME flange pose as spiral start."
            << std::endl;
  const auto start = robot.flangePos(ec);  // 从 HOME 读取当前法兰位姿 [X,Y,Z,A,B,C]。
  checkError(ec, "flangePos before spiral / 螺旋前读取法兰位姿");

  for(int i = 1; i <= kSpiralPoints; ++i) {  // 将螺旋拆成多个小直线段。
    const double t = static_cast<double>(i) / static_cast<double>(kSpiralPoints);  // 当前进度 0~1。
    const double angle = 2.0 * kPi * kSpiralTurns * t;  // 当前螺旋角度。

    auto target = start;  // 复制起点姿态，保持姿态角 A/B/C 不变。
    target[0] = start[0] + kSpiralRadiusM * std::cos(angle);  // X 方向按圆形轨迹偏移。
    target[1] = start[1] + kSpiralRadiusM * std::sin(angle);  // Y 方向按圆形轨迹偏移。
    target[2] = start[2] - kSpiralDropM * t;                  // Z 方向逐步下降。

    std::cout << "[螺旋 Spiral] 第 " << i << "/" << kSpiralPoints
              << " 段，下落进度 " << std::fixed << std::setprecision(1) << (t * 100.0)
              << "%. / Segment " << i << "/" << kSpiralPoints << "." << std::endl;

    executeLinearMove(robot, target, "Spiral descent segment / 螺旋下落分段");
  }

  std::cout << "\n[螺旋 Spiral] 螺旋完成，使用 MoveAbsJ 回 HOME。/ Spiral complete, return HOME by MoveAbsJ."
            << std::endl;
  executeVariableSpeedHomeReturn(robot, "Variable return HOME after spiral / 螺旋后变速回 HOME");
}

void printPowerFailureHint(const std::string &where) {
  std::cerr << "\n[失败 Failed] " << where << std::endl;
  std::cerr << "程序不能绕过控制器安全限制。/ The program cannot bypass controller safety interlocks." << std::endl;
  std::cerr << "请先在 Robot Assist 中处理: / Please handle in Robot Assist first:" << std::endl;
  std::cerr << "1. 释放实体/外部急停。/ Release physical or external emergency stop." << std::endl;
  std::cerr << "2. 清除报警。/ Clear alarms." << std::endl;
  std::cerr << "3. 退出拖动、力矩控制或实时控制状态。/ Exit drag, torque-control, or real-time-control state." << std::endl;
  std::cerr << "4. 切换到允许上电的控制模式。/ Switch to a mode that permits servo power." << std::endl;
}

} // namespace

int main() {
  setConsoleUtf8();  // 程序启动时先设置控制台编码。

  try {  // 用 try 捕获运行过程中的异常。
    error_code ec;  // 主流程使用的 SDK 错误码。
    XMateRobot robot(kRobotIp);  // 创建机器人对象并指定控制器 IP。

    std::cout << "[连接 Connect] 连接控制器 connectToRobot(): " << kRobotIp << std::endl;
    robot.connectToRobot(ec);  // 建立 SDK 与控制器的连接。
    checkError(ec, "connectToRobot / 连接控制器");

    const auto info = robot.robotInfo(ec);  // 读取控制器版本和机器人型号。
    checkError(ec, "robotInfo / 读取机器人信息");
    std::cout << "Robot version / 控制器版本: " << info.version
              << ", type / 机型: " << info.type
              << ", SDK: " << robot.sdkVersion() << std::endl;

    if(!prepareModeAndPowerOnWithRetry(robot)) {  // 自动整理状态、切模式、上电，最多重试 5 次。
      printPowerFailureHint("startup retry / 启动重试失败");
      return 2;
    }

    std::cout << "[参数 Params] 设置默认速度 setDefaultSpeed(" << kJointSpeed << ")" << std::endl;
    robot.setDefaultSpeed(kJointSpeed, ec);  // 设置默认关节运动速度。
    checkError(ec, "setDefaultSpeed / 设置默认速度");

    std::cout << "[参数 Params] 设置默认转弯区 setDefaultZone(" << kFineZone << ")" << std::endl;
    robot.setDefaultZone(kFineZone, ec);  // 设置默认转弯区。
    checkError(ec, "setDefaultZone / 设置默认转弯区");

    std::cout << "\n[HOME] 先移动到 HOME 初始姿态。/ Move to HOME initial posture first."
              << std::endl;
    executeVariableSpeedHomeReturn(robot, "Variable move to HOME / 变速移动到 HOME");

    std::cout << "\n[确认 Confirm] 即将依次演示 J1 到 J6，每个关节旋转 60 度再复位。"
              << " / The program will demonstrate J1-J6 one by one."
              << std::endl;
    std::cout << "确认现场安全后按 Enter 继续。/ Press Enter after confirming the workspace is clear."
              << std::endl;
    std::cin.get();  // 等待人工确认后再开始演示。

    runJointDemonstration(robot);  // 执行 J1 到 J6 的单关节演示。

    std::cout << "\n[确认 Confirm] 即将执行小范围螺旋下落，然后回 HOME。"
              << " / The program will execute a small spiral descent, then return HOME."
              << std::endl;
    std::cout << "确认末端周围空间安全后按 Enter 继续。/ Press Enter after confirming TCP workspace is clear."
              << std::endl;
    std::cin.get();  // 等待人工确认螺旋运动。

    runSpiralDescent(robot);  // 执行螺旋下落并回 HOME。

    printRobotSnapshot(robot, "final state / 最终状态");

    std::cout << "[断开 Disconnect] 断开 SDK 连接 disconnectFromRobot()" << std::endl;
    robot.disconnectFromRobot(ec);  // 断开 SDK 连接。
    checkError(ec, "disconnectFromRobot / 断开连接");

    std::cout << "\n[完成 Done] study_example finished / 学习示例结束。" << std::endl;
    return 0;  // 正常结束。
  } catch(const std::exception &e) {  // 捕获所有标准异常。
    std::cerr << "[错误 Error] " << e.what() << std::endl;
    return 1;  // 异常结束。
  }
}
