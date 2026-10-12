#pragma warning(disable: 4819)
#pragma warning(disable: 4996)
#pragma warning(disable: 26495)
#pragma warning(disable: 26451)

#ifdef _MSC_VER
#pragma execution_character_set("utf-8")
#endif

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <winsock2.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include "rokae/robot.h"

#pragma comment(lib, "ws2_32.lib")

using namespace std;
using namespace rokae;
using namespace Eigen;

std::string robot_ip = "192.168.2.160";
std::error_code ec;
std::unique_ptr<XMateRobot> robot;

// HOME/默认观察位法兰位姿：[X,Y,Z] 单位为米，[A,B,C] 单位为弧度，角度除以π！！！
// 返回观察位时先对该位姿求固定关节逆解并做安全预检，再使用控制器内部规划的非实时 MoveJ。
constexpr std::array<double, 6> kObservationFlangePose = {
    -0.198162, 0.342077, 0.540341,  -3.042894284,0.090216069,0.210748507
};
constexpr double kPi = 3.14159265358979323846;

// 根据 Robot Assist 软限位截图整理的 xMate ER3 关节软限位，单位是角度。
constexpr std::array<double, 6> kSoftMinDeg = {-165.0, -115.0, -115.0, -165.0, -115.0, -355.0};
constexpr std::array<double, 6> kSoftMaxDeg = { 165.0,  115.0,  115.0,  165.0,  115.0,  355.0};

constexpr int kStartupRetryCount = 5;
constexpr auto kStateChangeWait = std::chrono::milliseconds(500);
constexpr auto kStartupRetryWait = std::chrono::milliseconds(1500);
constexpr int kStateWaitLoops = 20;
constexpr auto kMotionPollPeriod = std::chrono::milliseconds(100);
constexpr auto kImageSettleTime = std::chrono::milliseconds(800);
constexpr int kMotionWaitLoops = 600;  // 60 秒运动超时。

constexpr const char* kDefaultVisionServerIp = "192.168.2.100";
constexpr unsigned short kDefaultVisionServerPort = 5010;
constexpr int kSocketTimeoutMs = 30000;
constexpr double kDesiredTargetZMm = 300.0;
constexpr double kDiveDistanceMm = 200.0;
constexpr double kMinBaseZMm = 0.0;
constexpr double kMinVisionDepthMm = 80.0;
constexpr double kMaxVisionDepthMm = 2000.0;
constexpr double kMaxVisionAbsXyMm = 1000.0;
constexpr double kMinConfidence = 0.50;
constexpr double kMaxInitialCorrectionMm = 800.0;
constexpr double kMaxFineCorrectionAxisMm = 50.0;
// 三轴各 50 mm 时的最大空间距离（sqrt(3) * 50），仅作总距离异常保护。
constexpr double kMaxFineCorrectionDistanceMm = 86.7;
constexpr double kObservationJointArrivalToleranceDeg = 0.20;  // 到位后每轴最大允许误差。
constexpr double kObservationFkPositionToleranceMm = 2.0;       // 逆解再正解的位置允许误差。
constexpr double kObservationFkRotationToleranceDeg = 0.50;    // 逆解再正解的姿态允许误差。

// 以下 speed 参数传给 ROKAE 非实时 MoveL，表示末端最大线速度，单位 mm/s。
// 所有识别位置都使用 zone=0，确保机器人真正停稳后取图。
constexpr int kInitialAlignSpeed = 200;  // 首次移到目标上方约 300 mm 的 MoveL 速度，单位 mm/s。
constexpr int kFineAlignSpeed = 100;     // 二次识别后的短距离精校 MoveL 速度，单位 mm/s。
constexpr int kDiveSpeed = 200;          // 沿相机 +Z 下探的 MoveL 速度，单位 mm/s。
constexpr int kRetractSpeed = 200;       // 正常/失败回撤的 MoveL 速度，单位 mm/s。
constexpr int kObservationMoveJSpeed = 100;  // 非实时 MoveJ 速度，由控制器换算关节速度档。
constexpr int kFineZone = 0;             // 0=fine：每段精确到点并减速至停，便于稳定取图。

std::string vision_server_ip = kDefaultVisionServerIp;
unsigned short vision_server_port = kDefaultVisionServerPort;

// 第一次返回观察位时由固定 XYZABC 求得并确认；该结果用于逆向预检和到位核对。
std::array<double, 6> observation_joint_target_rad{};
bool observation_joint_target_ready = false;
bool observation_joint_target_confirmed = false;

struct VisionTarget {
    std::uint64_t request_id = 0;
    std::uint64_t frame_number = 0;
    double timestamp_ms = 0.0;
    int class_id = -1;
    double confidence = 0.0;
    double pixel_u = 0.0;
    double pixel_v = 0.0;
    Eigen::Vector3d camera_mm = Eigen::Vector3d::Zero();
};

struct VisionReply {
    bool found = false;
    VisionTarget target;
    std::string reason;
};

// 功能：把 Windows 控制台输入输出切换为 UTF-8，保证中英文日志正常显示；不访问机器人。
void setConsoleUtf8() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);  // 将 Windows 控制台输出切到 UTF-8，避免中文提示乱码。
    SetConsoleCP(CP_UTF8);        // 将 Windows 控制台输入也切到 UTF-8。
#endif
}

// 功能：把 SDK 返回的本地代码页错误文本规范为 UTF-8；已经是 UTF-8 时保持原样。
std::string sdkMessageToUtf8(const std::string& text) {
#ifdef _WIN32
    if (text.empty()) {
        return text;
    }

    // 先严格验证 UTF-8，避免把 SDK 新版本已经修正的 UTF-8 文本再次转码。
    const int valid_utf8 = MultiByteToWideChar(
        CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()),
        nullptr, 0);
    if (valid_utf8 > 0) {
        return text;
    }

    // 旧版 Windows SDK 错误文本通常使用系统 ANSI 代码页（中文系统一般为 CP936）。
    const int wide_size = MultiByteToWideChar(
        CP_ACP, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
    if (wide_size <= 0) {
        return text;
    }
    std::wstring wide(static_cast<std::size_t>(wide_size), L'\0');
    MultiByteToWideChar(CP_ACP, 0, text.data(), static_cast<int>(text.size()),
                        &wide[0], wide_size);

    const int utf8_size = WideCharToMultiByte(
        CP_UTF8, 0, wide.data(), wide_size, nullptr, 0, nullptr, nullptr);
    if (utf8_size <= 0) {
        return text;
    }
    std::string utf8(static_cast<std::size_t>(utf8_size), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide.data(), wide_size, &utf8[0], utf8_size,
                        nullptr, nullptr);
    return utf8;
#else
    return text;
#endif
}

// 功能：为现场常见的运动错误码补充稳定的中文解释，不依赖 SDK 消息编码。
const char* knownRokaeErrorMeaning(int value) {
    switch (value) {
    case -50101: return "关节角超过运动范围";
    case -50102: return "运动路径经过奇异点";
    default: return "";
    }
}

// 功能：检查必须成功的 SDK 操作；存在错误码时立即抛出异常，中止当前流程。
std::string describeError(const std::error_code& code) {
    std::ostringstream message;
    message << "value=" << code.value()
            << ", category=" << code.category().name()
            << ", message=" << sdkMessageToUtf8(code.message());
    const char* known_meaning = knownRokaeErrorMeaning(code.value());
    if (known_meaning[0] != '\0') {
        message << ", meaning=" << known_meaning;
    }
    return message.str();
}

// 功能：检查必须成功的 SDK 操作；同时保留数字错误码，避免中文乱码妨碍诊断。
void checkError(const std::error_code& code, const char* step) {
    if (code) {  // SDK 返回错误码时，抛出异常并停止当前流程。
        throw std::runtime_error(std::string(step) + ": " + describeError(code));
    }
}

// 功能：记录允许后续重试的 SDK 错误，不立即终止程序。
void warnError(const std::error_code& code, const char* step) {
    if (code) {  // 可恢复步骤只打印警告，后面交给五次轮询继续尝试。
        std::cerr << "[警告 Warning] " << step << ": "
                  << describeError(code) << std::endl;
    }
}

// 功能：把 SDK 运行状态枚举转换成便于操作人员阅读的中英文文字。
const char* operationStateName(OperationState state) {
    switch (state) {  // 将 SDK 运行状态转换为中英混合文本，便于看懂控制器当前状态。
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

// 功能：读取控制器当前运行状态；读取失败时抛出异常。
OperationState readOperationState(XMateRobot& rb, const char* step) {
    std::error_code code;                 // 单独接收本次读取状态的错误码。
    const auto state = rb.operationState(code);  // 查询控制器当前运行状态。
    checkError(code, step);               // 读取失败时抛出异常。
    return state;                         // 返回当前运行状态。
}

// 功能：轮询等待机器人停止运动并回到空闲状态，超时或读取失败返回 false。
bool waitForIdleOrUnknown(XMateRobot& rb, const char* reason) {
    for (int i = 0; i < kStateWaitLoops; ++i) {  // 最多等待约 10 秒，让控制器从运动/点动切回空闲。
        std::error_code code;
        const auto state = rb.operationState(code);
        if (code) {
            std::cerr << reason << " state read warning / 读取状态警告: "
                      << sdkMessageToUtf8(code.message()) << std::endl;
            return false;
        }

        if (state == OperationState::idle || state == OperationState::unknown) {
            return true;
        }

        std::cout << reason << " / 等待控制器空闲: "
                  << operationStateName(state) << std::endl;
        std::this_thread::sleep_for(kStateChangeWait);
    }
    return false;
}

// 功能：停止残留运动、退出拖动、下电清警并切到非实时命令模式，为自动运行做准备。
void prepareRobotForNrtAutomatic(XMateRobot& rb) {
    std::error_code code;
    auto state = readOperationState(rb, "operationState before preparation / 准备前读取状态");
    std::cout << "\n[准备 Prepare] 当前运行状态 current operationState: "
              << operationStateName(state) << std::endl;

    if (state == OperationState::drag) {  // 如果上一次停在拖动模式，先请求退出拖动。
        std::cout << "[准备 Prepare] 退出拖动模式 disableDrag()" << std::endl;
        rb.disableDrag(code);
        warnError(code, "disableDrag / 退出拖动模式");
        code.clear();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    std::cout << "[准备 Prepare] 停止残留 Jog/运动 stop()" << std::endl;
    rb.stop(code);                         // 停止可能残留的 Jog 或运动指令。
    warnError(code, "stop / 停止运动");
    code.clear();
    std::this_thread::sleep_for(kStateChangeWait);

    if (!waitForIdleOrUnknown(rb, "waiting for controller idle")) {
        state = readOperationState(rb, "operationState after stop / 停止后读取状态");
        throw std::runtime_error(std::string("Controller is still busy / 控制器仍忙: ") +
                                 operationStateName(state));
    }

    std::cout << "[准备 Prepare] 请求伺服下电 setPowerState(false)" << std::endl;
    rb.setPowerState(false, code);          // 切模式前先请求下电，失败也允许后续重试。
    warnError(code, "setPowerState(false) / 请求下电");
    code.clear();

    std::cout << "[准备 Prepare] 清除伺服报警 clearServoAlarm()" << std::endl;
    rb.clearServoAlarm(code);               // 清除可由软件恢复的伺服报警。
    warnError(code, "clearServoAlarm / 清除伺服报警");
    code.clear();

    std::cout << "[准备 Prepare] 切换非实时命令模式 setMotionControlMode(NrtCommand)" << std::endl;
    rb.setMotionControlMode(MotionControlMode::NrtCommand, code);  // 进入普通命令运动模式。
    checkError(code, "setMotionControlMode(NrtCommand) / 切换非实时命令模式");

    std::cout << "[准备 Prepare] 清空运动队列 moveReset()" << std::endl;
    rb.moveReset(code);                     // 清空可能残留的运动队列。
    warnError(code, "moveReset before automatic / 自动模式前清空队列");
}

// 功能：执行一次“准备控制器→自动模式→上电→清空队列”；成功返回 true。
bool prepareModeAndPowerOnOnce(XMateRobot& rb, int attempt) {
    std::error_code code;
    std::cout << "\n[启动 Startup] 第 " << attempt << "/" << kStartupRetryCount
              << " 次准备控制器 / startup preparation attempt "
              << attempt << "/" << kStartupRetryCount << std::endl;

    try {
        prepareRobotForNrtAutomatic(rb);    // 先把控制器整理到 NRT、空闲、可上电的状态。
    }
    catch (const std::exception& e) {
        std::cerr << "[启动 Startup] 准备控制器失败 / preparation failed: "
                  << e.what() << std::endl;
        return false;
    }

    std::cout << "[模式 Mode] 切换自动模式 setOperateMode(automatic)" << std::endl;
    rb.setOperateMode(OperateMode::automatic, code);
    if (code) {
        std::cerr << "[启动 Startup] setOperateMode(automatic) 暂未成功 / not ready yet: "
                  << sdkMessageToUtf8(code.message()) << std::endl;
        return false;
    }

    std::this_thread::sleep_for(kStateChangeWait);
    std::cout << "[上电 Power] 伺服上电 setPowerState(true)" << std::endl;
    rb.setPowerState(true, code);
    if (code) {
        std::cerr << "[启动 Startup] setPowerState(true) 暂未成功 / not ready yet: "
                  << sdkMessageToUtf8(code.message()) << std::endl;
        return false;
    }

    std::this_thread::sleep_for(kStateChangeWait);
    std::cout << "[队列 Queue] 再次清空运动队列 moveReset()" << std::endl;
    rb.moveReset(code);
    if (code) {
        std::cerr << "[启动 Startup] moveReset 暂未成功 / not ready yet: "
                  << sdkMessageToUtf8(code.message()) << std::endl;
        return false;
    }

    return true;
}

// 功能：最多重试指定次数完成控制器模式切换和上电，避免状态尚未稳定导致偶发失败。
bool prepareModeAndPowerOnWithRetry(XMateRobot& rb) {
    for (int attempt = 1; attempt <= kStartupRetryCount; ++attempt) {  // 最多轮询/重试 5 次。
        if (prepareModeAndPowerOnOnce(rb, attempt)) {
            std::cout << "[启动 Startup] 控制器已可用 / Controller is ready." << std::endl;
            return true;
        }

        if (attempt < kStartupRetryCount) {
            std::cout << "[启动 Startup] 等待状态稳定后重试 / Wait for state to settle, then retry."
                      << std::endl;
            std::this_thread::sleep_for(kStartupRetryWait);
        }
    }
    return false;
}

// 功能：启动失败时打印急停、报警、拖动模式等人工排查提示。
void printPowerFailureHint(const char* where) {
    std::cerr << "\n[失败 Failed] " << where << std::endl;
    std::cerr << "程序不能绕过控制器安全限制。/ The program cannot bypass controller safety interlocks." << std::endl;
    std::cerr << "请先确认: / Please check first:" << std::endl;
    std::cerr << "1. 释放实体/外部急停。/ Release physical or external emergency stop." << std::endl;
    std::cerr << "2. 清除报警。/ Clear alarms." << std::endl;
    std::cerr << "3. 退出拖动、力矩控制或实时控制状态。/ Exit drag, torque-control, or real-time-control state." << std::endl;
    std::cerr << "4. 切换到允许上电的控制模式。/ Switch to a mode that permits servo power." << std::endl;
}

// 功能：把 SDK 的 ABC 欧拉角和平移量组合成 4×4 齐次变换矩阵。
Matrix4d eulerToMatrix(Vector3d euler, Vector3d pos) {
    // 欧拉角 + 平移量转换为 4x4 齐次变换矩阵。
    // 当前按 R = Rz(C) * Ry(B) * Rx(A) 处理 ABC 姿态角；如果手册定义不同，需要同步调整这里。
    Matrix3d r;
    r = AngleAxisd(euler.z(), Vector3d::UnitZ()) *
        AngleAxisd(euler.y(), Vector3d::UnitY()) *
        AngleAxisd(euler.x(), Vector3d::UnitX());
    Matrix4d RT = Matrix4d::Identity();
    RT.block<3, 3>(0, 0) = r;
    RT.block<3, 1>(0, 3) = pos;
    return RT;
}

// 功能：把 SDK 法兰位姿 [m, rad] 转成视觉计算使用的 [mm] 齐次矩阵。
template <typename Pose6>
Matrix4d flangePoseToMatrixMm(const Pose6& flange) {
    // robot.flangePos(ec) 返回 [X,Y,Z,A,B,C]：
    // X/Y/Z 单位是 m，A/B/C 单位是 rad。
    // 当前视觉链路 object_coord 和 hand_in_eye() 的平移量使用 mm，
    // 所以这里把法兰平移从 m 转成 mm，保证矩阵乘法单位一致。
    Vector3d pos_mm(flange[0] * 1000.0, flange[1] * 1000.0, flange[2] * 1000.0);
    Vector3d abc_rad(flange[3], flange[4], flange[5]);
    return eulerToMatrix(abc_rad, pos_mm);
}

// 功能：返回手眼标定得到的“法兰到相机”变换矩阵，平移单位为毫米。
Matrix4d hand_in_eye() {
    // 手眼标定矩阵：把视觉端给出的目标点从相机/视觉坐标系转换到末端/法兰相关坐标系。
    // 注意：矩阵方向必须和标定结果一致；当前计算公式是 T_base_flange * hand_in_eye() * object_coord。
    // 这里的平移量单位是 mm，因此 T_base_flange 也要用 mm 平移量。
    Matrix4d m;
    m << 0.00882397, 0.99995778, -0.00256451, -124.60064718,
        -0.99990371, 0.00885091, 0.01068793, -22.19809241,
        0.01071018, 0.00246995, 0.99993959, 34.82308787,
        0.00000000, 0.00000000, 0.00000000, 1.00000000;
       /* 0.49540362, 0.86814727, -0.02992614, -111.62044483,
        -0.86835787, 0.49584859, 0.00942212, 6.83338616,
        0.02301862, 0.02131884, 0.99950770, -113.81849505,
        0.00000000, 0.00000000, 0.00000000, 1.00000000;*/
    /*0.75981629, 0.64720819, -0.06165038, -275.29256957,
        -0.64449883, 0.76229415, 0.05940451, -11.97003904,
        0.08544281, -0.00540291, 0.99632843, 365.42365570,
        0.00000000, 0.00000000, 0.00000000, 1.00000000;*/
    return m;
}

// 功能：把上电、下电、急停和安全门状态转换成可读文字。
const char* powerStateName(PowerState state) {
    switch (state) {
    case PowerState::on: return "on / 上电";
    case PowerState::off: return "off / 下电";
    case PowerState::estop: return "estop / 急停";
    case PowerState::gstop: return "gstop / 安全门";
    case PowerState::unknown: return "unknown / 未知";
    default: return "unhandled / 未处理";
    }
}

// 功能：在发送每个运动前确认机器人对象已创建且处于上电状态。
void ensureRobotPowered(const char* step) {
    if (!robot) {
        throw std::runtime_error(std::string(step) + ": robot is not initialized");
    }
    std::error_code code;
    const auto state = robot->powerState(code);
    checkError(code, step);
    if (state != PowerState::on) {
        throw std::runtime_error(std::string(step) + ": powerState=" + powerStateName(state));
    }
}

// 功能：等待运动完成，并持续监控急停、掉电、运行状态、超时和最后错误码。
void waitRobot(const char* command_name) {
    for (int i = 0; i < kMotionWaitLoops; ++i) {
        std::this_thread::sleep_for(kMotionPollPeriod);

        std::error_code code;
        const auto power = robot->powerState(code);
        checkError(code, "powerState while waiting / 等待运动时读取电源状态");
        if (power == PowerState::estop || power == PowerState::gstop || power == PowerState::off) {
            throw std::runtime_error(std::string(command_name) +
                                     ": motion interrupted, powerState=" + powerStateName(power));
        }

        const auto state = robot->operationState(code);
        checkError(code, "operationState while waiting / 等待运动时读取运行状态");
        if (state == OperationState::idle || state == OperationState::unknown) {
            checkError(robot->lastErrorCode(), "motion execution result / 运动执行结果");
            return;
        }
    }

    std::error_code stop_code;
    robot->stop(stop_code);
    throw std::runtime_error(std::string(command_name) + ": motion timeout / 运动超时");
}

// 功能：连接指定 IP 的机械臂，完成自动模式、上电、速度和转弯区初始化。
void connect_computer() {
    // 连接控制器，并使用和 study_example 相同的五次启动轮询，把控制器整理到可执行非实时命令的状态。
    std::cout << "[连接 Connect] 连接控制器 connectToRobot(): " << robot_ip << std::endl;
    if (!robot) {
        throw std::runtime_error("robot is not initialized / 机器人对象尚未初始化");
    }
    robot->connectToRobot(ec);
    checkError(ec, "connectToRobot / 连接控制器");

    if (!prepareModeAndPowerOnWithRetry(*robot)) {
        printPowerFailureHint("startup retry / 启动重试");
        throw std::runtime_error("startup preparation failed / 启动准备失败");
    }

    robot->setDefaultZone(kFineZone, ec);  // 默认转弯区为 0，识别点必须完全停稳。
    checkError(ec, "setDefaultZone / 设置默认转弯区");
    robot->setDefaultSpeed(kInitialAlignSpeed, ec);  // 未显式给 speed 的命令默认使用 200。
    checkError(ec, "setDefaultSpeed / 设置默认速度");
    std::cout << ">>> Robot Connected / 机器人连接并准备完成。" << std::endl;
}

// 功能：正常结束或异常退出前显式断开 SDK 连接，再析构机器人对象；
// 避免仍处于已连接状态的全局 Robot 对象在进程退出阶段触发旧版 SDK 的容器断言。
void disconnectRobotNoThrow(const char* reason) noexcept {
    if (!robot) {
        return;
    }

    std::cout << "[断开 Disconnect] " << reason
              << "：调用 disconnectFromRobot()。" << std::endl;
    std::error_code disconnect_code;
    robot->disconnectFromRobot(disconnect_code);  // SDK 文档说明：断开前会先停止机器人运动。
    if (disconnect_code) {
        std::cerr << "[警告 Warning] disconnectFromRobot / 断开机器人连接: "
                  << describeError(disconnect_code) << std::endl;
    }
    robot.reset();  // 已显式断开后立即析构，避免拖到全局静态对象销毁阶段。
}

// 功能：区分视觉通信/协议错误，便于调用层执行对应的安全返回策略。
class VisionError : public std::runtime_error {
public:
    explicit VisionError(const std::string& message) : std::runtime_error(message) {}
};

// 功能：标记下探或原路回撤阶段的异常，防止上层再次直接发送 HOME 运动。
class DiveSequenceError : public std::runtime_error {
public:
    explicit DiveSequenceError(const std::string& message) : std::runtime_error(message) {}
};

// 功能：用 RAII 管理 Winsock 的启动和释放，保证退出时自动调用 WSACleanup。
class WinsockSession {
public:
    WinsockSession() {
        WSADATA data;
        const int result = WSAStartup(MAKEWORD(2, 2), &data);
        if (result != 0) {
            throw VisionError("WSAStartup failed: " + std::to_string(result));
        }
    }

    ~WinsockSession() {
        WSACleanup();
    }

    WinsockSession(const WinsockSession&) = delete;
    WinsockSession& operator=(const WinsockSession&) = delete;
};

// 功能：独占管理一个 TCP 套接字，支持移动所有权并在析构时自动关闭。
class SocketHandle {
public:
    explicit SocketHandle(SOCKET socket_value = INVALID_SOCKET) : value_(socket_value) {}
    ~SocketHandle() {
        if (value_ != INVALID_SOCKET) {
            closesocket(value_);
        }
    }

    SOCKET get() const { return value_; }
    SocketHandle(const SocketHandle&) = delete;
    SocketHandle& operator=(const SocketHandle&) = delete;
    SocketHandle(SocketHandle&& other) noexcept : value_(other.value_) {
        other.value_ = INVALID_SOCKET;
    }
    SocketHandle& operator=(SocketHandle&& other) noexcept {
        if (this != &other) {
            if (value_ != INVALID_SOCKET) {
                closesocket(value_);
            }
            value_ = other.value_;
            other.value_ = INVALID_SOCKET;
        }
        return *this;
    }

private:
    SOCKET value_;
};

// 功能：把 Winsock 最后错误码附加到步骤名称中，形成可定位问题的异常文字。
std::string socketError(const char* step) {
    return std::string(step) + ", WSA error=" + std::to_string(WSAGetLastError());
}

// 功能：循环发送完整的一行协议数据，避免一次 send() 只发送部分字节。
void sendAll(SOCKET socket_value, const std::string& data) {
    std::size_t sent = 0;
    while (sent < data.size()) {
        const int result = send(socket_value, data.data() + sent,
                                static_cast<int>(data.size() - sent), 0);
        if (result == SOCKET_ERROR || result == 0) {
            throw VisionError(socketError("send / 发送视觉请求失败"));
        }
        sent += static_cast<std::size_t>(result);
    }
}

// 功能：从 TCP 连接读取一行响应，并限制最大长度，防止无边界数据占用内存。
std::string receiveLine(SOCKET socket_value) {
    std::string line;
    line.reserve(256);

    while (line.size() < 4096) {
        char ch = 0;
        const int result = recv(socket_value, &ch, 1, 0);
        if (result == 0) {
            throw VisionError("vision server closed the connection / 视觉服务端断开连接");
        }
        if (result == SOCKET_ERROR) {
            throw VisionError(socketError("recv / 接收视觉响应失败或超时"));
        }
        if (ch == '\n') {
            return line;
        }
        if (ch != '\r') {
            line.push_back(ch);
        }
    }

    throw VisionError("vision response is too long / 视觉响应行过长");
}

// 功能：连接 Python 视觉服务并完成协议握手；实机模式下拒绝 MOCK 数据源。
SocketHandle connectVisionServer(bool allow_mock) {
    SocketHandle socket_handle(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
    if (socket_handle.get() == INVALID_SOCKET) {
        throw VisionError(socketError("socket / 创建视觉套接字失败"));
    }

    const DWORD timeout_ms = kSocketTimeoutMs;
    if (setsockopt(socket_handle.get(), SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&timeout_ms), sizeof(timeout_ms)) == SOCKET_ERROR ||
        setsockopt(socket_handle.get(), SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&timeout_ms), sizeof(timeout_ms)) == SOCKET_ERROR) {
        throw VisionError(socketError("setsockopt / 设置视觉通信超时失败"));
    }

    SOCKADDR_IN address {};
    address.sin_family = AF_INET;
    address.sin_addr.S_un.S_addr = inet_addr(vision_server_ip.c_str());
    if (address.sin_addr.S_un.S_addr == INADDR_NONE) {
        throw VisionError("invalid IPv4 vision server address / 视觉服务 IPv4 地址无效: " +
                          vision_server_ip);
    }
    address.sin_port = htons(vision_server_port);

    std::cout << "[视觉 Vision] 连接 " << vision_server_ip << ":" << vision_server_port << std::endl;
    if (connect(socket_handle.get(), reinterpret_cast<SOCKADDR*>(&address), sizeof(address)) == SOCKET_ERROR) {
        throw VisionError(socketError("connect / 连接 Python 视觉服务失败"));
    }

    sendAll(socket_handle.get(), "HELLO VISION_PICK 1\n");
    const std::string line = receiveLine(socket_handle.get());
    std::istringstream response(line);
    std::string ready;
    int version = 0;
    std::string mode;
    if (!(response >> ready >> version >> mode) || ready != "READY" || version != 1) {
        throw VisionError("bad vision handshake / 视觉握手响应无效: " + line);
    }
    if (mode == "MOCK" && !allow_mock) {
        throw VisionError("physical execution refuses MOCK vision data / 实机模式拒绝模拟视觉数据");
    }
    if (mode != "MOCK" && mode != "REALSENSE") {
        throw VisionError("unsupported vision mode / 不支持的视觉模式: " + mode);
    }

    std::cout << "[视觉 Vision] 协议就绪，模式 mode=" << mode << std::endl;
    return socket_handle;
}

// 功能：按请求编号、类别和选择规则请求一次新识别，并解析 TARGET/NOT_FOUND/ERROR。
VisionReply requestDetection(SOCKET socket_value, std::uint64_t request_id,
                             int class_filter, const char* selection) {
    std::ostringstream request;
    request << "DETECT " << request_id << " " << class_filter << " " << selection;
    if (robot) {
        // 把当前法兰位姿一起发送给 Python，便于实时生成 end_to_base 并打印基座坐标。
        std::error_code code;
        const auto flange = robot->flangePos(code);
        checkError(code, "flangePos for vision transform / 读取视觉转换所需法兰位姿");
        request << std::setprecision(15)
                << " " << flange[0] << " " << flange[1] << " " << flange[2]
                << " " << flange[3] << " " << flange[4] << " " << flange[5];
    }
    request << "\n";
    sendAll(socket_value, request.str());

    const std::string line = receiveLine(socket_value);
    std::istringstream response(line);
    std::string kind;
    response >> kind;

    if (kind == "TARGET") {
        VisionReply reply;
        reply.found = true;
        auto& target = reply.target;
        if (!(response >> target.request_id >> target.frame_number >> target.timestamp_ms >>
              target.class_id >> target.confidence >> target.pixel_u >> target.pixel_v >>
              target.camera_mm.x() >> target.camera_mm.y() >> target.camera_mm.z())) {
            throw VisionError("malformed TARGET response / TARGET 响应格式错误: " + line);
        }
        if (target.request_id != request_id) {
            throw VisionError("stale or mismatched response id / 响应编号不匹配");
        }
        return reply;
    }

    if (kind == "NOT_FOUND") {
        VisionReply reply;
        std::uint64_t response_id = 0;
        if (!(response >> response_id) || response_id != request_id) {
            throw VisionError("malformed NOT_FOUND response / NOT_FOUND 响应格式错误: " + line);
        }
        std::getline(response, reply.reason);
        if (!reply.reason.empty() && reply.reason.front() == ' ') {
            reply.reason.erase(0, 1);
        }
        return reply;
    }

    if (kind == "ERROR") {
        std::uint64_t response_id = 0;
        response >> response_id;
        std::string reason;
        std::getline(response, reason);
        throw VisionError("vision server error / 视觉服务错误:" + reason);
    }

    throw VisionError("unknown vision response / 未知视觉响应: " + line);
}

// 功能：检查视觉结果的数值、类别、置信度、帧信息和相机坐标范围是否可信。
bool validateVisionTarget(const VisionTarget& target, std::string& reason) {
    if (!target.camera_mm.allFinite() || !std::isfinite(target.confidence) ||
        !std::isfinite(target.pixel_u) || !std::isfinite(target.pixel_v)) {
        reason = "NON_FINITE_VALUE";
        return false;
    }
    if (target.class_id < 0) {
        reason = "INVALID_CLASS";
        return false;
    }
    if (target.frame_number == 0 || target.timestamp_ms <= 0.0) {
        reason = "INVALID_FRAME_METADATA";
        return false;
    }
    if (target.confidence < kMinConfidence) {
        reason = "LOW_CONFIDENCE";
        return false;
    }
    if (std::abs(target.camera_mm.x()) > kMaxVisionAbsXyMm ||
        std::abs(target.camera_mm.y()) > kMaxVisionAbsXyMm) {
        reason = "XY_OUT_OF_RANGE";
        return false;
    }
    if (target.camera_mm.z() < kMinVisionDepthMm ||
        target.camera_mm.z() > kMaxVisionDepthMm) {
        reason = "DEPTH_OUT_OF_RANGE";
        return false;
    }
    return true;
}

// 功能：打印识别请求、帧号、类别、置信度、像素中心和相机 XYZ，便于现场核对。
void printTarget(const char* label, const VisionTarget& target) {
    std::cout << std::fixed << std::setprecision(2)
              << "[视觉 Vision] " << label
              << " request=" << target.request_id
              << " frame=" << target.frame_number
              << " class=" << target.class_id
              << " confidence=" << target.confidence
              << " pixel=(" << target.pixel_u << "," << target.pixel_v << ")"
              << " camera_mm=(" << target.camera_mm.x() << ","
              << target.camera_mm.y() << "," << target.camera_mm.z() << ")"
              << std::endl;
}

// 功能：读取当前法兰位姿 [X,Y,Z,A,B,C]；读取失败时终止当前动作。
std::array<double, 6> readFlangePose(const char* step) {
    std::error_code code;
    const auto flange = robot->flangePos(code);
    checkError(code, step);
    return flange;
}

// 功能：打印法兰位姿，XYZ 使用米，ABC 使用弧度，便于与 SDK 指令直接比较。
void printFlangePose(const char* label, const std::array<double, 6>& pose) {
    std::cout << std::fixed << std::setprecision(6)
              << "[诊断 Diagnostic] " << label
              << " [X,Y,Z,A,B,C]=["
              << pose[0] << "," << pose[1] << "," << pose[2] << ","
              << pose[3] << "," << pose[4] << "," << pose[5] << "]"
              << " (m,rad)" << std::endl;
}

// 功能：拒绝法兰目标 Z≤0 或非有限值的笛卡尔运动，保护约定的工作平面边界。
void ensureDestinationAboveWorkplane(const std::array<double, 6>& flange_pose,
                                     const char* step) {
    const Matrix4d base_to_flange = flangePoseToMatrixMm(flange_pose);
    const double flange_z_mm = base_to_flange(2, 3);
    if (!std::isfinite(flange_z_mm) || flange_z_mm <= kMinBaseZMm) {
        std::ostringstream message;
        message << step << ": destination violates flange base Z>0, flange_z_mm="
                << flange_z_mm;
        throw std::runtime_error(message.str());
    }
}

// 功能：在上电和法兰 Z 安全检查通过后，以指定末端线速度执行一条 MoveL 并等待完成。
void executeLinearPose(const std::array<double, 6>& target, int speed,
                       const char* command_name) {
    ensureRobotPowered(command_name);
    ensureDestinationAboveWorkplane(target, command_name);
    const auto current = readFlangePose(
        "flangePos before MoveL / MoveL 前读取法兰位姿");
    printFlangePose("MoveL current / 直线运动起点", current);
    printFlangePose("MoveL target / 直线运动终点", target);
    std::cout << "[诊断 Diagnostic] MoveL speed=" << speed
              << " mm/s, zone=" << kFineZone << std::endl;
    // speed 是 MoveL 末端最大线速度（mm/s）；zone=0 表示终点精确停车，不跨点圆滑过渡。
    rokae::MoveLCommand command(
        { target[0], target[1], target[2], target[3], target[4], target[5] },
        speed, kFineZone);
    std::error_code code;
    robot->executeCommand({ command }, code);
    checkError(code, command_name);
    waitRobot(command_name);
}

// 功能：按相机/法兰 XY 平面理想平行的假设，近似把相机光轴调整为基座 -Z。
// 当前版本保持法兰 C 不变来近似保持相机 +X 的水平投影方向；精确手眼反算后续加入。
void applyApproximateCameraDownOrientation(std::array<double, 6>& flange,
                                           const char* step) {
    const double original_a = flange[3];
    const double original_b = flange[4];
    const double original_c = flange[5];

    // +pi 与 -pi 姿态等价；选择距离当前 A 更近的一支，避免跨欧拉角边界多转一圈。
    flange[3] = (std::abs(original_a - kPi) <= std::abs(original_a + kPi))
        ? kPi : -kPi;
    flange[4] = 0.0;
    // C 保持不变；平行平面假设下，相机与法兰的平面内安装角是固定常量。

    std::cout << std::fixed << std::setprecision(6)
              << "[姿态 Orientation] " << step
              << " approximate camera +Z -> base -Z / 近似令相机光轴垂直向下"
              << " ABC_from=(" << original_a << "," << original_b << ","
              << original_c << ")"
              << " ABC_to=(" << flange[3] << "," << flange[4] << ","
              << flange[5] << ")"
              << std::endl;
    std::cerr << "[警告 Warning] 当前尚未补偿相机相对法兰的安装平移；"
                 "二次识别偏差仍受 50 mm 安全限值约束。"
              << std::endl;
}

// 功能：把相机坐标系平移增量旋转到基座系，生成新的法兰目标位姿。
std::array<double, 6> cameraDeltaTargetPose(const Eigen::Vector3d& delta_camera_mm,
                                            double max_distance_mm,
                                            const char* step,
                                            bool approximate_camera_down = false) {
    const double distance_mm = delta_camera_mm.norm();
    if (!delta_camera_mm.allFinite() || distance_mm > max_distance_mm) {
        std::ostringstream message;
        message << step << ": camera correction rejected, distance_mm=" << distance_mm
                << ", limit_mm=" << max_distance_mm;
        throw std::runtime_error(message.str());
    }

    auto flange = readFlangePose("flangePos before camera translation / 相机平移前读取法兰位姿");
    printFlangePose("camera translation current flange / 相机平移前法兰", flange);
    const Matrix4d base_to_camera = flangePoseToMatrixMm(flange) * hand_in_eye();
    const Eigen::Vector3d delta_base_mm =
        base_to_camera.block<3, 3>(0, 0) * delta_camera_mm;

    flange[0] += delta_base_mm.x() / 1000.0;
    flange[1] += delta_base_mm.y() / 1000.0;
    flange[2] += delta_base_mm.z() / 1000.0;
    if (approximate_camera_down) {
        applyApproximateCameraDownOrientation(flange, step);
    }
    printFlangePose("camera translation target flange / 相机平移目标法兰", flange);

    std::cout << std::fixed << std::setprecision(2)
              << "[运动 Motion] " << step
              << " delta_camera_mm=(" << delta_camera_mm.x() << ","
              << delta_camera_mm.y() << "," << delta_camera_mm.z() << ")"
              << " delta_base_mm=(" << delta_base_mm.x() << ","
              << delta_base_mm.y() << "," << delta_base_mm.z() << ")"
              << std::endl;
    ensureDestinationAboveWorkplane(flange, step);
    return flange;
}

// 功能：比较识别 XYZ 与 (0,0,300)，校验各轴/总修正量后移动相机完成光轴对准。
void alignCameraToTarget(const VisionTarget& target, double max_correction_mm,
                         double max_axis_correction_mm,
                         int linear_speed,
                         const char* step,
                         bool approximate_camera_down = false) {
    const Eigen::Vector3d desired_camera_mm(0.0, 0.0, kDesiredTargetZMm);
    const Eigen::Vector3d delta_camera_mm = target.camera_mm - desired_camera_mm;
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(delta_camera_mm[axis]) > max_axis_correction_mm) {
            std::ostringstream message;
            message << step << ": axis " << axis
                    << " correction rejected, delta_mm=" << delta_camera_mm[axis]
                    << ", axis_limit_mm=" << max_axis_correction_mm;
            throw std::runtime_error(message.str());
        }
    }
    if (delta_camera_mm.norm() < 0.5 && !approximate_camera_down) {
        std::cout << "[运动 Motion] " << step << ": correction below 0.5 mm, skipped."
                  << std::endl;
        return;
    }
    const auto pose = cameraDeltaTargetPose(
        delta_camera_mm, max_correction_mm, step, approximate_camera_down);
    executeLinearPose(pose, linear_speed, step);
}

// 功能：把弧度关节角打印为弧度和角度，便于与 Robot Assist 状态监控交叉检查。
void printJointArray(const char* label, const std::array<double, 6>& joints) {
    std::cout << std::fixed << std::setprecision(6)
              << "[诊断 Diagnostic] " << label << " rad=[";
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i != 0) std::cout << ",";
        std::cout << joints[i];
    }
    std::cout << "] deg=[" << std::setprecision(3);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i != 0) std::cout << ",";
        std::cout << joints[i] * 180.0 / kPi;
    }
    std::cout << "]" << std::endl;
}

// 功能：拒绝任何超过 Robot Assist 软限位或包含非有限值的关节目标。
void validateJointSoftLimits(const std::array<double, 6>& joints, const char* label) {
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const double deg = joints[i] * 180.0 / kPi;
        if (!std::isfinite(deg) || deg < kSoftMinDeg[i] || deg > kSoftMaxDeg[i]) {
            std::ostringstream message;
            message << label << ": J" << (i + 1) << "=" << deg
                    << " deg, allowed=[" << kSoftMinDeg[i] << ","
                    << kSoftMaxDeg[i] << "] deg";
            throw std::runtime_error(message.str());
        }
    }
}

// 功能：从固定观察位 XYZABC 求一次六轴逆解，并用正解、软限位验证后缓存。
// 同一进程内始终复用这组预检关节角，避免重复逆解并用于到位核对。
const std::array<double, 6>& resolveObservationJointTarget() {
    if (observation_joint_target_ready) {
        return observation_joint_target_rad;
    }

    auto model = robot->model();
    std::error_code code;
    const auto candidate = model.calcIk(rokae::CartesianPosition(kObservationFlangePose), code);
    checkError(code, "calcIk HOME observation / 计算观察位关节逆解");
    validateJointSoftLimits(candidate, "HOME IK target / 观察位逆解目标");

    const auto forward = model.calcFk(candidate, code);
    checkError(code, "calcFk HOME verification / 观察位逆解正解回验");

    const Eigen::Vector3d target_xyz(
        kObservationFlangePose[0], kObservationFlangePose[1], kObservationFlangePose[2]);
    const Eigen::Vector3d fk_xyz(forward.trans[0], forward.trans[1], forward.trans[2]);
    const double position_error_mm = (fk_xyz - target_xyz).norm() * 1000.0;

    const Matrix3d target_rotation = eulerToMatrix(
        Eigen::Vector3d(kObservationFlangePose[3], kObservationFlangePose[4],
                        kObservationFlangePose[5]),
        Eigen::Vector3d::Zero()).block<3, 3>(0, 0);
    const Matrix3d fk_rotation = eulerToMatrix(
        Eigen::Vector3d(forward.rpy[0], forward.rpy[1], forward.rpy[2]),
        Eigen::Vector3d::Zero()).block<3, 3>(0, 0);
    const double trace_value = (target_rotation.transpose() * fk_rotation).trace();
    const double cosine = std::fmax(-1.0, std::fmin(1.0, (trace_value - 1.0) * 0.5));
    const double rotation_error_deg = std::acos(cosine) * 180.0 / kPi;

    std::cout << std::fixed << std::setprecision(3)
              << "[诊断 Diagnostic] HOME IK->FK error / 观察位逆解回验: position="
              << position_error_mm << " mm, rotation=" << rotation_error_deg << " deg"
              << std::endl;
    if (position_error_mm > kObservationFkPositionToleranceMm ||
        rotation_error_deg > kObservationFkRotationToleranceDeg) {
        std::ostringstream message;
        message << "HOME IK/FK verification failed / 观察位逆解回验超差: position="
                << position_error_mm << " mm, rotation=" << rotation_error_deg << " deg";
        throw std::runtime_error(message.str());
    }

    observation_joint_target_rad = candidate;
    observation_joint_target_ready = true;
    printJointArray("HOME fixed joint target / 观察位固定关节目标",
                    observation_joint_target_rad);
    return observation_joint_target_rad;
}

// 功能：使用控制器内部规划的非实时 MoveJ 返回固定观察位。
void moveToObservation() {
    ensureRobotPowered("move to observation / 返回观察位");
    const auto current_pose =
        readFlangePose("flangePos before HOME / 回 HOME 前读取法兰位姿");
    printFlangePose("HOME start flange / 返回观察位前法兰", current_pose);
    ensureDestinationAboveWorkplane(current_pose, "current flange before HOME / 回 HOME 前法兰位置");

    ensureDestinationAboveWorkplane(
        kObservationFlangePose, "HOME observation target / HOME 观察位目标");
    printFlangePose("HOME target flange / 观察位目标法兰", kObservationFlangePose);

    std::error_code code;
    const auto start_joints = robot->jointPos(code);
    checkError(code, "jointPos before NRT HOME / 非实时 HOME 前读取关节角");
    validateJointSoftLimits(start_joints, "HOME start joints / 返回观察位起始关节");
    printJointArray("HOME start joints / 返回观察位起始关节", start_joints);

    const auto& target_joints = resolveObservationJointTarget();
    printJointArray("HOME target joints / 返回观察位目标关节", target_joints);

    if (!observation_joint_target_confirmed) {
        std::cout << "[安全确认 Safety] 请核对以上六轴观察位目标与 Robot Assist。"
                     "输入 RUN_HOME 才执行首次非实时 MoveJ 返回: ";
        std::string confirmation;
        if (!std::getline(std::cin, confirmation) || confirmation != "RUN_HOME") {
            throw std::runtime_error(
                "HOME joint target not confirmed / 未确认观察位关节逆解，不发送运动");
        }
        observation_joint_target_confirmed = true;
    }

    double max_delta_deg = 0.0;
    for (std::size_t i = 0; i < start_joints.size(); ++i) {
        max_delta_deg = std::max(
            max_delta_deg, std::abs(target_joints[i] - start_joints[i]) * 180.0 / kPi);
    }

    if (max_delta_deg > kObservationJointArrivalToleranceDeg) {
        rokae::MoveJCommand command(
            { kObservationFlangePose[0], kObservationFlangePose[1], kObservationFlangePose[2],
              kObservationFlangePose[3], kObservationFlangePose[4], kObservationFlangePose[5] },
            kObservationMoveJSpeed, kFineZone);
        std::cout << std::fixed << std::setprecision(2)
                  << "[运动 Motion] NRT MoveJ HOME / 控制器非实时规划返回观察位"
                  << " speed=" << kObservationMoveJSpeed
                  << ", zone=" << kFineZone
                  << ", max_joint_delta=" << max_delta_deg << " deg" << std::endl;

        // 非实时 MoveJ 由控制器内部规划；executeCommand() 异步下发，waitRobot() 等待到位。
        robot->executeCommand({ command }, code);
        checkError(code, "execute MoveJ HOME / 下发非实时 MoveJ 返回观察位");
        waitRobot("NRT MoveJ HOME / 非实时 MoveJ 返回观察位");
    }
    else {
        std::cout << "[运动 Motion] 已在观察位关节容差内，跳过 NRT MoveJ。" << std::endl;
    }

    const auto arrived_joints = robot->jointPos(code);
    checkError(code, "jointPos after NRT HOME / 非实时 HOME 后读取关节角");
    printJointArray("HOME arrived joints / 到达观察位关节", arrived_joints);
    for (std::size_t i = 0; i < arrived_joints.size(); ++i) {
        const double error_deg =
            std::abs(arrived_joints[i] - target_joints[i]) * 180.0 / kPi;
        if (error_deg > kObservationJointArrivalToleranceDeg) {
            std::ostringstream message;
            message << "HOME joint arrival error / 观察位关节到位误差: J" << (i + 1)
                    << " error=" << error_deg << " deg, limit="
                    << kObservationJointArrivalToleranceDeg << " deg";
            throw std::runtime_error(message.str());
        }
    }

    const auto arrived_pose =
        readFlangePose("flangePos after HOME / 到达 HOME 后读取法兰位姿");
    printFlangePose("HOME arrived flange / 到达观察位法兰", arrived_pose);
    ensureDestinationAboveWorkplane(arrived_pose, "HOME flange / HOME 法兰位置");
    std::this_thread::sleep_for(kImageSettleTime);
}

// 功能：目标丢失或普通错误时，仅在仍上电的情况下尝试返回 HOME；急停时不自动移动。
void returnToObservationIfSafe(const std::string& reason) {
    std::cerr << "[流程 Abort] " << reason << std::endl;
    std::error_code code;
    const auto power = robot->powerState(code);
    if (code || power != PowerState::on) {
        std::cerr << "[安全 Safety] 电源/急停状态不允许自动回撤，不发送任何运动。"
                  << std::endl;
        return;
    }

    try {
        moveToObservation();
    }
    catch (const std::exception& error) {
        std::cerr << "[安全 Safety] 返回观察位失败，停止自动动作: "
                  << error.what() << std::endl;
    }
}

// 功能：把 Python 返回的视觉失败代码翻译为现场可直接理解的中英文诊断。
std::string describeVisionFailure(const std::string& reason) {
    if (reason == "TARGET_LOST") {
        return "target lost / 未检测到目标";
    }
    if (reason == "INVALID_DEPTH") {
        return "invalid depth / 检测框存在，但中心区域深度无效";
    }
    if (reason == "Z_JUMP") {
        return "Z jump / 相邻有效帧的 Z 深度跳变超过 50 mm";
    }
    if (reason == "CENTER_JUMP") {
        return "center jump / 相邻有效帧的中心跳变超过 35 px";
    }
    if (reason == "UNSTABLE_TARGET") {
        return "unstable target / 目标未在规定时间内达到稳定条件";
    }
    return "vision target unavailable / 视觉目标不可用";
}

// 功能：统一处理视觉成功和各类失败；失败日志保留具体类别并执行安全返回。
bool targetOrReturn(const VisionReply& reply, const char* stage, VisionTarget& target) {
    if (!reply.found) {
        returnToObservationIfSafe(
            std::string(stage) + ": " + describeVisionFailure(reply.reason) +
            " [" + reply.reason + "]");
        return false;
    }

    std::string reason;
    if (!validateVisionTarget(reply.target, reason)) {
        returnToObservationIfSafe(std::string(stage) + ": invalid target / 无效目标: " + reason);
        return false;
    }
    target = reply.target;
    printTarget(stage, target);
    return true;
}

// 功能：确认二次识别的帧号和时间戳均晚于第一次，防止使用移动前的旧图像。
bool isNewerVisionFrame(const VisionTarget& newer, const VisionTarget& older) {
    return newer.frame_number > older.frame_number &&
           newer.timestamp_ms > older.timestamp_ms;
}

// 功能：下探异常后先停止；仅在上电且空闲时原路退回下探前位姿并返回 HOME。
void retreatAfterDiveFailureIfSafe(const std::array<double, 6>& pre_dive_pose,
                                   const std::string& reason) {
    std::cerr << "[下探失败 Dive failed] " << reason << std::endl;

    std::error_code stop_code;
    robot->stop(stop_code);
    if (stop_code) {
        std::cerr << "[安全 Safety] stop() 失败，禁止自动回撤: "
                  << sdkMessageToUtf8(stop_code.message()) << std::endl;
        return;
    }
    std::this_thread::sleep_for(kStateChangeWait);

    std::error_code state_code;
    const auto power = robot->powerState(state_code);
    if (state_code || power != PowerState::on) {
        std::cerr << "[安全 Safety] 急停、掉电或状态不可读，禁止自动回撤。" << std::endl;
        return;
    }

    const auto operation = robot->operationState(state_code);
    if (state_code || (operation != OperationState::idle &&
                       operation != OperationState::unknown)) {
        std::cerr << "[安全 Safety] 控制器未回到空闲状态，禁止自动回撤。" << std::endl;
        return;
    }

    try {
        executeLinearPose(pre_dive_pose, kRetractSpeed,
                          "retract after failed dive / 下探失败后原路回撤");
        moveToObservation();
    }
    catch (const std::exception& error) {
        std::cerr << "[安全 Safety] 下探失败后的回撤未完成，停止自动动作: "
                  << error.what() << std::endl;
    }
}

// 功能：执行一轮“首次识别→300 mm 对准→二次校准→下探→回撤→HOME”的完整流程。
bool runVisionGuidedCycle(SOCKET socket_value, std::uint64_t& next_request_id) {
    VisionTarget first;
    if (!targetOrReturn(requestDetection(socket_value, next_request_id++, -1, "CLOSEST"),
                        "first detection / 首次识别", first)) {
        return false;
    }

    alignCameraToTarget(first, kMaxInitialCorrectionMm, kMaxInitialCorrectionMm,
                        kInitialAlignSpeed,
                        "initial alignment / 首次移动到目标前 300 mm",
                        true);
    std::this_thread::sleep_for(kImageSettleTime);

    VisionTarget second;
    if (!targetOrReturn(requestDetection(socket_value, next_request_id++, first.class_id, "CLOSEST"),
                        "second detection / 二次识别", second)) {
        return false;
    }
    if (!isNewerVisionFrame(second, first)) {
        returnToObservationIfSafe("second detection is not a fresh frame / 二次识别不是新帧");
        return false;
    }

    alignCameraToTarget(second, kMaxFineCorrectionDistanceMm,
                        kMaxFineCorrectionAxisMm,
                        kFineAlignSpeed,
                        "fine alignment / 二次校准对准光轴");

    const auto pre_dive_pose =
        readFlangePose("flangePos before dive / 下探前读取法兰位姿");
    const auto dive_pose = cameraDeltaTargetPose(
        Eigen::Vector3d(0.0, 0.0, kDiveDistanceMm),
        kDiveDistanceMm + 0.1, "camera +Z dive / 沿相机 +Z 下探");

    try {
        executeLinearPose(dive_pose, kDiveSpeed, "camera +Z dive / 沿相机 +Z 下探");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 当前阶段只做象征性下探，不触发夹爪 DO。成功后沿原直线路径回撤。
        executeLinearPose(pre_dive_pose, kRetractSpeed,
                          "retract after dive / 下探后原路回撤");
    }
    catch (const std::exception& error) {
        retreatAfterDiveFailureIfSafe(pre_dive_pose, error.what());
        throw DiveSequenceError(error.what());
    }
    moveToObservation();
    std::cout << "[完成 Done] 本轮双重校准与象征性下探完成，已返回观察位。" << std::endl;
    return true;
}

// 功能：建立一次视觉连接；每轮结束后等待 RE，再按 1、2 重新编号并执行完整流程。
void socketClientLoop() {
    WinsockSession winsock;
    auto socket_handle = connectVisionServer(false);

    try {
        while (true) {
            // Python 端把 request=2 识别为“二次识别”并允许最多两帧漏检，
            // 因此每个完整周期都从 1 重新编号，而不是继续使用 3、4、5……。
            std::uint64_t request_id = 1;
            const bool completed = runVisionGuidedCycle(socket_handle.get(), request_id);
            std::cout << (completed
                ? "[周期 Cycle] 本轮正常完成。"
                : "[周期 Cycle] 本轮已安全终止并返回观察位。")
                      << std::endl;

            while (true) {
                std::cout << "[循环 Repeat] 输入 RE 再执行一轮；输入 EXIT 正常断开并退出: ";
                std::string command;
                if (!std::getline(std::cin, command)) {
                    std::cout << "\n[循环 Repeat] 输入流已关闭，正常退出。" << std::endl;
                    return;
                }
                if (command == "RE") {
                    std::cout << "[循环 Repeat] 已确认 RE，开始下一完整识别周期。"
                              << std::endl;
                    break;
                }
                if (command == "EXIT" || command == "Q") {
                    std::cout << "[循环 Repeat] 收到退出指令，正常结束实机程序。"
                              << std::endl;
                    return;
                }
                std::cout << "[循环 Repeat] 无效输入；请输入 RE、EXIT 或 Q。" << std::endl;
            }
        }
    }
    catch (const DiveSequenceError&) {
        // 下探异常分支已经按电源/急停状态尝试原路回撤；这里禁止再次直接发运动。
        throw;
    }
    catch (const VisionError& error) {
        returnToObservationIfSafe(std::string("vision communication failed / 视觉通信失败: ") +
                                  error.what());
        throw;
    }
    catch (const std::exception& error) {
        returnToObservationIfSafe(std::string("vision-guided cycle failed / 视觉引导流程失败: ") +
                                  error.what());
        throw;
    }
}

// 功能：只测试 Python 视觉服务和返回数据，不创建机器人对象、不发送运动指令。
void runVisionOnlyTest() {
    WinsockSession winsock;
    auto socket_handle = connectVisionServer(true);
    const VisionReply reply = requestDetection(socket_handle.get(), 1, -1, "CLOSEST");
    if (!reply.found) {
        std::cout << "[视觉测试 Vision test] NOT_FOUND: " << reply.reason << std::endl;
        return;
    }
    std::string reason;
    printTarget("vision-only test / 仅视觉测试", reply.target);
    std::cout << "[视觉测试 Vision test] validation="
              << (validateVisionTarget(reply.target, reason) ? "PASS" : "FAIL")
              << (reason.empty() ? "" : ", reason=" + reason) << std::endl;
}

// 功能：判断命令行中是否存在指定开关，例如 --execute 或 --vision-test。
bool hasArgument(int argc, char* argv[], const std::string& expected) {
    for (int i = 1; i < argc; ++i) {
        if (argv[i] == expected) {
            return true;
        }
    }
    return false;
}

// 功能：读取命令行选项后面的字符串值；没有提供时使用默认值。
std::string argumentValue(int argc, char* argv[], const std::string& name,
                          const std::string& fallback) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (argv[i] == name) {
            return argv[i + 1];
        }
    }
    return fallback;
}

// 功能：解析视觉服务器 IP 和端口，并拒绝不在 1～65535 范围内的端口。
void configureVisionEndpoint(int argc, char* argv[]) {
    vision_server_ip = argumentValue(argc, argv, "--vision-ip", kDefaultVisionServerIp);
    const std::string port_text =
        argumentValue(argc, argv, "--vision-port", std::to_string(kDefaultVisionServerPort));
    try {
        const unsigned long port = std::stoul(port_text);
        if (port == 0 || port > 65535) {
            throw std::out_of_range("port");
        }
        vision_server_port = static_cast<unsigned short>(port);
    }
    catch (const std::exception&) {
        throw std::runtime_error("invalid --vision-port / 视觉服务端口无效: " + port_text);
    }
}

// 功能：程序入口；区分安全默认模式、纯视觉测试和需要 RUN 二次确认的实机模式。
int main(int argc, char* argv[]) {
    setConsoleUtf8();

    try {
        configureVisionEndpoint(argc, argv);
        if (hasArgument(argc, argv, "--vision-test")) {
            runVisionOnlyTest();
            return 0;
        }

        if (!hasArgument(argc, argv, "--execute")) {
            std::cout << "默认不连接或移动机器人。/ Robot motion is disabled by default.\n"
                      << "仅测试 YOLO/RealSense 通信: vision_pick.exe --vision-test "
                      << "[--vision-ip 127.0.0.1] [--vision-port 5010]\n"
                      << "实机执行: vision_pick.exe --execute "
                      << "[--vision-ip 192.168.2.100] [--vision-port 5010]" << std::endl;
            return 0;
        }

        std::cout << "即将连接并移动真实机器人。确认急停可用、工作区清空后输入 RUN: ";
        std::string confirmation;
        std::getline(std::cin, confirmation);
        if (confirmation != "RUN") {
            std::cout << "未确认，程序退出且不连接机器人。" << std::endl;
            return 0;
        }

        robot = std::make_unique<XMateRobot>(robot_ip);
        connect_computer();
        moveToObservation();
        socketClientLoop();
        disconnectRobotNoThrow("实机循环正常结束");
    }
    catch (const rokae::Exception& error) {
        std::error_code stop_code;
        if (robot) {
            robot->stop(stop_code);  // 仅请求规划停止，不执行自动回撤或清除急停。
        }
        // 旧版 Windows SDK 的异常字符串可能使用系统 ANSI/GBK；统一转成 UTF-8 后再输出。
        std::cerr << "[Rokae Error / 机器人错误] "
                  << sdkMessageToUtf8(error.what()) << std::endl;
        disconnectRobotNoThrow("机器人异常退出");
        return 1;
    }
    catch (const std::exception& error) {
        std::error_code stop_code;
        if (robot) {
            robot->stop(stop_code);  // 任何未处理故障都停止队列；急停后不会自动继续运动。
        }
        std::cerr << "[Error / 错误] "
                  << sdkMessageToUtf8(error.what()) << std::endl;
        disconnectRobotNoThrow("程序异常退出");
        return 1;
    }
    return 0;
}
