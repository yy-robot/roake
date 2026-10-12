#pragma warning(disable: 4819)
#pragma warning(disable: 4996)

#ifdef _MSC_VER
#pragma execution_character_set("utf-8")
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#endif

#include "rokae/robot.h"

namespace {

using rokae::CartesianPosition;
using rokae::MotionControlMode;
using rokae::MoveJCommand;
using rokae::OperateMode;
using rokae::OperationState;
using rokae::PowerState;
using rokae::RtSupportedFields::jointPos_m;
using rokae::XMateRobot;

constexpr double kPi = 3.14159265358979323846;
constexpr const char* kDefaultRobotIp = "192.168.2.160";

// 测试点 A、B：XYZ 单位为米，ABC 单位为弧度。
constexpr std::array<double, 6> kPointA = {
    0.570, 0.200, 0.550, -2.7925, 0.0, -1.2217
};
constexpr std::array<double, 6> kPointB = {
    -0.220, 0.500, 0.470, -3.14159, 0.0, 0.52359
};

// 第一段使用控制器普通 MoveJ。这里的 speed 是 SDK 的非实时速度参数，不是 mm/s。
// 取 50 是为了先以保守速度到达测试起点 A；zone=0 表示到 A 完全停车。
constexpr int kMoveToASpeed = 50;
constexpr int kFineZone = 0;

// 第二段使用实时 MoveJ 的单段关节空间规划。factor 是 0~1 的无量纲速度比例。
// 默认仅用 5%，并限制命令行最高只能设到 20%，先验证实时网络和轨迹行为。
constexpr double kDefaultRtSpeedFactor = 0.05;
constexpr double kMaxAllowedRtSpeedFactor = 0.20;
constexpr unsigned kRtNetworkTolerancePercent = 20;
constexpr double kRtStartJointToleranceDeg = 0.50;
constexpr double kArrivalJointToleranceDeg = 0.50;

constexpr std::array<double, 6> kSoftMinDeg = {
    -165.0, -115.0, -115.0, -165.0, -115.0, -355.0
};
constexpr std::array<double, 6> kSoftMaxDeg = {
     165.0,  115.0,  115.0,  165.0,  115.0,  355.0
};

constexpr auto kPollPeriod = std::chrono::milliseconds(100);
constexpr int kMotionWaitLoops = 900;  // 最长等待 90 秒。

struct Options {
    bool execute = false;
    std::string robot_ip = kDefaultRobotIp;
    std::string local_ip;
    double rt_speed_factor = kDefaultRtSpeedFactor;
};

// 功能：把控制台输入输出切换为 UTF-8，保证中文日志正常显示。
void setConsoleUtf8() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
}

// 功能：将旧版 SDK 可能返回的系统 ANSI/GBK 异常文本转换成 UTF-8。
std::string sdkMessageToUtf8(const std::string& text) {
#ifdef _WIN32
    if (text.empty()) {
        return text;
    }

    const int valid_utf8 = MultiByteToWideChar(
        CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()),
        nullptr, 0);
    if (valid_utf8 > 0) {
        return text;
    }

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
    WideCharToMultiByte(CP_UTF8, 0, wide.data(), wide_size,
                        &utf8[0], utf8_size, nullptr, nullptr);
    return utf8;
#else
    return text;
#endif
}

// 功能：把 SDK error_code 的数字、类别和文本一起保留，避免只看乱码无法定位。
std::string describeError(const std::error_code& code) {
    std::ostringstream out;
    out << "value=" << code.value()
        << ", category=" << code.category().name()
        << ", message=" << sdkMessageToUtf8(code.message());
    return out.str();
}

// 功能：必须成功的 SDK 调用若返回错误码，立即中止当前测试阶段。
void checkError(const std::error_code& code, const char* step) {
    if (code) {
        throw std::runtime_error(std::string(step) + ": " + describeError(code));
    }
}

// 功能：允许清理流程继续执行，但把错误记录出来。
void warnError(const std::error_code& code, const char* step) {
    if (code) {
        std::cerr << "[警告 Warning] " << step << ": "
                  << describeError(code) << std::endl;
    }
}

// 功能：将 SDK 运行状态转换成可读文字。
const char* operationStateName(OperationState state) {
    switch (state) {
    case OperationState::idle: return "idle / 空闲";
    case OperationState::moving: return "moving / 运动中";
    case OperationState::rtControlling: return "rtControlling / 实时控制";
    case OperationState::jog: return "jog / 点动";
    case OperationState::drag: return "drag / 拖动";
    case OperationState::unknown: return "unknown / 未知";
    default: return "other / 其他";
    }
}

// 功能：打印 XYZABC 位姿，单位保持为 SDK 的 m、rad。
void printPose(const char* title, const std::array<double, 6>& pose) {
    std::cout << std::fixed << std::setprecision(6)
              << "[诊断 Diagnostic] " << title << " [X,Y,Z,A,B,C]=["
              << pose[0] << "," << pose[1] << "," << pose[2] << ","
              << pose[3] << "," << pose[4] << "," << pose[5]
              << "] (m,rad)" << std::endl;
}

// 功能：同时用弧度和角度打印六轴角，便于与 Robot Assist 对照。
void printJoints(const char* title, const std::array<double, 6>& joints) {
    std::cout << std::fixed << std::setprecision(6)
              << "[诊断 Diagnostic] " << title << " rad=[";
    for (std::size_t i = 0; i < joints.size(); ++i) {
        std::cout << (i == 0 ? "" : ",") << joints[i];
    }
    std::cout << "] deg=[" << std::setprecision(3);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        std::cout << (i == 0 ? "" : ",") << joints[i] * 180.0 / kPi;
    }
    std::cout << "]" << std::endl;
}

// 功能：拒绝逆解结果超出 Robot Assist 给出的关节软限位。
void validateSoftLimits(const std::array<double, 6>& joints, const char* title) {
    printJoints(title, joints);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const double deg = joints[i] * 180.0 / kPi;
        if (!std::isfinite(deg) || deg < kSoftMinDeg[i] || deg > kSoftMaxDeg[i]) {
            std::ostringstream message;
            message << title << ": J" << (i + 1) << "=" << deg
                    << " deg, allowed=[" << kSoftMinDeg[i] << ","
                    << kSoftMaxDeg[i] << "] deg";
            throw std::runtime_error(message.str());
        }
    }
}

// 功能：轮询等待非实时 MoveJ 完成，并监控掉电、急停和控制器错误。
void waitForMotion(XMateRobot& robot, const char* name) {
    for (int i = 0; i < kMotionWaitLoops; ++i) {
        std::this_thread::sleep_for(kPollPeriod);

        std::error_code code;
        const PowerState power = robot.powerState(code);
        checkError(code, "读取电源状态");
        if (power != PowerState::on) {
            throw std::runtime_error(std::string(name) +
                                     ": 运动期间伺服不再处于上电状态");
        }

        const OperationState state = robot.operationState(code);
        checkError(code, "读取运行状态");
        if (state == OperationState::idle || state == OperationState::unknown) {
            checkError(robot.lastErrorCode(), "运动执行结果");
            return;
        }
    }

    std::error_code stop_code;
    robot.stop(stop_code);
    throw std::runtime_error(std::string(name) + ": 等待运动完成超时");
}

// 功能：把机器人整理到自动、上电、非实时命令模式，为普通 MoveJ 做准备。
void prepareForNrtMove(XMateRobot& robot) {
    std::error_code code;

    robot.stop(code);
    warnError(code, "stop / 停止残留运动");
    code.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const OperationState state = robot.operationState(code);
    checkError(code, "读取准备前运行状态");
    std::cout << "[准备 Prepare] operationState="
              << operationStateName(state) << std::endl;
    if (state == OperationState::drag) {
        robot.disableDrag(code);
        checkError(code, "disableDrag / 退出拖动模式");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    robot.setPowerState(false, code);
    warnError(code, "setPowerState(false) / 请求下电");
    code.clear();

    robot.clearServoAlarm(code);
    warnError(code, "clearServoAlarm / 清除报警");
    code.clear();

    robot.setMotionControlMode(MotionControlMode::NrtCommand, code);
    checkError(code, "setMotionControlMode(NrtCommand) / 切换非实时模式");

    robot.moveReset(code);
    warnError(code, "moveReset / 清空运动队列");
    code.clear();

    robot.setOperateMode(OperateMode::automatic, code);
    checkError(code, "setOperateMode(automatic) / 切换自动模式");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    robot.setPowerState(true, code);
    checkError(code, "setPowerState(true) / 伺服上电");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    robot.moveReset(code);
    checkError(code, "moveReset after power on / 上电后清空队列");
}

// 功能：读取命令行中紧跟在参数名之后的值。
std::string optionValue(int argc, char* argv[], const std::string& name) {
    for (int i = 1; i < argc; ++i) {
        if (argv[i] == name) {
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value / 参数缺少值: " + name);
            }
            return argv[i + 1];
        }
    }
    return {};
}

// 功能：判断命令行是否包含指定开关。
bool hasFlag(int argc, char* argv[], const std::string& flag) {
    for (int i = 1; i < argc; ++i) {
        if (argv[i] == flag) {
            return true;
        }
    }
    return false;
}

// 功能：解析连接地址与实时速度；local-ip 必须由操作者明确给出，禁止自动猜网卡。
Options parseOptions(int argc, char* argv[]) {
    Options options;
    options.execute = hasFlag(argc, argv, "--execute");

    const std::string robot_ip = optionValue(argc, argv, "--robot-ip");
    if (!robot_ip.empty()) {
        options.robot_ip = robot_ip;
    }
    options.local_ip = optionValue(argc, argv, "--local-ip");

    const std::string speed = optionValue(argc, argv, "--rt-speed");
    if (!speed.empty()) {
        try {
            std::size_t consumed = 0;
            options.rt_speed_factor = std::stod(speed, &consumed);
            if (consumed != speed.size()) {
                throw std::invalid_argument("trailing characters");
            }
        }
        catch (const std::exception&) {
            throw std::runtime_error("invalid --rt-speed / 实时速度系数无效: " + speed);
        }
    }

    if (!(options.rt_speed_factor > 0.0) ||
        options.rt_speed_factor > kMaxAllowedRtSpeedFactor) {
        std::ostringstream message;
        message << "--rt-speed must be in (0," << kMaxAllowedRtSpeedFactor
                << "] / 实时速度系数超出本测试允许范围";
        throw std::runtime_error(message.str());
    }
    return options;
}

// 功能：只在输入完全匹配时放行下一段真实运动。
void requireConfirmation(const char* prompt, const char* expected) {
    std::cout << prompt;
    std::string input;
    if (!std::getline(std::cin, input) || input != expected) {
        throw std::runtime_error(std::string("confirmation rejected / 未输入 ") +
                                 expected + "，未发送该段运动");
    }
}

// 功能：把实时控制器退出后的机器人恢复到非实时模式；清理阶段不抛异常。
void restoreNrtNoThrow(XMateRobot& robot) noexcept {
    std::error_code code;
    robot.setMotionControlMode(MotionControlMode::NrtCommand, code);
    if (code) {
        std::cerr << "[清理 Cleanup] 恢复 NrtCommand 失败: "
                  << describeError(code) << std::endl;
        return;
    }
    robot.moveReset(code);
    if (code) {
        std::cerr << "[清理 Cleanup] moveReset 失败: "
                  << describeError(code) << std::endl;
    }
}

// 功能：显式断开连接，避免旧版 SDK 在进程析构阶段触发容器断言。
void disconnectNoThrow(std::unique_ptr<XMateRobot>& robot) noexcept {
    if (!robot) {
        return;
    }
    std::error_code code;
    robot->disconnectFromRobot(code);
    if (code) {
        std::cerr << "[警告 Warning] disconnectFromRobot: "
                  << describeError(code) << std::endl;
    }
    robot.reset();
}

// 功能：执行“普通 MoveJ 到 A → 验证实时状态通道 → 实时平滑 MoveJ 到 B”的独立测试。
void runTest(const Options& options, std::unique_ptr<XMateRobot>& robot) {
    printPose("test point A / 测试起点 A", kPointA);
    printPose("test point B / 测试终点 B", kPointB);
    std::cout << "[速度 Speed] A 段 NRT MoveJ speed=" << kMoveToASpeed
              << ", zone=" << kFineZone
              << "; B 段 RT MoveJ speed_factor="
              << options.rt_speed_factor << std::endl;

    requireConfirmation(
        "[安全确认 Safety] 核对 A/B、急停、工作区和本机实时网卡 IP。"
        "输入 RUN_A 才连接并以普通 MoveJ 到 A: ",
        "RUN_A");

    // 构造函数第二个参数是实时 UDP 收发使用的本机网卡 IPv4；缺失/错误会导致首帧状态超时。
    robot = std::make_unique<XMateRobot>(options.robot_ip, options.local_ip);
    prepareForNrtMove(*robot);

    std::cout << "[运动 Motion] NRT MoveJ current -> A，完成后在 A 完全停车。" << std::endl;
    MoveJCommand move_to_a(CartesianPosition({
        kPointA[0], kPointA[1], kPointA[2],
        kPointA[3], kPointA[4], kPointA[5]
    }), kMoveToASpeed, kFineZone);
    std::error_code code;
    robot->executeCommand({move_to_a}, code);
    checkError(code, "execute MoveJ to A / 下发普通 MoveJ 到 A");
    waitForMotion(*robot, "MoveJ to A / 普通 MoveJ 到 A");

    const auto arrived_a_pose = robot->flangePos(code);
    checkError(code, "flangePos at A / 读取 A 位姿");
    const auto arrived_a_joints = robot->jointPos(code);
    checkError(code, "jointPos at A / 读取 A 关节角");
    printPose("arrived A flange / 到达 A 的法兰", arrived_a_pose);
    validateSoftLimits(arrived_a_joints, "arrived A joints / 到达 A 的关节角");

    // B 由 SDK 模型先做逆解，再用同一模型正解回验；实际目标仍需操作者核对六轴角。
    const auto target_b_joints =
        robot->model().calcIk(CartesianPosition({
            kPointB[0], kPointB[1], kPointB[2],
            kPointB[3], kPointB[4], kPointB[5]
        }), code);
    checkError(code, "calcIk(B) / 计算 B 的逆解");
    validateSoftLimits(target_b_joints, "IK target B joints / B 的逆解关节目标");

    const auto fk_b = robot->model().calcFk(target_b_joints, code);
    checkError(code, "calcFk(IK(B)) / B 逆解正解回验");
    printPose("FK check B / B 逆解回验位姿", fk_b.rpy.empty()
        ? kPointB
        : std::array<double, 6>{
            fk_b.trans[0], fk_b.trans[1], fk_b.trans[2],
            fk_b.rpy[0], fk_b.rpy[1], fk_b.rpy[2]
        });

    requireConfirmation(
        "[安全确认 Safety] 请在 Robot Assist 核对上面的 B 六轴角。"
        "输入 RUN_B 才验证实时状态通道并从 A 平滑 MoveJ 到 B: ",
        "RUN_B");

    // 丢包阈值必须在切换 RtCommand 之前设置。
    robot->setRtNetworkTolerance(kRtNetworkTolerancePercent, code);
    checkError(code, "setRtNetworkTolerance / 设置实时网络阈值");
    robot->setMotionControlMode(MotionControlMode::RtCommand, code);
    checkError(code, "setMotionControlMode(RtCommand) / 切换实时模式");

    auto rt = robot->getRtMotionController().lock();
    if (!rt) {
        restoreNrtNoThrow(*robot);
        throw std::runtime_error("getRtMotionController returned empty / 无法取得实时控制器");
    }

    bool receiving = false;
    bool moving = false;
    try {
        std::cout << "[网络 Network] 等待实时关节状态首帧；SDK 超时为 3 秒。" << std::endl;
        rt->startReceiveRobotState({jointPos_m});
        receiving = true;

        std::array<double, 6> rt_start_joints{};
        const int state_result = rt->getStateData(jointPos_m, rt_start_joints);
        if (state_result != 0) {
            throw std::runtime_error("getStateData(jointPos_m) failed / 读取实时关节状态失败");
        }
        validateSoftLimits(rt_start_joints, "RT start joints / 实时通道起始关节角");

        double max_start_difference_deg = 0.0;
        for (std::size_t i = 0; i < rt_start_joints.size(); ++i) {
            max_start_difference_deg = std::max(
                max_start_difference_deg,
                std::abs(rt_start_joints[i] - arrived_a_joints[i]) * 180.0 / kPi);
        }
        std::cout << "[网络 Network] 实时状态首帧接收成功，和 A 到位读数最大差="
                  << max_start_difference_deg << " deg" << std::endl;
        if (max_start_difference_deg > kRtStartJointToleranceDeg) {
            throw std::runtime_error(
                "RT start state does not match A / 实时首帧关节状态与 A 到位状态不一致");
        }

        std::cout << "[运动 Motion] RT MoveJ A -> B，单段规划、起终速度为 0，"
                  << "speed_factor=" << options.rt_speed_factor << std::endl;
        moving = true;
        rt->MoveJ(options.rt_speed_factor, rt_start_joints, target_b_joints);
        moving = false;

        rt->stopReceiveRobotState();
        receiving = false;
        rt.reset();
        restoreNrtNoThrow(*robot);
    }
    catch (...) {
        if (moving) {
            try {
                rt->stopMove();
            }
            catch (...) {
            }
        }
        if (receiving) {
            try {
                rt->stopReceiveRobotState();
            }
            catch (...) {
            }
        }
        rt.reset();
        restoreNrtNoThrow(*robot);
        throw;
    }

    const auto arrived_b_joints = robot->jointPos(code);
    checkError(code, "jointPos after B / 读取 B 到位关节角");
    const auto arrived_b_pose = robot->flangePos(code);
    checkError(code, "flangePos after B / 读取 B 到位位姿");
    printJoints("arrived B joints / B 到位关节角", arrived_b_joints);
    printPose("arrived B flange / B 到位法兰", arrived_b_pose);

    for (std::size_t i = 0; i < arrived_b_joints.size(); ++i) {
        const double error_deg =
            std::abs(arrived_b_joints[i] - target_b_joints[i]) * 180.0 / kPi;
        if (error_deg > kArrivalJointToleranceDeg) {
            std::ostringstream message;
            message << "B arrival error / B 到位误差: J" << (i + 1)
                    << " error=" << error_deg << " deg";
            throw std::runtime_error(message.str());
        }
    }
    std::cout << "[完成 Done] A -> B 独立平滑运动测试正常完成。" << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
    setConsoleUtf8();
    std::unique_ptr<XMateRobot> robot;

    try {
        const Options options = parseOptions(argc, argv);
        if (!options.execute) {
            std::cout
                << "默认不连接或移动机器人。\n"
                << "用法: smooth_move_test.exe --execute --local-ip <本机机械臂网卡IPv4> "
                   "[--robot-ip 192.168.2.160] [--rt-speed 0.05]\n"
                << "说明: --local-ip 不是机器人 IP，必须是电脑连接机械臂的那块网卡地址。"
                << std::endl;
            return 0;
        }
        if (options.local_ip.empty()) {
            throw std::runtime_error(
                "--local-ip is required / 实时测试必须明确指定本机机械臂网卡 IPv4");
        }

        std::cout << "[连接参数 Network] robot-ip=" << options.robot_ip
                  << ", local-ip=" << options.local_ip << std::endl;
        runTest(options, robot);
        disconnectNoThrow(robot);
        return 0;
    }
    catch (const rokae::Exception& error) {
        if (robot) {
            std::error_code stop_code;
            robot->stop(stop_code);
        }
        std::cerr << "[Rokae Error / 机器人错误] "
                  << sdkMessageToUtf8(error.what()) << std::endl;
        disconnectNoThrow(robot);
        return 1;
    }
    catch (const std::exception& error) {
        if (robot) {
            std::error_code stop_code;
            robot->stop(stop_code);
        }
        std::cerr << "[Error / 错误] "
                  << sdkMessageToUtf8(error.what()) << std::endl;
        disconnectNoThrow(robot);
        return 1;
    }
}
