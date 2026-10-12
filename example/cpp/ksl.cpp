#define _WIN32_WINNT 0x0501          
#define _WINSOCK_DEPRECATED_NO_WARNINGS  
#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include <string>
#include <climits>
#include <Eigen/Dense>
#include <winsock2.h> 
#include <cmath>
#include <iomanip>
#include <cstring>
#include <chrono>

#pragma comment(lib,"ws2_32.lib")

using namespace rokae;
using namespace Eigen;
using namespace std::chrono;

// ===================== 眼在手模式核心配置 =====================
const float VALID_COORD_THRESHOLD = 10.0f;  // 有效坐标阈值(mm)
const float GRAB_OFFSET_X = 0;        // X轴抓取补偿（新增：50mm）
const float GRAB_OFFSET_Y = 0;         // Y轴抓取补偿（新增：100mm）
const float GRAB_OFFSET_Z = 0;         // Z轴抓取补偿（眼在手微调）
const int ROBOT_SPEED = 200;                // 运动速度(mm/s)
const int ROBOT_ZONE = 50;                  // 混合半径(mm)

// Function declarations
void WaitRobot(BaseRobot* robot);
void connect_computer();
void move_to_point26();
void pick();
Matrix4d eulerAngleToRotatedMatrix(Vector3d& eulerAngle, Vector3d& poisition);
Matrix4d R_T2RT(Vector3d& poisition, Matrix3d rotation_matrix);
Matrix4d hand_in_eye();
void initialization();
void socket_client();

// Global variables
Eigen::Matrix4d camera_to_gripper;  // 相机→夹爪手眼矩阵（眼在手核心）
Eigen::VectorXd object(4, 1);       // 接收的坐标（齐次，mm）
std::string ip = "192.168.2.160";   // 机械臂IP
std::error_code ec;
XMateRobot robot(ip);               // xMate3机械臂

/*********************    矩阵运算（眼在手模式）    ****************************/
Matrix4d eulerAngleToRotatedMatrix(Vector3d& eulerAngle, Vector3d& poisition) {
    Matrix3d  rotX, rotY, rotZ;
    Matrix3d rotation_matrix;
    Matrix4d RT;

    auto rx = eulerAngle(0), ry = eulerAngle(1), rz = eulerAngle(2);
    auto xs = std::sin(rx), xc = std::cos(rx);
    auto ys = std::sin(ry), yc = std::cos(ry);
    auto zs = std::sin(rz), zc = std::cos(rz);
    rotX << 1, 0, 0,
        0, xc, -xs,
        0, xs, xc;
    rotY << yc, 0, ys,
        0, 1, 0,
        -ys, 0, yc;
    rotZ << zc, -zs, 0,
        zs, zc, 0,
        0, 0, 1;

    rotation_matrix = rotZ * rotY * rotX;
    RT << rotation_matrix, poisition,
        0, 0, 0, 1;
    return RT;
}

Matrix4d R_T2RT(Vector3d& poisition, Matrix3d rotation_matrix) {
    Matrix4d RT;
    RT << rotation_matrix, poisition,
        0, 0, 0, 1;
    return RT;
}

Matrix4d hand_in_eye() {
    // 眼在手模式：相机→夹爪手眼矩阵（需替换为你的标定值！）
    camera_to_gripper <<
        0.07809988, 0.99656242, 0.02763591, -125.12694,
        -0.99675085, 0.07750681, 0.02191882, -24.59342,
        0.0197015, -0.02925797, 0.99937772, 95.19177,
        0.0000001, 0.0000001, 0.0000001, 1.0000001;

    std::cout << "\n===================== 眼在手模式 - 手眼矩阵（相机→夹爪） =====================" << std::endl;
    std::cout << std::fixed << std::setprecision(6) << camera_to_gripper << std::endl;
    std::cout << "==================================================\n" << std::endl;

    return camera_to_gripper;
}

/*********************   机械臂连接与运动（眼在手模式）  ****************************/
void connect_computer() {
    // 清空报警
    robot.clearServoAlarm(ec);
    if (ec) std::cout << "Failed to clear servo alarm: " << ec.message() << std::endl;

    // 自动模式+上电
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    // 重新连接
    robot.disconnectFromRobot(ec);
    std::this_thread::sleep_for(milliseconds(1000));
    robot.connectToRobot(ec);

    if (ec) {
        std::cout << "Failed to connect to robot: " << ec.message() << std::endl;
        exit(1);
    }

    // 复位+默认参数
    robot.moveReset(ec);
    robot.setDefaultZone(ROBOT_ZONE, ec);
    robot.setDefaultSpeed(ROBOT_SPEED, ec);

    std::cout << "[OK] 机械臂连接成功（眼在手模式）！" << std::endl;
    std::cout << "操作模式：" << (robot.operateMode(ec) == OperateMode::automatic ? "自动模式" : "手动模式") << std::endl;
}

// 等待运动完成
void WaitRobot(BaseRobot* robot) {
    bool running = true;
    while (running) {
        std::this_thread::sleep_for(milliseconds(100));
        auto st = robot->operationState(ec);
        if (st == OperationState::idle || st == OperationState::unknown) {
            running = false;
        }
    }
}

// 移动到初始位姿（眼在手模式基准位）
void move_to_point26() {
    // 打开夹爪
    robot.setDO(0, 0, false, ec);
    robot.setDO(0, 1, false, ec);
    std::this_thread::sleep_for(milliseconds(500));

    // 初始位姿（米，弧度）- 与硬编码法兰→基矩阵对应
    rokae::MoveJCommand point26({
        -365.484 / 1000, 415.499 / 1000, 347.775 / 1000,
        -179.059 * M_PI / 180, -7.694 * M_PI / 180, -55.500 * M_PI / 180
        }, ROBOT_SPEED, ROBOT_ZONE);

    std::cout << "Moving to point26 (眼在手基准位)..." << std::endl;
    robot.executeCommand({ point26 }, ec);
    if (ec) {
        std::cout << "Movement command failed: " << ec.message() << std::endl;
        return;
    }

    WaitRobot(&robot);
    std::cout << "Reached point26 position (眼在手基准位)" << std::endl;
    std::this_thread::sleep_for(milliseconds(2000));
}

/*********************      抓取逻辑（适配视觉端无方向转换）      ****************************/
void pick() {
    // ========== 保留硬编码法兰→基矩阵（与基准位point26严格对应） ==========
    Matrix4d end_to_base;
    end_to_base <<
        0.5612339, -0.82281984, 0.08935968000000001, -365.484,
        -0.81675264, -0.5680729600000001, -0.10105584, 415.499,
        0.13391328, -0.01626864, -0.9908621, 347.775,
        0, 0, 0, 1;

    std::cout << "\n===================== 眼在手模式 - 法兰→基矩阵（mm） =====================" << std::endl;
    std::cout << std::fixed << std::setprecision(6) << end_to_base << std::endl;
    std::cout << "==================================================\n" << std::endl;

    // ========== 核心：手眼矩阵直接使用（不求逆！） ==========
    Matrix4d cam_to_gripper = hand_in_eye();  // 相机→夹爪矩阵（正确）

    // ========== 直接使用视觉端发送的原始相机坐标（无方向转换） ==========
    Vector3d cam_coord_raw(object(0), object(1), object(2));  // 原始相机坐标（视觉端直接发送）
    Vector4d object_aligned(
        cam_coord_raw(0),
        cam_coord_raw(1),
        cam_coord_raw(2),
        1.0  // 齐次坐标
    );
    std::cout << "[INFO] 视觉端原始相机坐标（mm）：X=" << cam_coord_raw(0)
        << ", Y=" << cam_coord_raw(1) << ", Z=" << cam_coord_raw(2) << std::endl;

    // ========== 正确坐标转换公式：基坐标 = 法兰→基矩阵 × 相机→夹爪矩阵 × 原始相机坐标 ==========
    Vector4d point_to_base = end_to_base * cam_to_gripper * object_aligned;

    // 输出转换后坐标（含补偿，调试关键）
    std::cout << "眼在手模式 - 目标基坐标（原始，mm）：X=" << point_to_base(0)
        << ", Y=" << point_to_base(1)
        << ", Z=" << point_to_base(2) << std::endl;
    std::cout << "眼在手模式 - 目标基坐标（补偿后，mm）：X=" << (point_to_base(0) + GRAB_OFFSET_X)
        << ", Y=" << (point_to_base(1) + GRAB_OFFSET_Y)
        << ", Z=" << (point_to_base(2) + GRAB_OFFSET_Z) << std::endl;

    // 无效坐标校验
    if (fabs(point_to_base(0)) < VALID_COORD_THRESHOLD ||
        fabs(point_to_base(1)) < VALID_COORD_THRESHOLD ||
        fabs(point_to_base(2)) < VALID_COORD_THRESHOLD) {
        std::cerr << "[WARNING] 无效目标坐标（眼在手模式），跳过抓取！" << std::endl;
        return;
    }

    // ========== 姿态角避免超限（使用基准位安全角度） ==========
    rokae::MoveJCommand put({
        (point_to_base(0) + GRAB_OFFSET_X) / 1000,                // X (m) + 50mm补偿
        (point_to_base(1) + GRAB_OFFSET_Y) / 1000,                // Y (m) + 100mm补偿
        (point_to_base(2) + GRAB_OFFSET_Z) / 1000,                // Z (m) + 补偿
        // 姿态角与基准位一致（避免超限）
        -174.227 * M_PI / 180, -5.622 * M_PI / 180, -27.581 * M_PI / 180
        }, ROBOT_SPEED, ROBOT_ZONE);

    // 执行抓取运动
    robot.executeCommand({ put }, ec);
    if (ec) {
        std::cerr << "[ERROR] 抓取运动指令失败（眼在手模式）：" << ec.message() << std::endl;
        return;
    }
    WaitRobot(&robot);
    std::cout << "[OK] 到达目标位置（眼在手模式）" << std::endl;
    std::this_thread::sleep_for(milliseconds(500));

    // 闭合夹爪
    robot.setDO(0, 0, true, ec);
    robot.setDO(0, 1, false, ec);
    std::this_thread::sleep_for(milliseconds(500));

    // 返回基准位
    rokae::MoveJCommand back_to_26({
        -365.484 / 1000, 415.499 / 1000, 347.775 / 1000,
        -179.059 * M_PI / 180, -7.694 * M_PI / 180, -55.500 * M_PI / 180
        }, ROBOT_SPEED, ROBOT_ZONE);
    robot.executeCommand({ back_to_26 }, ec);
    WaitRobot(&robot);
    std::this_thread::sleep_for(milliseconds(2000));
    std::cout << "[OK] 抓取完成（眼在手模式），返回基准位" << std::endl;
}

/*********************   Socket通信（眼在手模式）   ****************************/
union data_t {
    char buffer[3 * 4];
    struct {
        float x;
        float y;
        float z;
    } point;
};

void initialization() {
    WORD w_req = MAKEWORD(2, 2);
    WSADATA wsadata;
    int err = WSAStartup(w_req, &wsadata);
    if (err != 0) {
        std::cout << "Socket initialization failed!" << std::endl;
        exit(1);
    }
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
        std::cout << "Socket version incompatible!" << std::endl;
        WSACleanup();
        exit(1);
    }
}

void socket_client() {
    initialization();
    SOCKET s_server;
    SOCKADDR_IN server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.2.100");  // 视觉端IP
    server_addr.sin_port = htons(5010);                             // 视觉端端口

    s_server = socket(AF_INET, SOCK_STREAM, 0);
    if (connect(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
        std::cout << "[ERROR] 相机连接失败（眼在手模式）！Error: " << WSAGetLastError() << std::endl;
        WSACleanup();
        exit(1);
    }
    std::cout << "[OK] 相机连接成功（眼在手模式）！" << std::endl;

    // 连接后移动到基准位
    move_to_point26();

    while (1) {
        char send_buf[1024] = { 0 };
        data_t dd;
        memset(&dd, 0, sizeof(dd));

        // 发送就绪信号
        strcpy_s(send_buf, 1024, "Ready (Eye-in-Hand)");
        int send_len = send(s_server, send_buf, (int)strlen(send_buf), 0);
        if (send_len < 0) {
            std::cout << "[WARNING] 发送失败（眼在手模式）！Error: " << WSAGetLastError() << std::endl;
            std::this_thread::sleep_for(milliseconds(500));
            continue;
        }

        // 接收视觉端坐标（3个float，12字节，mm）
        int recv_len = recv(s_server, dd.buffer, 3 * 4, 0);
        if (recv_len != 3 * 4) {
            std::cout << "[WARNING] 接收无效数据（眼在手模式）！Length: " << recv_len << std::endl;
            std::this_thread::sleep_for(milliseconds(500));
            continue;
        }

        // 更新目标坐标（直接使用视觉端发送的原始相机坐标）
        object(0) = dd.point.x;
        object(1) = dd.point.y;
        object(2) = dd.point.z;
        object(3) = 1.0;
        std::cout << "\n[RECV] 眼在手模式 - 接收的相机原始坐标（mm）：X=" << object(0)
            << ", Y=" << object(1) << ", Z=" << object(2) << std::endl;

        // 执行抓取
        pick();

        // 发送完成信号
        strcpy_s(send_buf, 1024, "Completed (Eye-in-Hand)");
        send(s_server, send_buf, (int)strlen(send_buf), 0);
    }

    closesocket(s_server);
    WSACleanup();
}

/*********************      主函数（眼在手模式）      ****************************/
int main() {
    try {
        // 初始化机械臂
        connect_computer();

        // 选择模式
        std::cout << "\n===== 眼在手（Eye-in-Hand）模式 =====" << std::endl;
        std::cout << "Select mode: 1-Single Pick 2-Camera Mode" << std::endl;
        int mode;
        std::cin >> mode;

        if (mode == 1) {
            // 测试模式：固定相机原始坐标（mm，眼在手调试用）
            object(0) = 255;
            object(1) = 250;
            object(2) = 300;
            object(3) = 1.0;
            pick();
        }
        else if (mode == 2) {
            // 视觉联动模式（直接接收相机原始坐标）
            socket_client();
        }
        else {
            std::cerr << "[ERROR] 无效模式选择（眼在手模式）！" << std::endl;
        }

        // 程序结束：下电+断开
        robot.setPowerState(false, ec);
        robot.disconnectFromRobot(ec);
        std::cout << "[END] 程序结束（眼在手模式）" << std::endl;

    }
    catch (const rokae::Exception& e) {
        std::cout << "[ERROR] 机械臂异常（眼在手模式）：" << e.what() << std::endl;
        robot.setPowerState(false, ec);
        robot.disconnectFromRobot(ec);
    }
    catch (const std::exception& e) {
        std::cout << "[ERROR] 系统异常（眼在手模式）：" << e.what() << std::endl;
        robot.setPowerState(false, ec);
        robot.disconnectFromRobot(ec);
    }
    return 0;
}