#pragma warning(disable: 4819)
#pragma warning(disable: 4996)
#pragma warning(disable: 26495)
#pragma warning(disable: 26451)

#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <winsock2.h>
#include "rokae/robot.h"

#pragma comment(lib,"ws2_32.lib")

using namespace std;
using namespace rokae;
using namespace Eigen;

const double PI = 3.14159265358979323846;
std::string robot_ip = "192.168.2.160";
std::error_code ec;
XMateRobot robot(robot_ip);
Eigen::VectorXd object_coord(4);

const double OBS_X = -365.484 / 1000.0;
const double OBS_Y = 415.499 / 1000.0;
const double OBS_Z = 347.775 / 1000.0;
const double OBS_RX = -179.059 * PI / 180.0;
const double OBS_RY = -7.694 * PI / 180.0;
const double OBS_RZ = -55.500 * PI / 180.0;

union data_t {
    char buffer[12];
    struct { float x; float y; float z; } point;
};

Matrix4d eulerToMatrix(Vector3d euler, Vector3d pos) {
    Matrix3d r;
    r = AngleAxisd(euler.z(), Vector3d::UnitZ()) *
        AngleAxisd(euler.y(), Vector3d::UnitY()) *
        AngleAxisd(euler.x(), Vector3d::UnitX());
    Matrix4d RT = Matrix4d::Identity();
    RT.block<3, 3>(0, 0) = r;
    RT.block<3, 1>(0, 3) = pos;
    return RT;
}

Matrix4d hand_in_eye() {
    Matrix4d m;
    m << 0.49540362, 0.86814727, -0.02992614, -111.62044483,
        -0.86835787, 0.49584859, 0.00942212, 6.83338616,
        0.02301862, 0.02131884, 0.99950770, -113.81849505,
        0.00000000, 0.00000000, 0.00000000, 1.00000000;
    return m;
}

void WaitRobot(BaseRobot* r) {
    bool running = true;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::error_code ec_wait;
        auto st = r->operationState(ec_wait);
        if (st == OperationState::idle || st == OperationState::unknown) running = false;
    }
}

void connect_computer() {
    robot.clearServoAlarm(ec);
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);
    robot.disconnectFromRobot(ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    robot.connectToRobot(ec);
    robot.moveReset(ec);
    robot.setDefaultZone(50, ec);
    robot.setDefaultSpeed(500, ec);
    std::cout << ">>> Robot Connected." << std::endl;
}

void pick() {
    std::cout << ">>> Executing Pick Sequence..." << std::endl;

    auto Pos = robot.flangePos(ec);
    if (Pos.size() < 6) {
        std::cerr << "!!! Error: Empty pose data. Skipping." << std::endl;
        return;
    }

    Matrix4d end_to_base;
    end_to_base << 0.5612339, -0.82281984, 0.08935968000000001, -365.484,
        -0.81675264, -0.5680729600000001, -0.10105584, 415.499,
        0.13391328, -0.01626864, -0.9908621, 347.775,
        0, 0, 0, 1;

    Vector4d p_base = end_to_base * hand_in_eye() * object_coord;
    std::cout << ">>> Target Base: X=" << p_base(0) << " Y=" << p_base(1) << " Z=" << p_base(2) << std::endl;

    double gripper_len = 135.0;
    double hover_dist = 60.0;

    double tx = p_base(0) / 1000.0;
    double ty = p_base(1) / 1000.0;
    double tz_hover = (p_base(2) + gripper_len + hover_dist) / 1000.0;
    double tz_pick = (p_base(2) + gripper_len) / 1000.0;

    double rx = 177.349 * PI / 180.0;
    double ry = 2.694 * PI / 180.0;
    double rz = -129.738 * PI / 180.0;

    rokae::MoveJCommand hover_cmd({ tx, ty, tz_hover, rx, ry, rz }, 200, 50);
    robot.executeCommand({ hover_cmd }, ec);
    WaitRobot(&robot);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    rokae::MoveLCommand dive_cmd({ tx, ty, tz_pick, rx, ry, rz }, 100, 20);
    robot.executeCommand({ dive_cmd }, ec);
    WaitRobot(&robot);

    robot.setDO(0, 0, true, ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    robot.setDO(0, 0, false, ec);

    robot.executeCommand({ hover_cmd }, ec);
    WaitRobot(&robot);
    std::cout << ">>> Gripper Lifted." << std::endl;

    rokae::MoveJCommand obs_cmd({ OBS_X, OBS_Y, OBS_Z, OBS_RX, OBS_RY, OBS_RZ }, 200, 50);
    robot.executeCommand({ obs_cmd }, ec);
    WaitRobot(&robot);

    std::cout << ">>> Pick Finished. Returned to Observation Point." << std::endl;
}

void socket_client() {
    WORD w_req = MAKEWORD(2, 2);
    WSADATA wsadata;
    WSAStartup(w_req, &wsadata);

    SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
    SOCKADDR_IN addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.S_un.S_addr = inet_addr("192.168.2.100");
    addr.sin_port = htons(5010);

    std::cout << ">>> Connecting to Python..." << std::endl;
    if (connect(s, (SOCKADDR*)&addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
        std::cout << "!!! Connection Failed." << std::endl;
        WSACleanup();
        return;
    }

    while (true) {
        data_t dd = { 0 };
        int len = recv(s, dd.buffer, 12, 0);
        if (len == 12) {
            object_coord << dd.point.x, dd.point.y, dd.point.z, 1.0;
            pick();
        }
        else if (len <= 0) break;
    }
    closesocket(s);
    WSACleanup();
}

int main() {
    try {
        connect_computer();

        rokae::MoveJCommand obs_pt({ OBS_X, OBS_Y, OBS_Z, OBS_RX, OBS_RY, OBS_RZ }, 200, 50);
        robot.executeCommand({ obs_pt }, ec);
        WaitRobot(&robot);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        socket_client();

    }
    catch (const rokae::Exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    return 0;
}