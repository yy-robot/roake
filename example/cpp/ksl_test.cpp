#define _WIN32_WINNT 0x0501
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <iostream>
#include <array>
#include <thread>
#include <chrono>
#include <cstring>
#include <cmath>
#include <winsock2.h>
#include <ws2tcpip.h>
#include "rokae/robot.h"
#include <Eigen/Dense>

#pragma comment(lib,"ws2_32.lib")

using namespace rokae;
using namespace Eigen;
using namespace std::chrono;

// ===================== ���ò��� =====================
#define ROBOT_SPEED 30           // �������˶��ٶȣ�mm/s�����ݰ�ȫ���������
#define ROBOT_ZONE 100           // ƽ���˶��Ļ�ϰ뾶��Blending radius��
#define GRAB_OFFSET_Z 50         // Z��ץȡƫ������mm�����µ�����
#define TCP_PORT 5010            // TCPͨ�Ŷ˿�
#define MM_TO_M 0.001            // ��λת��ϵ��������ת�ף�
#define WAIT_TIMEOUT_MS 15000    // �˶���ʱʱ�䣨���룩
#define ROBOT_IP "192.168.0.160" // ������IP��ַ
#define ROBOT_MODEL "xmate6"     // �������ͺ�

Eigen::Matrix3d DIR_TRANS_MATRIX;

// ===================== �������� =====================
// �ȴ��������˶���ɣ�����ʱ�ʹ�������
void WaitRobot(BaseRobot* robot, int timeout_ms = WAIT_TIMEOUT_MS) {
    auto start = high_resolution_clock::now();
    bool running = true;

    while (running) {
        auto elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start).count();
        if (elapsed > timeout_ms) {
            std::cout << "[����] �������˶���ʱ: " << elapsed << "ms" << std::endl;
            break;
        }

        std::this_thread::sleep_for(milliseconds(100));
        std::error_code ec;
        auto op_state = robot->operationState(ec);

        if (ec) {
            std::cout << "[����] ��ȡ����״̬ʧ��: " << ec.message() << std::endl;
            break;
        }

        if (op_state == OperationState::idle || op_state == OperationState::unknown) {
            running = false;
        }
    }
}

// ������Դ���ر�Socket��Winsock��
void cleanup(SOCKET client_fd, SOCKET server_fd) {
    if (client_fd != INVALID_SOCKET) {
        closesocket(client_fd);
    }
    if (server_fd != INVALID_SOCKET) {
        closesocket(server_fd);
    }
    WSACleanup();
    std::cout << "[��Ϣ] ��Դ���ͷ�" << std::endl;
}

// ��ӡ�ؽ���λ��Ϣ����ǰSDK�汾��֧��getJointLimit������
void print_joint_limit(XMateRobot& robot) {
    std::cout << "[��Ϣ] ��ǰSDK�汾��֧��getJointLimit����" << std::endl;
    return;
}

// ===================== �����ܺ��� =====================
void recv_and_move() {
    WSADATA wsadata;
    if (WSAStartup(MAKEWORD(2, 2), &wsadata) != 0) {
        std::cout << "[����] Winsock��ʼ��ʧ��: " << WSAGetLastError() << std::endl;
        return;
    }

    std::error_code ec;
    XMateRobot robot(ROBOT_IP);
    if (ec) {
        std::cout << "[����] ����������ʧ��: " << ec.message() << std::endl;
        WSACleanup();
        return;
    }
    std::cout << "[��Ϣ] ���������ӳɹ�" << std::endl;

    print_joint_limit(robot);

    // ����ŷ�����
    robot.clearServoAlarm(ec);
    if (ec) {
        std::cout << "[����] ����ŷ�����ʧ��: " << ec.message() << std::endl;
    }
    else {
        std::cout << "[��Ϣ] �ŷ����������" << std::endl;
    }

    // �����ŷ���Դ
    robot.setPowerState(true, ec);
    if (ec) {
        std::cout << "[����] �ŷ�ʹ��ʧ��: " << ec.message() << std::endl;
        WSACleanup();
        return;
    }
    std::cout << "[��Ϣ] �ŷ���ʹ��" << std::endl;

    // �����Զ�ģʽ
    robot.setOperateMode(OperateMode::automatic, ec);
    if (ec) {
        std::cout << "[����] �����Զ�ģʽʧ��: " << ec.message() << std::endl;
        WSACleanup();
        return;
    }

    // ����Ĭ���˶��ٶ�
    robot.setDefaultSpeed(ROBOT_SPEED, ec);
    if (ec) {
        std::cout << "[����] ����Ĭ���ٶ�ʧ��: " << ec.message() << std::endl;
        WSACleanup();
        return;
    }

    // ����Ĭ�ϻ�ϰ뾶��ƽ�����ɣ�
    robot.setDefaultZone(ROBOT_ZONE, ec);
    if (ec) {
        std::cout << "[����] ����Ĭ�ϻ�ϰ뾶ʧ��: " << ec.message() << std::endl;
        WSACleanup();
        return;
    }

    // ����TCP�������׽���
    SOCKET server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == INVALID_SOCKET) {
        std::cout << "[����] �����׽���ʧ��: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return;
    }

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr)); // ��ʼ���Ĵ�Ϣ��
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY; // ������������
    addr.sin_port = htons(TCP_PORT);   // ת��Ϊ�����ֽ���

    // ���׽��ֵ��˿�
    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cout << "[����] �˿ڰ�ʧ��: " << WSAGetLastError() << std::endl;
        cleanup(INVALID_SOCKET, server_fd);
        return;
    }

    // ��ʼ�����ͻ�������
    if (listen(server_fd, 1) == SOCKET_ERROR) {
        std::cout << "[����] ����ʧ��: " << WSAGetLastError() << std::endl;
        cleanup(INVALID_SOCKET, server_fd);
        return;
    }
    std::cout << "[��Ϣ] TCP�����������˿�: " << TCP_PORT << "���ȴ��ͻ�������..." << std::endl;

    // ���ܿͻ�������
    SOCKET client_fd = accept(server_fd, nullptr, nullptr);
    if (client_fd == INVALID_SOCKET) {
        std::cout << "[����] ��������ʧ��: " << WSAGetLastError() << std::endl;
        cleanup(INVALID_SOCKET, server_fd);
        return;
    }
    std::cout << "[��Ϣ] �ͻ���������" << std::endl;

    // ���տͻ������ݣ�12�ֽڣ�3��float���ֱ��ӦX/Y/Z��
    char buf[12];
    int recv_len = recv(client_fd, buf, sizeof(buf), 0);
    if (recv_len == SOCKET_ERROR) {
        std::cout << "[����] ��������ʧ��: " << WSAGetLastError() << std::endl;
        cleanup(client_fd, server_fd);
        return;
    }
    else if (recv_len == 0) {
        std::cout << "[��Ϣ] �ͻ����ѶϿ�����" << std::endl;
        cleanup(client_fd, server_fd);
        return;
    }
    else if (recv_len != sizeof(buf)) {
        std::cout << "[����] ���ݳ��Ȳ�ƥ��: ����" << recv_len << "�ֽڣ�����12�ֽ�" << std::endl;
        cleanup(client_fd, server_fd);
        return;
    }

    // �������յ��Ļ�е�����꣨mm��
    float arm_x_mm, arm_y_mm, arm_z_mm;
    memcpy(&arm_x_mm, buf, 4);
    memcpy(&arm_y_mm, buf + 4, 4);
    memcpy(&arm_z_mm, buf + 8, 4);

    // ��λת��������ת�ף�������Z��ץȡƫ��
    double arm_x = arm_x_mm * MM_TO_M;
    double arm_y = arm_y_mm * MM_TO_M;
    double arm_z = (arm_z_mm + GRAB_OFFSET_Z) * MM_TO_M;

    // ��ӡ�������Ŀ��λ��
    std::cout << "\n[��Ϣ] ������Ŀ��λ��: " << std::endl;
    std::cout << "  X: " << arm_x << "m (" << arm_x_mm << "mm)" << std::endl;
    std::cout << "  Y: " << arm_y << "m (" << arm_y_mm << "mm)" << std::endl;
    std::cout << "  Z: " << arm_z << "m (" << arm_z_mm + GRAB_OFFSET_Z << "mm)" << std::endl;

    // ��ȡ��ǰĩ��ִ������������λ��
    std::array<double, 6> current_pose = robot.flangePos(ec);
    if (ec) {
        std::cout << "[����] ��ȡ��ǰλ��ʧ��: " << ec.message() << std::endl;
        cleanup(client_fd, server_fd);
        return;
    }

    // ����Ŀ��λ�ˣ���ǰλ�� + ƫ����
    std::array<double, 6> target_pose = current_pose;
    target_pose[0] += arm_x; // X�᣺��ǰλ�� + ƫ��
    target_pose[1] += arm_y; // Y�᣺��ǰλ�� + ƫ��
    target_pose[2] = arm_z;  // Z�᣺ֱ�Ӹ�ֵ����ץȡƫ�ƣ�

    // ִ�йؽڿռ��˶����MoveJ��
    rokae::MoveJCommand move_cmd(target_pose, ROBOT_SPEED, ROBOT_ZONE);
    robot.executeCommand({ move_cmd }, ec);
    if (ec) {
        std::cout << "[����] MoveJ����ִ��ʧ��: " << ec.message() << std::endl;
        cleanup(client_fd, server_fd);
        return;
    }
    std::cout << "[��Ϣ] MoveJ����ͳɹ�" << std::endl;

    // �ȴ��˶����
    WaitRobot(&robot);

    // �ر����Ӳ��ͷ���Դ
    cleanup(client_fd, server_fd);
}

// ===================== ������� =====================
int main() {
    try {
        std::cout << "====================================" << std::endl;
        std::cout << "=    ��ʯ������TCP������ʾ����      =" << std::endl;
        std::cout << "====================================" << std::endl;

        // ��ʼ������任���󣨻�е�ۡ������
        DIR_TRANS_MATRIX << 0, 0, 1,
            -1, 0, 0,
            0, -1, 0;

        recv_and_move();
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[����] �����쳣: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "[����] ����δ֪�쳣" << std::endl;
        return 1;
    }
}