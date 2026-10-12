#include <iostream>
#include <thread>
#include "rokae/robot.h"

using namespace std;
using namespace rokae;

void WaitRobot(BaseRobot* robot);

int main() {
    try {
        std::string ip = "192.168.0.160";
        std::error_code ec;
        XMateRobot robot(ip);

        robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
        robot.setOperateMode(OperateMode::automatic, ec);
        robot.setPowerState(true, ec);
        robot.moveReset(ec);
        robot.setDefaultZone(50, ec);
        robot.setDefaultSpeed(500, ec);

        std::cout << "Robot moving to initial position..." << std::endl;
        rokae::MoveAbsJCommand absjZero({0, M_PI/6, M_PI/3, 0, M_PI/2, 0, 0}, 200, 50);
        robot.executeCommand({absjZero}, ec);
        WaitRobot(&robot);
        std::cout << "Arrived at initial position" << std::endl;

        auto currentPose = robot.flangePos(ec);
        std::cout << "Current flange position: ";
        for (size_t i = 0; i < currentPose.size(); ++i) {
            std::cout << currentPose[i] << (i < currentPose.size() - 1 ? ", " : "\n");
        }

        double targetX = currentPose[0] - 0.1;
        double targetY = currentPose[1];
        double targetZ = currentPose[2];
        double targetRx = currentPose[3];
        double targetRy = currentPose[4];
        double targetRz = currentPose[5];

        std::cout << "Target position (move left 10cm): " 
                  << "X=" << targetX << ", Y=" << targetY << ", Z=" << targetZ 
                  << ", Rx=" << targetRx << ", Ry=" << targetRy << ", Rz=" << targetRz << std::endl;

        std::cout << "Moving left 10cm..." << std::endl;
        rokae::MoveLCommand moveLeft({targetX, targetY, targetZ, targetRx, targetRy, targetRz}, 200, 50);
        robot.executeCommand({moveLeft}, ec);
        WaitRobot(&robot);
        std::cout << "Move completed" << std::endl;

        robot.setPowerState(false, ec);
        robot.disconnectFromRobot(ec);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
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