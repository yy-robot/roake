/*
 * test5.cpp
 * 夹爪控制示例程序 - 实现夹爪的开启与闭合功能
 * Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "rokae/robot.h"

using namespace rokae;
using namespace std;

/*
 * 等待机器人运动完成
 * @param robot 机器人指针
 */
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

/*
 * 夹爪控制器类
 * 用于控制机器人末端夹爪的开启与闭合
 */
class GripperController {
private:
    BaseRobot* robot;
    int openPin;
    int closePin;
    bool isOpen;

public:
    /*
     * 构造函数
     * @param robot 机器人指针
     * @param open_pin 夹爪打开的数字输出端口号
     * @param close_pin 夹爪关闭的数字输出端口号
     */
    GripperController(BaseRobot* robot, int open_pin = 0, int close_pin = 1) {
        this->robot = robot;
        this->openPin = open_pin;
        this->closePin = close_pin;
        this->isOpen = false;
    }

    /*
     * 打开夹爪
     * @param ec 错误码（输出参数）
     * @return 是否成功打开
     */
    bool openGripper(error_code& ec) {
        try {
            // 设置输出端口，打开夹爪
            robot->setDO(0, openPin, true, ec);
            robot->setDO(0, closePin, false, ec);
            
            // 等待夹爪动作完成
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            isOpen = true;
            std::cout << "夹爪已成功打开" << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "打开夹爪时发生错误: " << e.what() << std::endl;
            return false;
        }
    }

    /*
     * 关闭夹爪
     * @param ec 错误码（输出参数）
     * @return 是否成功关闭
     */
    bool closeGripper(error_code& ec) {
        try {
            // 设置输出端口，关闭夹爪
            robot->setDO(0, openPin, false, ec);
            robot->setDO(0, closePin, true, ec);
            
            // 等待夹爪动作完成
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            isOpen = false;
            std::cout << "夹爪已成功闭合" << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "关闭夹爪时发生错误: " << e.what() << std::endl;
            return false;
        }
    }

    /*
     * 获取夹爪当前状态
     * @return true表示夹爪打开，false表示夹爪闭合
     */
    bool getGripperState() const {
        return isOpen;
    }

    /*
     * 切换夹爪状态（打开变闭合，闭合变打开）
     * @param ec 错误码（输出参数）
     * @return 是否成功切换
     */
    bool toggleGripper(error_code& ec) {
        if (isOpen) {
            return closeGripper(ec);
        } else {
            return openGripper(ec);
        }
    }
};

int main() {
    try {
        std::cout << "========== 夹爪控制示例程序 ==========\n";
        std::cout << "此程序演示如何控制机器人末端夹爪的开启与闭合功能\n";
        std::cout << "注意：在真实设备上运行前，请确保已正确连接机器人并配置了正确的IO端口\n" << std::endl;

        // 机器人IP地址
        std::string ip = "192.168.0.160";
        std::error_code ec;

        // 创建机器人对象（根据实际机器人类型选择合适的构造函数）
        XMateRobot robot(ip); // 连接xMate6轴机型
        // 其他机型示例：
        // XMateErProRobot robot(ip); // 连接xMatePro机型
        // StandardRobot robot(ip); // 连接工业6轴机型

        // 设置操作模式和运动控制模式
        robot.setOperateMode(OperateMode::automatic, ec);
        robot.setPowerState(true, ec); // 上电
        robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);

        // 重置运动缓存
        robot.moveReset(ec);

        // 创建夹爪控制器实例
        GripperController gripper(&robot);

        // 演示夹爪控制功能
        std::cout << "1. 打开夹爪..." << std::endl;
        gripper.openGripper(ec);
        
        std::cout << "\n2. 等待2秒..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n3. 关闭夹爪..." << std::endl;
        gripper.closeGripper(ec);
        
        std::cout << "\n4. 等待2秒..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n5. 切换夹爪状态（再次打开）..." << std::endl;
        gripper.toggleGripper(ec);

        // 演示完成，下电并断开连接
        std::cout << "\n演示完成，正在下电并断开连接..." << std::endl;
        robot.setPowerState(false, ec); // 下电
        robot.disconnectFromRobot(ec); // 断开连接

        std::cout << "\n程序执行完毕！" << std::endl;

    } catch (const rokae::Exception& e) {
        std::cerr << "发生ROKAE异常: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "发生标准异常: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "发生未知异常" << std::endl;
        return 1;
    }

    return 0;
}