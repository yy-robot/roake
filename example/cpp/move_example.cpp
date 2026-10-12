/**
 * @file move_example.cpp.
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include<winsock2.h>

using namespace std;
using namespace rokae;
using namespace Eigen;


std::string ip = "192.168.2.160";
std::error_code ec;
XMateRobot robot(ip); // 此处连接的是xMate3机型
void WaitRobot(BaseRobot* robot);


void initialization();


void connect_computer() {

    auto clearServoAlarm();
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    //2.断开重连，防止异常
    robot.disconnectFromRobot(ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    robot.connectToRobot(ec);

    robot.moveReset(ec);
    robot.setDefaultZone(50, ec); //设置默认转弯区
    robot.setDefaultSpeed(500, ec); //设置默认速度
}

int main() {
    

        connect_computer();

  
        rokae::MoveJCommand tcp({ 484.918/1000,  -18.155/1000,  343.126/1000,  -170.453 * M_PI / 180 ,  22.581 * M_PI / 180 ,  -166.606 * M_PI / 180 }, 50, 50);
        robot.executeCommand({ tcp }, ec);
        //rokae::MoveAbsJCommand absjZero({0, M_PI/6, M_PI/3 , 0, M_PI/2 , 0, 0});
        //robot.executeCommand({absjZero}, ec); //机器人返回零点位置

        //// *** 4. 走螺旋线轨迹 *** 
        //std::vector<double> posture = {0.56067, 0.0, 0.402345};   //0.56297, 0.0, 0.247345
        //rokae::MoveLCommand cmdl0({posture[0], posture[1], posture[2], M_PI, 0, M_PI}, 500, 100);
        //robot.executeCommand({cmdl0}, ec); //运动到初始位置
        //auto getflangePos = robot.flangePos(ec);
        //        std::cout << "当前#位姿为：" << getflangePos << std::endl;
        ////return 0;
        //Sleep(500);
        //std::cout << "等待机器人运动，回车键结束等待" << std::endl;
        //while (std::getchar() != '\n');

      //  double r = 0.005;
      //  for (int i = 1; i <= 3; i++) {
      //    //前半圈
      //    MoveCCommand cir0({posture[0] + 2 * i * r, posture[1], posture[2], M_PI, 0, M_PI},
      //                      {posture[0] + r, posture[1] + (2 * i - 1) * r, posture[2], M_PI, 0, M_PI});

      //    //后半圈
      //    MoveCCommand cir1({posture[0] - 2 * i * r, posture[1], posture[2], M_PI, 0, M_PI},
      //                      {posture[0], posture[1] - 2 * i * r, posture[2], M_PI, 0, M_PI});
      //    robot.executeCommand({cir0, cir1}, ec);
      //  }

      //  std::cout << "等待机器人运动，回车键结束等待" << std::endl;
      //  while (std::getchar() != '\n');

      //  // *** 5. 走方形轨迹 ***
      //  robot.executeCommand({ cmdl0 }, ec); //机器人返回零点位置
      //  std::cout << "等待机器人运动，回车键结束等待" << std::endl;
      //  while (std::getchar() != '\n');

      //  //std::vector<double> posture1 = { 0.52067, 0.0, 0.247345 };   //0.45, 0.3, 0.45

      //  rokae::MoveLCommand cmdj0({posture[0], posture[1], posture[2], M_PI, 0, M_PI}, 500, 100);
      //  rokae::MoveLCommand cmdj1({posture[0] - 0.02, posture[1], posture[2], M_PI, 0, M_PI}, 500, 100);
      //  rokae::MoveLCommand cmdj2({posture[0] - 0.02, posture[1] + 0.02, posture[2], M_PI, 0, M_PI}, 500, 100);
      //  rokae::MoveLCommand cmdj3({posture[0], posture[1] + 0.02, posture[2], M_PI, 0, M_PI}, 500, 100);
      //  robot.executeCommand({cmdj0, cmdj1, cmdj2, cmdj3, cmdj0}, ec);

      //  std::cout << "等待机器人运动，回车键结束等待" << std::endl;
      //  while (std::getchar() != '\n');
      //  robot.setPowerState(false, ec);
      //  robot.disconnectFromRobot(ec);
      //} catch (const std::exception &e) {
      //  print(std::cerr, e.what());
      //}
        return 0;
    
}




//#include <iostream>
//#include <cmath>
//#include "rokae/robot.h"
//#include "print_helper.hpp"
//#include<winsock.h>
//
//int main() {
//    try {
//        using namespace rokae;
//
//        // *** 1. 连接机器人 ***
//        std::string ip = "192.168.0.160";
//        std::error_code ec;
//        rokae::XMateRobot robot(ip); // ****   XMate 7-axis
//
//        // *** 2. 切换到自动模式并上电 ***
//        robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
//        robot.setOperateMode(OperateMode::automatic, ec);
//        robot.setPowerState(true, ec);
//
//        // *** 3. 运动指令 ***
//        robot.moveReset(ec);
//        robot.setDefaultZone(50, ec); //设置默认转弯区
//        robot.setDefaultSpeed(500, ec); //设置默认速度
//
//        rokae::MoveAbsJCommand absjZero({ 0, M_PI / 6 , M_PI / 3 , 0, M_PI / 2 , 0, 0 },200,50);
//        robot.executeCommand({absjZero}, ec); //机器人返回零点位置
//
//
//        // *** 4. 走螺旋线轨迹 ***
//        /*auto pos = robot.flangePos(ec);
//        for (int i = 0; i < pos.size(); i++) {
//            std::cout << pos[i] << "///";
//
//        }*/
//        //
//        std::vector<double> posture = { 0.563, 0.0, 0.432 };
        //std::vector<double> posture = { 5.6297, 0.0, 2.47345 };                                             //562.97, 0.0, 247.345     0.4, 0.2, 0.5
        //rokae::MoveJCommand cmdl0({ posture[0], posture[1] , posture[2], M_PI, 0, M_PI }, 500, 100);
//        robot.executeCommand({ cmdl0 }, ec); //运动到初始位置
//        auto getflangePos = robot.flangePos(ec);
//        std::cout << "当前#位姿为：" << getflangePos << std::endl;
//        //return 0;
//        Sleep(500);
//
//        std::cout << "等待机器人运动，回车键结束等待" << std::endl;
//        while (std::getchar() != '\n');
//
//        double r = 0.005;
//        for (int i = 1; i <= 3; i++) {
//            //前半圈
//            MoveCCommand cir0({ posture[0] + 2 * i * r, posture[1], posture[2], M_PI, 0, M_PI },
//                { posture[0] + r, posture[1] + (2 * i - 1) * r, posture[2], M_PI, 0, M_PI });
//
//            //后半圈
//            MoveCCommand cir1({ posture[0] - 2 * i * r, posture[1], posture[2], M_PI, 0, M_PI },
//                { posture[0], posture[1] - 2 * i * r, posture[2], M_PI, 0, M_PI });
//            robot.executeCommand({ cir0, cir1 }, ec);
//        }
//
//        std::cout << "等待机器人运动，回车键结束等待" << std::endl;
//        while (std::getchar() != '\n');
//
//        // *** 5. 走方形轨迹 ***
//        robot.executeCommand({ posture }, ec); //机器人返回零点位置
//        std::cout << "等待机器人运动，回车键结束等待" << std::endl;
//        while (std::getchar() != '\n');
//
//        std::vector<double> posture1 = { 0.45, 0.3, 0.45 };
//
//        rokae::MoveLCommand cmdj0({ posture1[0], posture1[1], posture1[2], M_PI, 0, M_PI }, 500, 100);
//        rokae::MoveLCommand cmdj1({ posture1[0] - 0.2, posture1[1], posture1[2], M_PI, 0, M_PI }, 500, 100);
//        rokae::MoveLCommand cmdj2({ posture1[0] - 0.2, posture1[1] + 0.2, posture1[2], M_PI, 0, M_PI }, 500, 100);
//        rokae::MoveLCommand cmdj3({ posture1[0], posture1[1] + 0.2, posture1[2], M_PI, 0, M_PI }, 500, 100);
//        robot.executeCommand({ cmdj0, cmdj1, cmdj2, cmdj3, cmdj0 }, ec);
//
//        std::cout << "等待机器人运动，回车键结束等待" << std::endl;
//        while (std::getchar() != '\n');
//        robot.setPowerState(false, ec);
//        robot.disconnectFromRobot(ec);
//    }
//    catch (const std::exception& e) {
//        print(std::cerr, e.what());
//    }
//    return 0;
//}