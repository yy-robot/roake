/**
 * @file sdk_example.cpp
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
// * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
// * And may contains trade secrets that must be stored and viewed confidentially.
// */
//
//#include <iostream>
//#include <thread>
//#include "rokae/robot.h"
//
//using namespace rokae;
//
//void WaitRobot(BaseRobot *robot) {
//  bool running = true;
//  while (running) {
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    error_code ec;
//    auto st = robot->operationState(ec);
//    if(st == OperationState::idle || st == OperationState::unknown){
//      running = false;
//    }
//  }
//}
//
//int main() {
//  try {
//    // *** 1. 连接机器人 ***
//    std::string ip = "192.168.0.160";
//    std::error_code ec;
//    XMateErProRobot robot(ip); // 此处连接的是xMatePro机型
//
//    // 其它机型
//    // XMateRobot robot(ip); // 连接xMate6轴机型
//    // StandardRobot robot(ip); // 连接工业6轴机型
//
//    // *** 2. 查询信息 ***
//    auto robotinfo = robot.robotInfo(ec);
//    std::cout << "控制器版本号: " << robotinfo.version << ", 机型：" << robotinfo.type << std::endl;
//    std::cout << "RokaeSDK版本: " << robot.sdkVersion() << std::endl;
//    auto model = robot.model();
//
//    // *** 3. 获取机器人当前位姿，轴角度，基坐标系等信息 ***
//    auto joint_pos = robot.jointPos(ec);
//    auto joint_vel = robot.jointVel(ec);
//    auto joint_torque = robot.jointTorque(ec);
//    auto posture = robot.flangePos(ec);
//    auto base = model.baseFrame(ec);
//
//    // *** 4. 计算逆解 ***
//    auto ik = model.calcIk(posture, ec);
//
//    // *** 5. 查询IO/寄存器 ***
//    std::cout << "DO1_0当前信号值为: " << robot.getDO(1,0,ec) << std::endl;
//    // 读取单个寄存器，类型为float
//    // 假设"register0"是个寄存器数组, 长度是10
//    float val_f;
//    std::vector<float> val_af;
//    // 读第1个，即状态监控里的register0[1], 读取结果赋值给val_f
//    robot.readRegister("register0", 0, val_f, ec);
//    // 读第10个，即状态监控里的register0[10], 读取结果赋值给val_f
//    robot.readRegister("register0", 9, val_f, ec);
//    // 读整个数组，赋值给val_af, val_af的长度也变为10。此时index参数是多少都无所谓
//    robot.readRegister("register0", 9, val_af, ec);
//
//    // 读取int类型寄存器数组
//    std::vector<int> val_ai;
//    robot.readRegister("register1", 1, val_ai, ec);
//    // 写入bool/bit类型寄存器
//    robot.writeRegister("register2", 0, true, ec);
//
//    // *** 6. 断开连接再重连 ***
//    robot.disconnectFromRobot(ec);
//    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//    robot.connectToRobot(ec);
//
//    // *** 7. 打开，关闭拖动 ***
//    robot.setOperateMode(rokae::OperateMode::manual, ec);
//    robot.setPowerState(false, ec); // 自动模式下上电
//    robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
//    std::cout << "打开拖动结果: " << ec.message() << std::endl;
//    std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
//    auto power = robot.powerState(ec);
//    std::cout << "上下电状态: " << static_cast<int>(power)<< std::endl; // 打开拖动后应自动上电
//    while(getchar() != '\n');
//    robot.disableDrag(ec);
//    std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
//
//
//    // *** 8. Jog机器人 ***
//    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
//    robot.setOperateMode(rokae::OperateMode::manual, ec); // 手动模式下jog
//    std::cout << "准备Jog机器人, 需手动模式上电, 请在10秒内按住使能开关\n";
//    std::this_thread::sleep_for(std::chrono::seconds(10));
//
//    std::cout << "-- 开始Jog机器人-- \n世界坐标系下, 沿Z+方向运动50mm, 速率50%，等待机器人停止运动后按回车继续\n";
//    robot.startJog(JogOpt::world, 0.5, 50, 2, true, ec);
//    while(getchar() != '\n');
//    std::cout << "轴空间，6轴负向连续转动，速率5%，按回车停止Jog\n";
//    robot.startJog(JogOpt::jointSpace, 0.05, 5000, 5, false, ec);
//    while(getchar() != '\n'); // 按回车停止
//    robot.stop(ec); // jog结束必须调用stop()停止
//
//    // *** 9. 运动指令, 设置运动控制模式为非实时运动指令 ***
//    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
//    robot.setOperateMode(OperateMode::automatic, ec); // 切换到自动模式
//    robot.setPowerState(true, ec); // 自动模式下上电
//    robot.moveReset(ec); // 发运动指令前必须重置缓存
//
//    // MoveL
//    rokae::MoveLCommand movel0({0.563,0.0,0.432,3.1415,0,3.14159},4000,100);
//    rokae::MoveLCommand movel1({0.33467,-0.095,0.510,3.14159,-0,3.14159},4000,100);
//    robot.executeCommand({movel0, movel1}, ec); // 发送指令，机器人开始运动
//    WaitRobot(&robot);
//    std::cout << "\n上一条错误信息: " << robot.lastErrorCode().message() << std::endl;
//
//    robot.stop(ec); // 停止运动
//
//    // MoveAbsJ
//    MoveAbsJCommand abs({-0.1,0.597,0,1.129,-1.32,1.57,0});
//    robot.executeCommand({abs}, ec);
//    // MoveC
//    MoveCCommand cir({0.559,-0.0766,0.385,-2.877,-0.031,3.122},
//                     {0.549,-0.2,0.49,-2.397,-0.08745,3.085});
//    robot.executeCommand({cir}, ec);
//
//    // 客户端程序执行结束前会强制机器人停止运动。故程序需要等待机器人运动结束
//    WaitRobot(&robot);
//
//    // *** 10. 结束控制，断开连接 ***
//    robot.setPowerState(false, ec); // 机器人下电
//    robot.disconnectFromRobot(ec);
//
//  } catch (const rokae::Exception &e) {
//    std::cout << e.what();
//  }
//
//  return 0;
//}




#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include <string>
#include <climits>
#include <Eigen\Dense>
//#include <sys/types.h>          /* See NOTES */
#include<winsock2.h> 
#include "rokae/socket.h"
#include <cmath>
#include <iomanip>
#pragma comment(lib,"ws2_32.lib")


using namespace std;
using namespace rokae;
using namespace Eigen;



char parseInput(std::string& str);
void initialization();
void WaitRobot(BaseRobot* robot);
void printHelp();
void socket_client();



////全局
Eigen::Matrix4d camera_to_grap;
//
Eigen:: VectorXd object(4,1);
std::string ip = "192.168.0.160";
std::error_code ec;
XMateRobot robot(ip); // 此处连接的是xMate3机型


                                     /*********************    矩阵运算    ****************************/

//eul和pois ---- 变换矩阵
Matrix4d eulerAngleToRotatedMatrix(Vector3d& eulerAngle, Vector3d& poisition) {
    Matrix3d  rotX, rotY, rotZ;
    Matrix3d rotation_matrix;
    Matrix4d RT;
    rotation_matrix.resize(3, 3);
    rotX.resize(3, 3);
    rotY.resize(3, 3);
    rotZ.resize(3, 3);

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

    std::cout << "rotation_matrix：\n" << rotation_matrix << "  " << endl;;
    RT << rotation_matrix, poisition,
        0, 0, 0, 1;//C=A+B左右拼接
    std::cout << "变换矩阵为：\n" << RT << "  " << endl;;
    return RT;
}
//R和T转RT矩阵得到grap_to_base
Matrix4d R_T2RT( Vector3d& poisition, Matrix3d rotation_matrix)
{
    Matrix4d RT;
    RT << rotation_matrix, poisition,
           0, 0, 0, 1;//C=A+B左右拼接
    std::cout << "变换矩阵为：\n" << RT << "  " << endl;;
    return RT;
}
//手眼标定矩阵
Matrix4d hand_in_eye() {
    
    camera_to_grap <<

        0.07809988, 0.99656242, 0.02763591, -125.12694,
        -0.99675085, 0.07750681, 0.02191882, -24.59342,
        0.0197015, -0.02925797, 0.99937772, 95.19177,
        0.0000001, 0.0000001, 0.0000001, 1.0000001;

    return camera_to_grap;
}


                                  /*********************   机械臂连接  ****************************/
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


                                  /*********************      抓取      ****************************/
void pick() {
    
       
        ////相机拍照
        robot.setDO(0, 0, false, ec);    //慧灵打开
        robot.setDO(0, 1, false, ec);
       // robot.setDO(0, 0, false, ec);    //大寰25开  50力
       
        //164.065, 2.380,-173.976     177.349, 2.694,-129.738  -312.529,331.377,224.170
        rokae::MoveJCommand point26({ /*489.725 / 1000, 93.841 / 1000, 407.303 / 1000,   179.274 * M_PI / 180,  20.794 * M_PI / 180,  -177.865 * M_PI / 180*/
          /*-253.877 / 1000, -522.191 / 1000, 361.477 / 1000,   159.452 * M_PI / 180,  -1.448 * M_PI / 180,  -164.587 * M_PI / 180*/
          /* -312.529 / 1000, 331.377 / 1000, 224.170 / 1000,  177.349 * M_PI / 180,2.694 * M_PI / 180, -129.738 * M_PI / 180 */ 
            -483.905 / 1000,-18.073 / 1000,355.518 / 1000,179.929 * M_PI,6.397 * M_PI,-145.878 * M_PI
         }, 200, 50);  //0.6299,  -0.001433  ,0.5295 , 0.998 * M_PI , 0.0888 * M_PI  ,  0.994 * M_PI
        
        
        robot.executeCommand({ point26 }, ec);
        


        WaitRobot(&robot);
        
        Sleep(2000);
  
         
/* 
        VectorXd pose(7, 1);
        auto Pos = robot.flangePos(ec);   //获取法兰当前位姿XYZ,RPY
        for (int i = 0; i < Pos.size(); i++) {
            pose[i] = Pos[i];
            //std::cout << pose[i] << "  ";
        }
        //std::cout << "\n";
 */
   

        //获取夹爪到基座的变换矩阵
       // vector3d poisition = pose.head(3) * 1000;   //分离xyz与rpy      
       // cout << " 位置：" << "\n" << poisition << endl;
        //vector3d eulerangle(3, 1);
       // eulerangle = pose.segment(3, 3);
        //matrix4d end_to_base = eulerangletorotatedmatrix(eulerangle,poisition);       //rpy转旋转矩阵
       // cout << "end_to_base" << end_to_base << endl;
        
       Matrix4d end_to_base;
        end_to_base << 
            
                      //tanzhen3的point16
                     /* 0.9420039, -0.05388132, 0.33126166, 490.351,
                       -0.07925011999999999, -0.9949794000000001, 0.06355476000000002, 82.06399999999999,
                       0.32612406, -0.08611964, -0.9415530599999999, 439.042,
                       0, 0, 0, 1;*/
                        //Tool0的point16
              /*  -0.9343354400000001, -0.04181436, 0.35423964, 421.55,
                -0.03489084, 0.99905518, 0.02591298, 78.515,
                -0.35498844, 0.01184958, -0.9349116200000001, 616.375,
                0, 0, 0, 1;*/
            //newright的point16
         /*
        -0.9343354400000001, -0.04181436, 0.35423964, 495.238;
        -0.03489084, 0.99905518, 0.02591298, 71.771;
        -0.35498844, 0.01184958, -0.9349116200000001, 475.07;
        0, 0, 0, 1;
        */

        //newright -370.231,302.356,389.727,-0.0270,-0.3831,0.9226,0.0366
        /*
        -0.9343354400000001, -0.04181436, 0.35423964, 495.238;
        -0.03489084, 0.99905518, 0.02591298, 71.771;
        -0.35498844, 0.01184958, -0.9349116200000001, 475.07;
        0, 0, 0, 1;
            */
        
        //newright -312.529,331.377,224.170, 177.349,2.694,-129.738, 0.0115,-0.4250,0.9046,0.0309
       /*
         -0.6385119399999999, -0.7696206999999999, -0.005459200000000001, -312.529;
         -0.7681993, 0.63684038, 0.06567928000000001, 331.377;
         -0.0470708, 0.04612928, -0.9978523199999998, 224.17;
         0, 0, 0, 1;
         */
            -0.8227185000000001, -0.5611458, 0.09159679999999999, -483.905;
        -0.5575281999999999, 0.827724, 0.0636282, -18.073;
        -0.1115208, 0.001277800000000006, -0.9938385000000001, 355.518;
        0, 0, 0, 1;

        
            //11.4使用虚拟机修改end_to_base矩阵，发现需要先Ctrl S保存vscode后，终端输入make

         //5.29新建基准点，虚拟机新生成end_to_base矩阵，还未调试，修改了277行的MoveJCommand改为新建基准点


            // Matrix4d grap_to_base = R_T2RT(poisition, Mat);

             //获取手眼矩阵
            hand_in_eye();

        //Vector4d object(4, 1);
        object << 2000, 2000,3000,  1;/*-42.2215, 101.742, 504, 1; */
        //bt:348,328,345.667

        //抓取
        VectorXd point_to_base(4, 1);
        
 
       point_to_base = end_to_base * hand_in_eye() * object;
        // point_to_base << 366.236,-543.180,241.299, 1;


        //cout << "接受失败！" << endl;
        //3.12 把pick函数里面的WaitRobot(&robot)全部注释掉了,夹爪可以正常夹取
        
        //point_to_base(0), point_to_base(1),point_to_base(2)
    
        std::cout << "point_to_base：" << point_to_base(0) << "    " << point_to_base(1) << "    " << point_to_base(2) << "    " << endl;
        
       // rokae::MoveJCommand put_re({ point_to_base(0) / 1000, point_to_base(1) / 1000, (point_to_base(2) / 1000)+0.02 , /*176.427 * M_PI / 180,  -29.034 * M_PI / 180, 11.469 * M_PI / 180*/164.065 * M_PI / 180,  2.380 * M_PI / 180, -173.976 * M_PI / 180}, 200, 50);    //0.0318 * M_PI, 0.3 * M_PI, 0.2896 * M_PI, 0, 0.388 * M_PI, 0.016 * M_PI
       // robot.executeCommand({ put_re }, ec);
       // WaitRobot(&robot);
        

       
        //cout << "到达预备点" << endl;
        //Sleep(100);
        //robot.setDO(0, 1, true, ec);  
        //robot.setDO(0, 1, false, ec); 

        //176.427 * M_PI / 180,  -29.034 * M_PI / 180, 11.469 * M_PI / 180    164.065 * M_PI / 180,  2.380 * M_PI / 180, -173.976 * M_PI / 180 

        //-312.529,331.377,224.170
        rokae::MoveJCommand put({ point_to_base(0) / 1000 ,point_to_base(1) / 1000  ,point_to_base(2) / 1000 ,177.349 * M_PI /180, 2.694 * M_PI /180, -129.738 * M_PI /180 }, 200, 50);    //0.0318 * M_PI, 0.3 * M_PI, 0.2896 * M_PI, 0, 0.388 * M_PI, 0.016 * M_PI    3.14 *171.695/180, -22.961*3.14/180,  3.14*17.494/180 
        robot.executeCommand({ put }, ec);
        WaitRobot(&robot);
        std::cout << "到达目标点" << endl;
        Sleep(500);

        robot.setDO(0, 0, true, ec);
        robot.setDO(0, 1, false, ec);    //大寰闭合
        robot.setDO(0, 1, true, ec);    //慧灵闭合
       
        //robot.setDO(0, 0, true, ec);   //全毕   50力

        //放置
       /* rokae::MoveJCommand down(注释{568.8344 / 1000, 7.51982 / 1000, 278.3175 / 1000,   -M_PI,  -6.050313 * M_PI / 180,  0.835031 * M_PI / 180}注释
            {-515.012 / 1000, -296.810 / 1000, 326.045 / 1000, 169.568 * -M_PI / 180, -1.276 * M_PI / 180, -137.092 * M_PI / 180}, 400, 50);

        robot.executeCommand({ down }, ec);
        WaitRobot(&robot);*/
       /* rokae::MoveJCommand down_1({665.956 / 1000, -150.150 / 1000, 166.397 / 1000, 179.353 * -M_PI / 180, 1.931 * M_PI / 180, -123.668 * M_PI / 180}, 400, 50);
        robot.executeCommand({ down_1 }, ec);
        WaitRobot(&robot);
       
       */
        //rokae::MoveCCommand down({ 502.539 / 1000,  377.785 / 1000  ,94.160 / 1000 ,  M_PI  , 0,  M_PI },           //目标点
        //    { 568.8344 / 1000, 7.51982 / 1000, 278.3175 / 1000,   -M_PI,  -6.050313 * M_PI / 180,  0.835031 * M_PI / 180 },  //辅助点
        //    300, 50);
        //robot.executeCommand({ down }, ec);
        //WaitRobot(&robot);


        //robot.setDO(0, 0, false, ec);
        //robot.setDO(0, 1, true, ec);  // 75开   50力
        //robot.setDO(0, 1, false, ec);   //全毕  100力
        robot.setDO(0, 0, false, ec);    //慧灵打开
        robot.setDO(0, 1, true, ec);
        Sleep(500);
        std::cout << "抓取完成" << endl;

        //返回

        robot.executeCommand({ point26 }, ec);
        WaitRobot(&robot);
        Sleep(2000);

    
}


                                  /*********************   socket通讯   ****************************/

//配置传输缓存格式

union varr {
    int send_len;
    int recv_len;
    char send_buf[1024];
    char recv_buf[1024];
};
union data_t
{
    char buffer[3 * 4];   //uint8_t
    struct
    {
        float x;     //4字节
        float y;
        float z;
    }point;
};
//初始化套接字
void initialization() {
    //初始化套接字库
    WORD w_req = MAKEWORD(2, 2);//版本号
    WSADATA wsadata;
    int err;
    err = WSAStartup(w_req, &wsadata);
    if (err != 0) {
        std::cout << "初始化套接字库失败！" << endl;
    }
    else {
        std::cout << "初始化套接字库成功！" << endl;
    }
    //检测版本号
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
        std::cout << "套接字库版本号不符！" << endl;
        WSACleanup();
    }
    else {
        std::cout << "套接字库版本正确！" << endl;
    }
    //填充服务端地址信息

}
//做客户端通讯

void socket_client() {
    int send_len = 0;
    int recv_len = 0;
    char send_buf[1024] = {0};     // Initialize send buffer
    char recv_buf[1024] = {0};     // Initialize receive buffer
    SOCKET s_server;                //定义服务端套接字，接受请求套接字
    SOCKADDR_IN server_addr;           //服务端地址客户端地址
    initialization();
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.0.100");       //填充服务端信息
    server_addr.sin_port = htons(5010);
    s_server = socket(AF_INET, SOCK_STREAM, 0);   //创建套接字
    std::cout << "发送连接" << endl;
    if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
        std::cout << "服务器连接失败！" << endl;
        WSACleanup();
    }
    else {
        std::cout << "服务器连接成功！" << endl;
    }

    while (1) {

        union varr data;
        char send_buf[1024] = "预备";
        data.send_len = send(s_server, send_buf, sizeof(data.send_len), 0);
        if (data.send_len < 0) {
            std::cout << "发送失败！" << endl;
            Sleep(500);
            //robot.disconnectFromRobot(ec);
        }
        std::cout << "发送的数据为:" << data.send_len << endl;


        //接收相机三维点
        union data_t d;
        d.point.x = 1.0f;
        d.point.y = 2.0f;
        d.point.z = 3.0f;
                  /*   printf("%2x，%2x，%2x，%2x\n", d.buffer[0 + 0 * 4], d.buffer[1 + 0 * 4], d.buffer[2 + 0 * 4], d.buffer[3 + 0 * 4]);
                       printf("%2x，%2x，%2x，%2x\n", d.buffer[0 + 1 * 4], d.buffer[1 + 1 * 4], d.buffer[2 + 1 * 4], d.buffer[3 + 1 * 4]);
                       printf("%2x，%2x，%2x，%2x\n", d.buffer[0 + 2 * 4], d.buffer[1 + 2 * 4], d.buffer[2 + 2 * 4], d.buffer[3 + 2 * 4]);*/
        data_t dd;
        data.recv_len = recv(s_server, dd.buffer, 3 * 4, 0);
        if (data.recv_len < 0) {
            std::cout << "接受失败！" << endl;
            break;
        }
        else {
            object(0) = dd.point.x;
            object(1) = dd.point.y;
            object(2) = dd.point.z;
            object(3) = 1.0;
            std::cout << "服务器连接成功！" << endl;
            std::cout << "\n";
            std::cout << "object: " << object(0) << "  " << object(1) << "  " << object(2)  << "  " << object(3) << endl;;
            //std::cout << "\n";
           

            pick();
            std::cout << "服务器连接成功！" << endl;

            char send_buf_1[1024] = "完成";
            data.send_len = send(s_server, send_buf_1, sizeof(data.send_len), 0);
            if (data.send_len < 0) {
                std::cout << "发送失败！" << endl;
                Sleep(500);
              //  robot.disconnectFromRobot(ec);
            }

        }
        continue;
    }

        //关闭套接字
        closesocket(s_server);
        //释放DLL资源
        WSACleanup();
}


int main() {
    
    //cout << "发送连接" << endl;
    //connect_computer();
    //while (true) {
    //socket_client();
    pick();
    //}
    robot.setDO(0, 0, false, ec);    //慧灵打开
    //robot.setDO(0, 0, true, ec);     //(闭合
    //robot.setDO(0, 1, false, ec);    //大寰闭合(半闭合
    Sleep(2000);
    //robot.setDO(0, 0, false, ec);    //慧灵打开
    //robot.setDO(0, 1, true, ec);
    robot.setDO(0, 0, true, ec);     //(闭合
    //robot.setDO(0, 1, false, ec);
    //robot.setDO(0, 1, true, ec);
    robot.setDO(0, 0, false, ec);
    //robot.setDO(0, 0, true, ec);
    return 0;
}





