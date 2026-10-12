//矩阵运算
#include <iostream>
#include <string>
#include <cmath>
#include <iomanip>
#include "rokae/robot.h"
#include <Eigen/Dense>

using namespace rokae;
using namespace Eigen;
using namespace std;

// 函数声明
Matrix4d eulerAngleToRotatedMatrix(Vector3d& eulerAngle, Vector3d& position);
Matrix4d hand_in_eye();

int main(int argc, char* argv[]) {
    try {
        std::cout << "Test3 program started: Point to Base Matrix Calculation" << std::endl;
        std::cout << "=======================================================\n" << std::endl;

        // 1. 定义测试数据
        // 目标点的齐次坐标 (mm)
        VectorXd object(4);
        object << 275.0, 307.0, 422.0, 1.0;

        // 2. 手动输入末端到基坐标系的变换矩阵（从用户提供的值）
        Matrix4d end_to_base;
        end_to_base <<
            /*
            0.88206936, -0.45195928, 0.13306624, -476.209,
            -0.46082776, -0.88657704, 0.04406368, 37.857,
            0.09803872, -0.10018304, -0.99029904, 276.157,
            0, 0, 0, 1;
            */
            /*
            -0.5300125000000002, 0.8315146, 0.1660278, -558.0650000000001,
            0.8370314000000001, 0.54443, -0.05425079999999999, 258.126,
            -0.1354998, 0.1102188, -0.9845585000000001, 22.841,
            0, 0, 0, 1;
            */
            /*
            -0.5300125000000002, 0.8315146, 0.1660278, -558.0650000000001,
            0.8370314000000001, 0.54443, -0.05425079999999999, 258.126,
            -0.1354998, 0.1102188, -0.9845585000000001, 72.84099999999999,
            0, 0, 0, 1;
            */
            0.6412401599999999, -0.75034148, 0.1606878, -459.538,
            -0.7547358000000001, -0.6545928000000003, -0.04452980000000001, 90.742,
            0.13858852, -0.09271836, -0.9860616000000002, 335.118,
            0, 0, 0, 1;


        std::cout << "1. End to Base Matrix (end_to_base):" << std::endl;
        std::cout << std::fixed << std::setprecision(4) << end_to_base << std::endl;
        std::cout << std::endl;

        // 3. 获取手眼矩阵
        Matrix4d hand_eye = hand_in_eye();

        // 4. 计算point_to_base矩阵
        VectorXd point_to_base(4);
        point_to_base = end_to_base * hand_eye * object;

        std::cout << "3. Object coordinates (object):" << std::endl;
        std::cout << std::fixed << std::setprecision(4) << object.transpose() << std::endl;
        std::cout << std::endl;

        std::cout << "4. Calculated point_to_base matrix:" << std::endl;
        std::cout << std::fixed << std::setprecision(4) << point_to_base.transpose() << std::endl;
        std::cout << std::endl;

        // 5. 输出结果解释
        std::cout << "Result interpretation:" << std::endl;
        std::cout << "- Point in base coordinate system (x, y, z):" << std::endl;
        std::cout << "  x: " << std::fixed << std::setprecision(4) << point_to_base(0) << " mm" << std::endl;
        std::cout << "  y: " << std::fixed << std::setprecision(4) << point_to_base(1) << " mm" << std::endl;
        std::cout << "  z: " << std::fixed << std::setprecision(4) << point_to_base(2) << " mm" << std::endl;

        std::cout << "\nTest3 program completed successfully." << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}

// 欧拉角转旋转矩阵函数
Matrix4d eulerAngleToRotatedMatrix(Vector3d& eulerAngle, Vector3d& position) {
    Matrix3d  rotX, rotY, rotZ;
    Matrix3d rotation_matrix;
    Matrix4d RT;

    auto rx = eulerAngle(0), ry = eulerAngle(1), rz = eulerAngle(2);
    auto xs = std::sin(rx), xc = std::cos(rx);
    auto ys = std::sin(ry), yc = std::cos(ry);
    auto zs = std::sin(rz), zc = std::cos(rz);

    // 绕X轴旋转
    rotX << 1, 0, 0,
        0, xc, -xs,
        0, xs, xc;

    // 绕Y轴旋转
    rotY << yc, 0, ys,
        0, 1, 0,
        -ys, 0, yc;

    // 绕Z轴旋转
    rotZ << zc, -zs, 0,
        zs, zc, 0,
        0, 0, 1;

    // 组合旋转矩阵 (Z*Y*X)
    rotation_matrix = rotZ * rotY * rotX;

    // 构造齐次变换矩阵
    RT << rotation_matrix, position,
        0, 0, 0, 1;

    return RT;
}

// 手眼矩阵函数
Matrix4d hand_in_eye() {
    // 设置手眼矩阵 (从相机到夹爪的变换)
    Matrix4d camera_to_grap;
    camera_to_grap <<
        0.07809988, 0.99656242, 0.02763591, -125.12694,
        -0.99675085, 0.07750681, 0.02191882, -24.59342,
        0.0197015, -0.02925797, 0.99937772, 95.19177,
        0.0000001, 0.0000001, 0.0000001, 1.0000001;

    std::cout << "2. Hand-Eye Matrix (camera_to_grap):" << std::endl;
    std::cout << std::fixed << std::setprecision(4) << camera_to_grap << std::endl;
    std::cout << std::endl;

    return camera_to_grap;
}