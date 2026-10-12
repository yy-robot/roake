//根据固定点生成end_to_base矩阵
#include <iostream>
#include <cmath>
#include <iomanip>

class Matrix4d {
public:
    double data[4][4];

    Matrix4d() {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                data[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }
};

// 创建ZYX顺序的旋转矩阵
Matrix4d createRotationMatrixZYX(double rx, double ry, double rz) {
    Matrix4d rot;

    double cos_rx = cos(rx), sin_rx = sin(rx);
    double cos_ry = cos(ry), sin_ry = sin(ry);
    double cos_rz = cos(rz), sin_rz = sin(rz);

    // ZYX顺序旋转矩阵 - 正确
    rot.data[0][0] = cos_rz * cos_ry;
    rot.data[0][1] = cos_rz * sin_ry * sin_rx - sin_rz * cos_rx;
    rot.data[0][2] = cos_rz * sin_ry * cos_rx + sin_rz * sin_rx;

    rot.data[1][0] = sin_rz * cos_ry;
    rot.data[1][1] = sin_rz * sin_ry * sin_rx + cos_rz * cos_rx;
    rot.data[1][2] = sin_rz * sin_ry * cos_rx - cos_rz * sin_rx;

    rot.data[2][0] = -sin_ry;
    rot.data[2][1] = cos_ry * sin_rx;
    rot.data[2][2] = cos_ry * cos_rx;

    return rot;
}

// 创建平移矩阵
Matrix4d createTranslationMatrix(double x, double y, double z) {
    Matrix4d trans;
    trans.data[0][3] = x;
    trans.data[1][3] = y;
    trans.data[2][3] = z;
    return trans;
}

// 矩阵乘法
Matrix4d multiplyMatrices(const Matrix4d& a, const Matrix4d& b) {
    Matrix4d result;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result.data[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                result.data[i][j] += a.data[i][k] * b.data[k][j];
            }
        }
    }
    return result;
}

// 生成固定点的 end_to_base 矩阵 - 修正版本
Matrix4d generateEndToBaseMatrix() {
    // point26 的位姿数据
    double x = -476.209;  // mm
    double y = 37.857;    // mm
    double z = 276.157;   // mm

    double rx = -174.227 * M_PI / 180;  // 弧度
    double ry = -5.622 * M_PI / 180;     // 弧度
    double rz = -27.581 * M_PI / 180;  // 弧度

    // 创建旋转和平移矩阵
    Matrix4d rotation = createRotationMatrixZYX(rx, ry, rz);
    Matrix4d translation = createTranslationMatrix(x, y, z);

    // 修正：正确的组合顺序应该是 rotation × translation
    return multiplyMatrices(rotation, translation);
}

// 打印矩阵
void printMatrix(const Matrix4d& matrix) {
    std::cout << "End to Base Matrix:" << std::endl;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << std::fixed << std::setprecision(6) << std::setw(12) << matrix.data[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

// 验证矩阵的函数
void verifyMatrix(const Matrix4d& matrix) {
    std::cout << "\n=== Matrix Verification ===" << std::endl;

    // 检查旋转矩阵部分是否正交
    double dot00 = matrix.data[0][0] * matrix.data[0][0] + matrix.data[0][1] * matrix.data[0][1] + matrix.data[0][2] * matrix.data[0][2];
    double dot11 = matrix.data[1][0] * matrix.data[1][0] + matrix.data[1][1] * matrix.data[1][1] + matrix.data[1][2] * matrix.data[1][2];
    double dot22 = matrix.data[2][0] * matrix.data[2][0] + matrix.data[2][1] * matrix.data[2][1] + matrix.data[2][2] * matrix.data[2][2];

    std::cout << "Rotation matrix norms (should be ~1.0):" << std::endl;
    std::cout << "X-axis: " << std::sqrt(dot00) << std::endl;
    std::cout << "Y-axis: " << std::sqrt(dot11) << std::endl;
    std::cout << "Z-axis: " << std::sqrt(dot22) << std::endl;

    // 检查行列式（应该接近1）
    double det = matrix.data[0][0] * (matrix.data[1][1] * matrix.data[2][2] - matrix.data[1][2] * matrix.data[2][1])
        - matrix.data[0][1] * (matrix.data[1][0] * matrix.data[2][2] - matrix.data[1][2] * matrix.data[2][0])
        + matrix.data[0][2] * (matrix.data[1][0] * matrix.data[2][1] - matrix.data[1][1] * matrix.data[2][0]);

    std::cout << "Determinant: " << det << " (should be close to 1.0)" << std::endl;
}

int main() {
    std::cout << "=== Fixed Point End to Base Matrix Generation ===" << std::endl;

    // 生成并打印 end_to_base 矩阵
    Matrix4d end_to_base = generateEndToBaseMatrix();
    printMatrix(end_to_base);

    // 验证矩阵性质
    verifyMatrix(end_to_base);

    return 0;
}