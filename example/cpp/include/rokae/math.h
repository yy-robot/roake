#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <iterator> 
#include <algorithm>  
using namespace cv;
using namespace std;
using namespace Eigen;


Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity(); //单位矩阵初始化


Eigen::rotVec
//欧拉角转旋转矩阵
Eigen::Matrix3d eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);

	eulerAngle /= 180 / CV_PI;
	cv::Matx13d m(eulerAngle);
	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto xs = std::sin(rx), xc = std::cos(rx);
	auto ys = std::sin(ry), yc = std::cos(ry);
	auto zs = std::sin(rz), zc = std::cos(rz);

	cv::Mat rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, xc, -xs, 0, xs, xc);
	cv::Mat rotY = (cv::Mat_<double>(3, 3) << yc, 0, ys, 0, 1, 0, -ys, 0, yc);
	cv::Mat rotZ = (cv::Mat_<double>(3, 3) << zc, -zs, 0, zs, zc, 0, 0, 0, 1);

	cv::Mat rotMat;

	if (seq == "zyx")		rotMat = rotX * rotY * rotZ;
	else if (seq == "yzx")	rotMat = rotX * rotZ * rotY;
	else if (seq == "zxy")	rotMat = rotY * rotX * rotZ;
	else if (seq == "xzy")	rotMat = rotY * rotZ * rotX;
	else if (seq == "yxz")	rotMat = rotZ * rotX * rotY;
	else if (seq == "xyz")	rotMat = rotZ * rotY * rotX;
	else {
		cv::error(cv::Error::StsAssert, "Euler angle sequence string is wrong.",
			__FUNCTION__, __FILE__, __LINE__);
	}

	if (!isRotationMatrix(rotMat)) {
		cv::error(cv::Error::StsAssert, "Euler angle can not convert to rotated matrix",
			__FUNCTION__, __FILE__, __LINE__);
	}

	return rotMat;
}


/** @brief ((四元数||欧拉角||旋转向量) && 转移向量) -> 4*4 的Rt
*	@param 	m				1*6 || 1*7的矩阵  -> 6  {x,y,z, rx,ry,rz}   7 {x,y,z, qw,qx,qy,qz}
*	@param 	useQuaternion	如果是1*7的矩阵，判断是否使用四元数计算旋转矩阵
*	@param 	seq				如果通过欧拉角计算旋转矩阵，需要指定欧拉角xyz的排列顺序如："xyz" "zyx" 为空表示旋转向量
*/
MatrixXd attitudeVectorToMatrix( m, bool useQuaternion, const std::string& seq)
{
	CV_Assert(m.total() == 6 || m.total() == 7);
	if (m.cols == 1)
		m = m.t();
	cv::Mat tmp = cv::Mat::eye(4, 4, CV_64FC1);
	//如果使用四元数转换成旋转矩阵则读取m矩阵的第第四个成员，读4个数据
	if (useQuaternion)	// normalized vector, its norm should be 1.
	{
		cv::Vec4d quaternionVec = m({ 3, 0, 4, 1 });
		quaternionToRotatedMatrix(quaternionVec).copyTo(tmp({ 0, 0, 3, 3 }));
		// cout << norm(quaternionVec) << endl; 
	}
	else
	{
		cv::Mat rotVec;
		if (m.total() == 6)
			rotVec = m({ 3, 0, 3, 1 });		//6
		else
			rotVec = m({ 7, 0, 3, 1 });		//10

		//如果seq为空表示传入的是旋转向量，否则"xyz"的组合表示欧拉角
		if (0 == seq.compare(""))
			cv::Rodrigues(rotVec, tmp({ 0, 0, 3, 3 }));
		else
			eulerAngleToRotatedMatrix(rotVec, seq).copyTo(tmp({ 0, 0, 3, 3 }));
	}
	tmp({ 3, 0, 1, 3 }) = m({ 0, 0, 3, 1 }).t();
	//std::swap(m,tmp);
	return tmp;
}




VectorXd eulerAngle(4, 1) ;   //初始化欧拉角
MatrixXd camera_to_grap(4, 4);
VectorXd point_to_base(4, 1);
MatrixXd grap_to_base(4, 4) ;
VectorXd poisiton(3, 1);
VectorXd tmp(4, 1);

object << 0, 0, 0, 1;
grap_to_base = Eigen::MatrixXd::Identity();

//4..回到最初点
rokae::MoveAbsJCommand base({ 0, M_PI / 6 , M_PI / 3, 0, M_PI / 2, 0,0 }, 200, 50);
robot.executeCommand({ base }, ec);
WaitRobot(&robot);
auto Pos = robot.flangePos(ec);   //获取法兰当前位姿XYZ,RPY

for (int i = 0; i < Pos.size(); i++) {  //xyz与rpy分开
	if (i < 4)
		poisiton[i] = Pos[i];
	else
		eulerAngle[i-3] = Pos[i];
}

//初始化旋转矩阵
Eigen::Matrix3d rotation_matrix; 
rotation_matrix << x_00, x_01, x_02, x_10, x_11, x_12, x_20, x_21, x_22;





grap_to_base <<
1, 0, 0, 2,
0, 1, 0, 5,
0, 0, 1, 170;


camera_to_grap <<

//慧灵
-0.09442971, -0.98910085, -0.11297144, 129.96034,
0.99542364, -0.09213896, -0.02534137, 28.90779,
0.0146561, -0.11484742, 0.99327502, -154.50424,
0, 0, 0, 1;

point_to_base = grap_to_base * camera_to_grap * object;

std::cout << point_to_base << "   ";