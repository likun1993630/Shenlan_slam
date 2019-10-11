#include <iostream>
#include <Eigen/Core>
#include <cmath>
//#include <Eigen/Dense>
#include <Eigen/Geometry>

//using namespace std;

int main(int argc, char** argv)
{

	Eigen::Quaterniond q(2, 0, 1, -3);
	std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " << std::endl << q.vec() << std::endl;
	q.normalize();// 无返回值，直接将q归一化，归一化是有必要的
	std::cout << "To represent rotation, we need to normalize it such that its length is " << q.norm() << std::endl;
	Eigen::Vector3d v(1, 2, -1);

	//初始化 四元数p，即使用虚四元数表示点p的坐标
	Eigen::Quaterniond p;
	p.w() = 0;
	p.vec() = v;

	//求旋转之后的p点的坐标
	Eigen::Quaterniond rotatedP = q * p * q.inverse();
	//std::cout << "rotatedP" << rotatedP.coeffs() <<std::endl;
	Eigen::Vector3d rotatedV = rotatedP.vec();
	std::cout << "We can now use it to rotate a vector" << std::endl << v << "to"<< std::endl << rotatedV << std::endl;
	Eigen::Matrix3d R = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
	std::cout << "Compare with the result using an rotation matrix "<< std::endl << R * v << std::endl;

	//四元数加法运算
	Eigen::Quaterniond a = Eigen::Quaterniond::Identity();
	Eigen::Quaterniond b = Eigen::Quaterniond::Identity();
	Eigen::Quaterniond c; // Adding two quaternion as two 4x1 vectors is not supported by the EIgen API. That is, c = a + b is not allowed. We have to do this in a hard way
	c.w() = a.w() + b.w();
	c.x() = a.x() + b.x();
	c.y() = a.y() + b.y();
	c.z() = a.z() + b.z();
	std::cout <<"c: \n" << c.coeffs() <<std::endl;

	return 0;
}


