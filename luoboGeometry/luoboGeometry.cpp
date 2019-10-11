#include <iostream>
#include <Eigen/Core>
//#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv)
{
	Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2);
	Eigen::Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
	Eigen::Vector3d t1(0.7, 1.1, 0.2);
	Eigen::Vector3d t2(-0.1, 0.4, 0.8);
	Eigen::Vector3d p0;
	Eigen::Vector3d p1(0.5, -0.1, 0.2);
	Eigen::Vector3d p2;

	//四元数的归一化
	// q1.normalize(); //可定义无返回值的归一化，要区别q1.normalized(),后者不改变q1
    Eigen::Quaterniond q1n = q1.normalized();
    cout << "q1:"<<endl << q1.coeffs().transpose()<<endl;
    cout << "normalized q1:"<<endl << q1n.coeffs().transpose()<<endl;
    Eigen::Quaterniond q2n = q2.normalized();
    cout << "q2:"<<endl << q2.coeffs().transpose()<<endl;
    cout << "normalized q2:"<<endl << q2n.coeffs().transpose()<<endl;

    //求变换矩阵
    Eigen::Isometry3d Tq1w = Eigen::Isometry3d::Identity();
    Tq1w.rotate (q1n);
    Tq1w.pretranslate(t1);
    cout<< "Transform matrix Tq1 = \n" << Tq1w.matrix() << endl;

    Eigen::Isometry3d Tq2w = Eigen::Isometry3d::Identity();
    //等价方式：
    //Eigen::Matrix3d q1R = q1n.toRotationMatrix();
    //Tq2.rotate(q1R)
    Tq2w.rotate(q2n);
    Tq2w.pretranslate(t2);
    cout<<"Transform matrix Tq2 = \n" << Tq2w.matrix() <<endl;

    //通过变换矩阵求解点在小萝卜2坐标系下的坐标
    p0 = Tq1w.inverse() * p1;
    p2 = Tq2w * p0;
    cout<< "Position \n" << p2.transpose();

    // 归一化的四元数直接对旋转向量进行初始化
    //Eigen::AngleAxisd p(q1n);
    //cout << "q" <<endl << p.matrix() <<endl;

	return 0;
}
