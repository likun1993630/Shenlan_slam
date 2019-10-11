#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ctime>

using namespace std;
#define MATRIX_SIZE 100


int main(int argc, char** argv)
{
    //定义系数矩阵 A
	Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> A;
	A = Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE>::Random();

	//将矩阵A转化为一个对称矩阵（但不一定是一个对称正定矩阵）
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> A_sym;
    A_sym = A;
    for(int i=0; i<MATRIX_SIZE; i++){
        for(int j=0; j<MATRIX_SIZE; j++){
            if(i>j){
                A_sym(i,j) = A(j,i);
            }
        }

    }

    A = A_sym;

	//定义 b
	Eigen::Matrix<double, MATRIX_SIZE, 1> b;
    b = Eigen::Matrix<double, MATRIX_SIZE, 1>::Random();

    //设置计时器
    clock_t time_stt = clock();
    //使用求逆的方式求解线性方程
    Eigen::Matrix<double, MATRIX_SIZE,1> x = A.inverse()*b;
    cout << "x_soution_with_inverse = " << x.transpose() << endl;
    cout << "time use in normal inverse is" << 1000* (clock() - time_stt)/ (double)CLOCKS_PER_SEC <<"ms" <<endl;

    //使用矩阵分解QR分解求解方程
    time_stt = clock();
    x = A.colPivHouseholderQr().solve(b);
    cout << "x_solution_with_Qr" << x.transpose() <<endl;
    cout<< "time use in Qr decompostion is" << 1000* (clock() - time_stt)/ (double)CLOCKS_PER_SEC <<"ms" <<endl;

    //使用矩阵分解Cholesky分解求解方程
    time_stt = clock();
    x = A.ldlt().solve(b);
    cout << "x_solution_with_Cholesky" << x.transpose() <<endl;
    cout<< "time use in Cholesky decompostion is" << 1000* (clock() - time_stt)/ (double)CLOCKS_PER_SEC <<"ms" <<endl;

	return 0;

}
