#include <iostream>
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace std;

#define M_PI 3.14

int main(int argc, char** argv)
{
    // 机器人B在坐标系O中的位姿：
    Eigen::Vector3d BinO(3, 4, M_PI);
    // 坐标系B到坐标系O的转换矩阵：
    Eigen::Matrix3d B2O;
    B2O << cos(BinO(2)), -sin(BinO(2)), BinO(0),
           sin(BinO(2)),  cos(BinO(2)), BinO(1),
              0,          0,        1;
    // 坐标系O到坐标系B的转换矩阵:
    Eigen::Matrix3d O2B = B2O.inverse();


    // 机器人A在坐标系O中的姿态：
    Eigen::Vector3d AinO(1, 3, -M_PI / 2);
    // 坐标系A到坐标系O的转换矩阵
    Eigen::Matrix3d A2O;
    A2O <<  cos(AinO(2)), -sin(AinO(2)), AinO(0),
            sin(AinO(2)),  cos(AinO(2)), AinO(1),
            0,          0,        1;

/* ------------------ // start your code here (5~10 lines) ------------------ */

    // A到B的变换矩阵，等于 O到B的转换矩阵乘以A到O的转换矩阵
    Eigen::Matrix3d A2B;
    A2B = O2B * A2O;

    // 求机器人A在机器人B中的坐标 等于 A 在 B中的位姿
    Eigen::Vector3d AinB(A2B(0,2), A2B(1,2), atan2(A2B(1,0),A2B(0,0)));

/* -------------------------- // end your code here ------------------------- */

    cout << "The right answer is BA: 2 1 1.5708" << endl;
    cout << "Your answer is BA: " << AinB.transpose() << endl;

    system("pause");
    return 0;
}
