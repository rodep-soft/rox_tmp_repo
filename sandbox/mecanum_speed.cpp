#include <iostream>
#include <cmath>
#include <Eigen/Dense>

const int WHIDE = 0.40;
const int LENGTH = 0.40;
const int WHEEL_RADIUS = 0.02;

int main(){
    //それぞれのrpmm
    float rpm[4] = {0.0, 0.0, 0.0, 0.0};//[FL, FR, RL, RR]  rotation/min

    //rpmをrad/sに変換
    
    Eigen::Vector4f rad_s;
    for(int i = 0;i < 4;i++){
        rad_s[i] = rpm[i] * 2 * M_PI /60.0;
    }

    Eigen::Matrix4f A;
    A << 1, -1, -(WHIDE + LENGTH)/2,
        1, 1, -(WHIDE + LENGTH)/2,
        1, -1, (WHIDE + LENGTH)/2,
        1, 1, (WHIDE + LENGTH)/2;

    Eigen::Matrix3f A_inverce = (A.transpose() * A).inverse() * A.transpose();
    Eigen::Vector3f speed = A_inverce * rad_s;

    std::cout << "vx = "<< speed[0] << " m/s" << std::endl;
    std::cout << "vy = "<< speed[1] << " m/s" << std::endl;
    std::cout << "wz = "<< speed[2] << " rad/s" << std::endl;

}