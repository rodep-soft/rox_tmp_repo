#include <Eigen/Dense>
#include <cmath>
#include <iostream>

const float WIDE = 0.40;
const float LENGTH = 0.40;
const float WHEEL_RADIUS = 0.02;

int main() {
  // それぞれのrpmm
  float rpm[4] = {40.0, 40.0, 40.0, 40.0};  //[FL, FR, RL, RR]  rotation/min

  Eigen::Vector4f rad_s;      //  rad/s
  Eigen::Vector4f speed_m_s;  //  m/s

  for (int i = 0; i < 4; i++) {
    rad_s[i] = rpm[i] * 2 * M_PI / 60.0;     // 角速度(rad/s)に変換
    speed_m_s[i] = rad_s[i] * WHEEL_RADIUS;  // 角速度→線速度(m/s)に変換
  }

  Eigen::Matrix<float, 4, 3> A;
  A << 1, -1, -(WIDE + LENGTH) / 2, 1, 1, -(WIDE + LENGTH) / 2, 1, -1,
      (WIDE + LENGTH) / 2, 1, 1, (WIDE + LENGTH) / 2;

  // Aの逆行列を計算
  Eigen::Matrix3f A_inverse = (A.transpose() * A).inverse() * A.transpose();
  Eigen::Vector3f speed = A_inverse * speed_m_s;

  std::cout << "vx = " << speed[0] << " m/s" << std::endl;  // x方向の速度
  std::cout << "vy = " << speed[1] << " m/s" << std::endl;  // y方向の速度
  std::cout << "wz = " << speed[2] << " rad/s" << std::endl;  // z軸周りの角速度

  return 0;
}