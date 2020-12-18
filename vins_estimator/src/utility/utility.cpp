/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "utility.h"

/**
 *  获取使数据g与重力对齐的旋转矩阵R0
 * https://blog.csdn.net/huanghaihui_123/article/details/103075107
 * @param g
 * @return
 */
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    //  返回R^{ng1}_{ng2}
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    /**
     * Q:为什么要在这把yaw旋转量（相对于理想imu参考系{0,0,1}）置零？
     * A:因为这里理想imu参考系没有yaw方向，所以旋转矩阵反算的yaw应该是随机值，需要消掉
     */
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
