#ifndef POINTWORKS_COMMON_H
#define POINTWORKS_COMMON_H

#include "exports.h"

#include <string>
#include <Eigen/Geometry>

namespace ct
{
    /**
     * @brief 将颜色从HSV格式转换为RGB格式
     * 在HSV模型中，颜色是通过三个分量表示的：色调（Hue），饱和度（Saturation）和明度（Value）
     */
    void CT_EXPORT HSVtoRGB(float h, float s, float v, float& r, float& g, float& b);

    /**
     * @brief 从给定的变换中提取欧拉角（内在旋转， ZYX约定）
     */
    void CT_EXPORT getEulerAngles(const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw);

    /**
     * @brief 从给定的变换中提取轴角（内在旋转，ZYX约定）
     */
    void CT_EXPORT getAngleAxis(const Eigen::Affine3f& t, float& angle, float& axisX, float& axisY, float& axisZ);

    /**
     * @brief 从给定的变换中提取x, y, z和欧拉角（内在旋转，ZYX约定）
     */
    void CT_EXPORT getTranslationAndEulerAngles(const Eigen::Affine3f& t, float& x, float& y, float& z, float& roll, float& pitch, float& yaw);

    /**
     * @brief 从给定的平移和欧拉角（内在旋转，ZYX约定）创建变换
     */
    void CT_EXPORT getTransformation(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Affine3f& t);

    /**
     * @brief 将给定的变换转换为字符串
     * @param decimals 有效位数
     */
    std::string CT_EXPORT getTransformationString(const Eigen::MatrixXf& mat, int decimals);

    /**
     * @brief 从给定的平移和欧拉角（内在旋转，ZYX约定）创建变换
     */
    Eigen::Affine3f CT_EXPORT getTransformation(float x, float y, float z, float roll, float pitch, float yaw);

    /**
     * @brief 从给定的轴角和平移创建变换
     * @param angle 旋转角度（度）
     * @param ax, ay, az 旋转轴方向
     * @param tx, ty, tz 平移量
     */
    Eigen::Affine3f CT_EXPORT getTransformation(float angle, float ax, float ay, float az,
                                                float tx, float ty, float tz);
}


#endif //POINTWORKS_COMMON_H
