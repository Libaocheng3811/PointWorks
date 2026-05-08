
#include "common.h"

#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace pw {
    void HSVtoRGB(float h, float s, float v, float &r, float &g, float &b) {
        // 如果饱和度为0，表示颜色为灰色，所有的RGB值等于明度v，然后直接返回
        if (s == 0.0f) {
            r = g = b = v;
            return;
        }

        // fmodf 函数将 h 限定在0到1之间，并将其转换为0到6的范围。
        h = fmodf(h, 1.0f) / (60.0f / 360.0f);
        // i为整数部分，f为小数部分
        int i = (int) h;
        float f = h - (float) i;
        // 计算RGB的辅助值
        float p = v * (1.0f - s);
        float q = v * (1.0f - s * f);
        float t = v * (1.0f - s * (1.0f - f));

        // 根据色相区间确定RGB值
        switch (i)
        {
            case 0:
                r = v, g = t, b = p;
                break;
            case 1:
                r = q, g = v, b = p;
                break;
            case 2:
                r = p, g = v, b = t;
                break;
            case 3:
                r = p, g = q, b = v;
                break;
            case 4:
                r = t, g = p, b = v;
                break;
            case 5:
                r = v, g = p, b = q;
                break;
            default:
                r = v, g = p, b = q;
                break;
        }
    }

    void getEulerAngles(const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw)
    {
        // 通过仿射变换矩阵提取欧拉角
        pcl::getEulerAngles(t, roll, pitch, yaw);
        //  将弧度转为度
        roll = pcl::rad2deg(roll);
        pitch = pcl::rad2deg(pitch);
        yaw = pcl::rad2deg(yaw);
    }

    void getTransformation(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Affine3f& t)
    {
        return pcl::getTransformation(x, y, z, pcl::deg2rad(roll), pcl::deg2rad(pitch), pcl::deg2rad(yaw), t);
    }

    std::string getTransformationString(const Eigen::MatrixXf& mat, int decimals)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(decimals);
        for (int i = 0; i < mat.rows(); i++)
        {
            for (int j = 0; j < mat.cols(); j++)
            {
                oss << mat(i, j) << " ";
            }
            if (i < mat.rows() - 1) oss << "\n";
        }
        return oss.str();
    }

    Eigen::Affine3f getTransformation(float x, float y, float z, float roll, float pitch, float yaw)
    {
        // 调用PCL库的函数pcl::getTransformation计算从三维空间坐标到仿射变换的矩阵
        return pcl::getTransformation(x, y, z, pcl::deg2rad(roll), pcl::deg2rad(pitch), pcl::deg2rad(yaw));
    }

    void getAngleAxis(const Eigen::Affine3f& t, float& angle, float& axisX, float& axisY, float& axisZ)
    {
        Eigen::AngleAxisf aa(t.rotation());
        angle = pcl::rad2deg(aa.angle());
        axisX = aa.axis().x();
        axisY = aa.axis().y();
        axisZ = aa.axis().z();
    }

    void getTranslationAndEulerAngles(const Eigen::Affine3f& t, float& x, float& y, float& z,
                                       float& roll, float& pitch, float& yaw)
    {
        x = t.translation().x();
        y = t.translation().y();
        z = t.translation().z();
        getEulerAngles(t, roll, pitch, yaw);
    }

    Eigen::Affine3f getTransformation(float angle, float ax, float ay, float az,
                                       float tx, float ty, float tz)
    {
        Eigen::Vector3f axis(ax, ay, az);
        if (axis.norm() > 0) axis.normalize();
        Eigen::AngleAxisf rotation(pcl::deg2rad(angle), axis);
        Eigen::Affine3f t = Eigen::Translation3f(tx, ty, tz) * rotation;
        return t;
    }
}
