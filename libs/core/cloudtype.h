//
// Created by LBC on 2026/1/26.
//

#ifndef POINTWORKS_CLOUDTYPE_H
#define POINTWORKS_CLOUDTYPE_H

#define _USE_MATH_DEFINES

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

namespace ct {
    typedef pcl::PointXYZ PointXYZ;
    typedef pcl::PointXYZRGB PointXYZRGB;
    typedef pcl::PointXYZRGBNormal PointXYZRGBN;
    typedef pcl::Normal PointNormal;
    typedef pcl::Indices Indices;
    typedef pcl::console::TicToc TicToc;

    // 包围盒
    struct Box {
        double width = 0.0;
        double height = 0.0;
        double depth = 0.0;

        Eigen::Affine3f pose = Eigen::Affine3f::Identity();
        Eigen::Vector3f translation = Eigen::Vector3f::Zero();
        Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
    };

    // 坐标系
    struct Coord {
        std::string id;
        double scale;
        Eigen::Affine3f pose;

        Coord()
            : scale(1.0), pose(Eigen::Affine3f::Identity()) {}

        Coord(const std::string& id_, double scale_, const Eigen::Affine3f& pose_)
            : id(id_), scale(scale_), pose(pose_) {}
    };

    struct ColorRGB {
        ColorRGB() = default;

        ColorRGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}

        double rf() const { return (double) r / 255; }

        double gf() const { return (double) g / 255; }

        double bf() const { return (double) b / 255; }

        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;
    };

    // 压缩法线（球面坐标编码，2 bytes）
    struct CompressedNormal {
        uint16_t data = 0;

        // 从 3D 向量编码
        void set(const Eigen::Vector3f &n) {
            float len = n.norm();
            if (len < 1e-6f) {
                data = 0;
                return;
            }

            Eigen::Vector3f normalized = n / len;

            // 计算 phi (极角): [0, π]
            float phi = std::acos(std::clamp(normalized.z(), -1.0f, 1.0f));

            // 计算 theta (方位角): [0, 2π]
            float theta = std::atan2(normalized.y(), normalized.x());
            if (theta < 0) theta += 2.0f * M_PI;

            // 编码: theta(9 bits) | phi(7 bits)
            uint16_t theta_bits = static_cast<uint16_t>(theta / (2.0f * M_PI) * 511.0f);
            uint16_t phi_bits = static_cast<uint16_t>(phi / M_PI * 127.0f);

            data = (theta_bits << 7) | phi_bits;
        }

        // 解码为 3D 向量
        Eigen::Vector3f get() const {
            uint16_t theta_bits = (data >> 7) & 0x1FF;
            uint16_t phi_bits = data & 0x7F;

            float theta = theta_bits / 511.0f * 2.0f * M_PI;
            float phi = phi_bits / 127.0f * M_PI;

            float sin_phi = std::sin(phi);

            return Eigen::Vector3f(
                    sin_phi * std::cos(theta),
                    sin_phi * std::sin(theta),
                    std::cos(phi)
            );
        }

        bool isZero() const { return data == 0; }
    };

    // define color
    namespace Color {
        const ColorRGB White = {255, 255, 255};
        const ColorRGB Black = {0, 0, 0};
        const ColorRGB Red = {255, 0, 0};
        const ColorRGB Green = {0, 255, 0};
        const ColorRGB Blue = {0, 0, 255};
        const ColorRGB Yellow = {255, 255, 0};
        const ColorRGB Cyan = {0, 255, 255};
        const ColorRGB Purple = {255, 0, 255};
    }

    /**
     * @brief 自动八叉树配置参数
     */
    namespace AutoOctreeConfig {
        // 启用八叉树的最小点数阈值 (小于此值走直通模式)
        static constexpr size_t MIN_POINTS_FOR_OCTREE = 10000000;

        // 默认的标准块大小 (用于初始化或回退)
        // 设为 6万 是比较通用的经验值
        static constexpr size_t DEFAULT_BLOCK_SIZE = 60000;

        // 目标块数量 (用于反推理想的块大小)
        // 并不是强制只有这么多块，而是作为一个参考基准
        static constexpr size_t TARGET_BLOCK_COUNT = 1024;

        // 块大小限制 (每个叶子节点容纳的点数)
        static constexpr size_t MIN_BLOCK_SIZE = 30000;    // 太小会导致视锥剔除开销过大
        static constexpr size_t MAX_BLOCK_SIZE = 1000000;  // 太大会导致单次上传显卡卡顿

        // LOD 采样比例 (LOD点数 = Block点数 * Ratio)
        static constexpr float  LOD_POINT_RATIO = 0.50f;

        // LOD 大小限制
        static constexpr size_t MIN_LOD_SIZE = 10000;      // 太稀疏看不清
        static constexpr size_t MAX_LOD_SIZE = 100000;     // 太密影响渲染性能

        // 默认最大深度
        static constexpr int    DEFAULT_MAX_DEPTH = 8;
    }

    /**
     * @brief 点云配置参数
     */
    struct CloudConfig {
        // 是否启用八叉树 (点数过少时关闭)
        bool enableOctree = true;

        // 使用常量初始化，消除硬编码
        size_t maxPointsPerBlock = AutoOctreeConfig::DEFAULT_BLOCK_SIZE;

        // 默认 LOD = Block * Ratio
        size_t maxLODPoints = static_cast<size_t>(AutoOctreeConfig::DEFAULT_BLOCK_SIZE * AutoOctreeConfig::LOD_POINT_RATIO);

        int maxDepth = AutoOctreeConfig::DEFAULT_MAX_DEPTH;

        // 渲染预算 (可选，暂时预留)
        size_t pointBudget = 10000000;
    };

} // namespace ct

#endif POINTWORKS_CLOUDTYPE_H