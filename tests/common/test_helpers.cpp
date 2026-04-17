#include "test_helpers.h"

#include <pcl/common/transforms.h>

#include <Eigen/Eigenvalues>

#include <numeric>

namespace test_helpers {

// ============ 基础几何体生成 ============

ct::Cloud::Ptr makePlane(int n, float width, float depth, float noise, unsigned seed) {
    auto cloud = std::make_shared<ct::Cloud>();
    std::vector<ct::PointXYZ> pts(n);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dx(-width / 2, width / 2);
    std::uniform_real_distribution<float> dy(-depth / 2, depth / 2);
    std::normal_distribution<float> gauss(0.0f, noise);

    for (int i = 0; i < n; ++i) {
        pts[i] = {dx(rng), dy(rng), noise > 0 ? gauss(rng) : 0.0f};
    }

    cloud->addPoints(pts);
    cloud->update();
    return cloud;
}

ct::Cloud::Ptr makeSphere(int n, float radius, float noise, unsigned seed) {
    auto cloud = std::make_shared<ct::Cloud>();
    std::vector<ct::PointXYZ> pts(n);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> theta(0.0f, 2.0f * static_cast<float>(M_PI));
    std::uniform_real_distribution<float> phi(0.0f, static_cast<float>(M_PI));
    std::normal_distribution<float> gauss(0.0f, noise);

    for (int i = 0; i < n; ++i) {
        float t = theta(rng);
        float p = phi(rng);
        float r = radius;
        if (noise > 0) r += gauss(rng);
        pts[i] = {
            r * std::sin(p) * std::cos(t),
            r * std::sin(p) * std::sin(t),
            r * std::cos(p)
        };
    }

    cloud->addPoints(pts);
    cloud->update();
    return cloud;
}

ct::Cloud::Ptr makeCube(int n, float size, float noise, unsigned seed) {
    auto cloud = std::make_shared<ct::Cloud>();
    std::vector<ct::PointXYZ> pts(n);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> pos(-size / 2, size / 2);
    std::normal_distribution<float> gauss(0.0f, noise);
    int face = n / 6;  // 每个面约 n/6 个点

    for (int i = 0; i < n; ++i) {
        int f = i / face;
        if (f > 5) f = 5;
        float a = pos(rng);
        float b = pos(rng);
        float nz = noise > 0 ? gauss(rng) : 0.0f;
        float half = size / 2;

        switch (f) {
        case 0: pts[i] = { a,  b,  half + nz}; break;  // +Z
        case 1: pts[i] = { a,  b, -half + nz}; break;  // -Z
        case 2: pts[i] = { a,  half + nz, b}; break;    // +Y
        case 3: pts[i] = { a, -half + nz, b}; break;    // -Y
        case 4: pts[i] = { half + nz, a,  b}; break;    // +X
        case 5: pts[i] = {-half + nz, a,  b}; break;    // -X
        }
    }

    cloud->addPoints(pts);
    cloud->update();
    return cloud;
}

ct::Cloud::Ptr makeCylinder(int n, float radius, float height, float noise, unsigned seed) {
    auto cloud = std::make_shared<ct::Cloud>();
    std::vector<ct::PointXYZ> pts(n);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> theta(0.0f, 2.0f * static_cast<float>(M_PI));
    std::uniform_real_distribution<float> h(-height / 2, height / 2);
    std::normal_distribution<float> gauss(0.0f, noise);

    for (int i = 0; i < n; ++i) {
        float t = theta(rng);
        float r = radius + (noise > 0 ? gauss(rng) : 0.0f);
        pts[i] = {r * std::cos(t), r * std::sin(t), h(rng)};
    }

    cloud->addPoints(pts);
    cloud->update();
    return cloud;
}

// ============ 特殊场景生成 ============

OverlappingPair makeOverlappingSpheres(int n, float radius, float overlap,
                                       float angle_deg, unsigned seed) {
    OverlappingPair pair;

    // 生成完整的球体
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> theta(0.0f, 2.0f * static_cast<float>(M_PI));
    std::uniform_real_distribution<float> phi(0.0f, static_cast<float>(M_PI));

    auto source = std::make_shared<ct::Cloud>();
    auto target = std::make_shared<ct::Cloud>();

    // source: 完整球体
    std::vector<ct::PointXYZ> src_pts(n);
    for (int i = 0; i < n; ++i) {
        float t = theta(rng);
        float p = phi(rng);
        src_pts[i] = {
            radius * std::sin(p) * std::cos(t),
            radius * std::sin(p) * std::sin(t),
            radius * std::cos(p)
        };
    }
    source->addPoints(src_pts);
    source->update();

    // 构造旋转变换
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float angle = angle_deg * static_cast<float>(M_PI) / 180.0f;
    transform(0, 0) = std::cos(angle);
    transform(0, 2) = std::sin(angle);
    transform(2, 0) = -std::sin(angle);
    transform(2, 2) = std::cos(angle);

    // target: 旋转后的球体
    auto src_pcl = source->toPCL_XYZ();
    pcl::PointCloud<pcl::PointXYZ> tgt_pcl;
    pcl::transformPointCloud(*src_pcl, tgt_pcl, transform);

    std::vector<ct::PointXYZ> tgt_pts(tgt_pcl.size());
    for (size_t i = 0; i < tgt_pcl.size(); ++i) {
        tgt_pts[i] = {tgt_pcl[i].x, tgt_pcl[i].y, tgt_pcl[i].z};
    }
    target->addPoints(tgt_pts);
    target->update();

    pair.source = source;
    pair.target = target;
    pair.ground_truth_transform = transform;
    return pair;
}

ct::Cloud::Ptr addOutliers(ct::Cloud::Ptr cloud, int n_outliers,
                           float outlier_range, unsigned seed) {
    auto result = cloud->clone();

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist(-outlier_range, outlier_range);

    std::vector<ct::PointXYZ> outliers(n_outliers);
    for (int i = 0; i < n_outliers; ++i) {
        outliers[i] = {dist(rng), dist(rng), dist(rng)};
    }
    result->addPoints(outliers);
    result->update();
    return result;
}

ClusterPair makeTwoClusters(int n1, int n2, float separation,
                            float noise, unsigned seed) {
    ClusterPair pair;
    pair.cloud = std::make_shared<ct::Cloud>();
    pair.labels.resize(n1 + n2);

    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.0f, noise);

    std::vector<ct::PointXYZ> pts(n1 + n2);

    // 聚类 0: 中心在 (-separation/2, 0, 0)
    for (int i = 0; i < n1; ++i) {
        pts[i] = {
            -separation / 2 + gauss(rng),
            gauss(rng),
            gauss(rng)
        };
        pair.labels[i] = 0;
    }

    // 聚类 1: 中心在 (+separation/2, 0, 0)
    for (int i = 0; i < n2; ++i) {
        pts[n1 + i] = {
            separation / 2 + gauss(rng),
            gauss(rng),
            gauss(rng)
        };
        pair.labels[n1 + i] = 1;
    }

    pair.cloud->addPoints(pts);
    pair.cloud->update();
    return pair;
}

ct::Cloud::Ptr applyTransform(ct::Cloud::Ptr cloud, const Eigen::Matrix4f& transform) {
    auto result = std::make_shared<ct::Cloud>();
    auto pcl_cloud = cloud->toPCL_XYZ();
    pcl::PointCloud<pcl::PointXYZ> transformed;
    pcl::transformPointCloud(*pcl_cloud, transformed, transform);

    std::vector<ct::PointXYZ> pts(transformed.size());
    for (size_t i = 0; i < transformed.size(); ++i) {
        pts[i] = {transformed[i].x, transformed[i].y, transformed[i].z};
    }

    result->addPoints(pts);
    result->update();
    return result;
}

// ============ 验证辅助 ============

bool isApproximatelyPlanar(ct::Cloud::Ptr cloud, float max_residual) {
    if (cloud->size() < 3) return true;

    auto pcl_cloud = cloud->toPCL_XYZ();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pcl_cloud, centroid);
    Eigen::Vector3f c(centroid[0], centroid[1], centroid[2]);

    // PCA: 计算协方差矩阵
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& pt : pcl_cloud->points) {
        Eigen::Vector3f v(pt.x - c.x(), pt.y - c.y(), pt.z - c.z());
        cov += v * v.transpose();
    }
    cov /= static_cast<float>(cloud->size());

    // 最小特征值对应法线方向，如果接近0则为平面
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    float min_eigen = solver.eigenvalues().minCoeff();

    // 将最小特征值转换为 RMS 残差并判断
    return std::sqrt(min_eigen) < max_residual;
}

bool isApproximatelySpherical(ct::Cloud::Ptr cloud, float expected_radius,
                              float tolerance) {
    auto centroid = computeCentroid(cloud);
    Eigen::Vector3f center(centroid[0], centroid[1], centroid[2]);

    auto pcl_cloud = cloud->toPCL_XYZ();
    float sum_sq = 0.0f;
    for (const auto& pt : pcl_cloud->points) {
        Eigen::Vector3f v(pt.x, pt.y, pt.z);
        float r = (v - center).norm();
        sum_sq += (r - expected_radius) * (r - expected_radius);
    }
    float rms = std::sqrt(sum_sq / cloud->size());
    return rms < tolerance;
}

Eigen::Vector4f computeCentroid(ct::Cloud::Ptr cloud) {
    auto pcl_cloud = cloud->toPCL_XYZ();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pcl_cloud, centroid);
    return centroid;
}

float computeRMSE(const std::vector<float>& errors) {
    if (errors.empty()) return 0.0f;
    float sum = 0.0f;
    for (float e : errors) sum += e * e;
    return std::sqrt(sum / errors.size());
}

bool matrixApproxEqual(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b,
                       float tolerance) {
    return (a - b).cwiseAbs().maxCoeff() < tolerance;
}

}  // namespace test_helpers
