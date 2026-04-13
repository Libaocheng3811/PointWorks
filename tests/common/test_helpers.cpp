#include "test_helpers.h"

#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <numeric>

namespace test_helpers {

// ============ 内部辅助 ============

namespace {

Eigen::Vector3f randomOnSphere(std::mt19937& rng) {
    std::uniform_real_distribution<float> z_dist(-1.0f, 1.0f);
    std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * static_cast<float>(M_PI));

    float z = z_dist(rng);
    float r = std::sqrt(1.0f - z * z);
    float theta = angle_dist(rng);

    return Eigen::Vector3f(r * std::cos(theta), r * std::sin(theta), z);
}

}  // anonymous namespace

// ============ 基础几何体 ============

ct::Cloud::Ptr makePlane(int n, float width, float depth, float noise, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> x_dist(-width / 2, width / 2);
    std::uniform_real_distribution<float> y_dist(-depth / 2, depth / 2);
    std::normal_distribution<float> noise_dist(0.0f, noise);

    std::vector<ct::PointXYZ> pts;
    pts.reserve(n);

    for (int i = 0; i < n; ++i) {
        ct::PointXYZ pt;
        pt.x = x_dist(rng);
        pt.y = y_dist(rng);
        pt.z = noise > 0 ? noise_dist(rng) : 0.0f;
        pts.push_back(pt);
    }

    auto cloud = std::make_shared<ct::Cloud>();
    cloud->addPoints(pts);
    cloud->update();
    return cloud;
}

ct::Cloud::Ptr makeSphere(int n, float radius, float noise, unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise_dist(0.0f, noise);

    std::vector<ct::PointXYZ> pts;
    pts.reserve(n);

    for (int i = 0; i < n; ++i) {
        auto dir = randomOnSphere(rng);
        ct::PointXYZ pt;
        pt.x = dir.x() * radius + (noise > 0 ? noise_dist(rng) : 0.0f);
        pt.y = dir.y() * radius + (noise > 0 ? noise_dist(rng) : 0.0f);
        pt.z = dir.z() * radius + (noise > 0 ? noise_dist(rng) : 0.0f);
        pts.push_back(pt);
    }

    auto cloud = std::make_shared<ct::Cloud>();
    cloud->addPoints(pts);
    cloud->update();
    return cloud;
}

ct::Cloud::Ptr makeCube(int n, float size, float noise, unsigned seed) {
    std::mt19937 rng(seed);
    float half = size / 2;

    // 每个面分配等量的点
    int points_per_face = n / 6;
    int remainder = n - points_per_face * 6;

    // 各面的法线和偏移
    // face: 固定坐标轴值, 另外两个轴随机
    struct FaceInfo {
        int fixed_axis;    // 0=x, 1=y, 2=z
        float fixed_val;
    };
    std::vector<FaceInfo> faces = {
        {0,  half}, {0, -half},
        {1,  half}, {1, -half},
        {2,  half}, {2, -half}
    };

    std::uniform_real_distribution<float> u_dist(-half, half);
    std::normal_distribution<float> noise_dist(0.0f, noise);

    std::vector<ct::PointXYZ> pts;
    pts.reserve(n);

    for (int f = 0; f < 6; ++f) {
        int count = points_per_face + (f == 0 ? remainder : 0);
        auto& face = faces[f];
        for (int i = 0; i < count; ++i) {
            ct::PointXYZ pt;
            float vals[3];
            vals[face.fixed_axis] = face.fixed_val + (noise > 0 ? noise_dist(rng) : 0.0f);
            int axes[2];
            int idx = 0;
            for (int a = 0; a < 3; ++a) {
                if (a != face.fixed_axis) axes[idx++] = a;
            }
            vals[axes[0]] = u_dist(rng);
            vals[axes[1]] = u_dist(rng);
            pt.x = vals[0];
            pt.y = vals[1];
            pt.z = vals[2];
            pts.push_back(pt);
        }
    }

    auto cloud = std::make_shared<ct::Cloud>();
    cloud->addPoints(pts);
    cloud->update();
    return cloud;
}

ct::Cloud::Ptr makeCylinder(int n, float radius, float height, float noise, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * static_cast<float>(M_PI));
    std::uniform_real_distribution<float> z_dist(-height / 2, height / 2);
    std::normal_distribution<float> noise_dist(0.0f, noise);

    std::vector<ct::PointXYZ> pts;
    pts.reserve(n);

    for (int i = 0; i < n; ++i) {
        float angle = angle_dist(rng);
        float r = radius + (noise > 0 ? noise_dist(rng) : 0.0f);
        ct::PointXYZ pt;
        pt.x = r * std::cos(angle);
        pt.y = r * std::sin(angle);
        pt.z = z_dist(rng);
        pts.push_back(pt);
    }

    auto cloud = std::make_shared<ct::Cloud>();
    cloud->addPoints(pts);
    cloud->update();
    return cloud;
}

// ============ 特殊场景 ============

OverlappingPair makeOverlappingSpheres(int n, float radius, float overlap, float angle_deg, unsigned seed) {
    // 先生成完整的球面点云
    auto full_cloud = makeSphere(n, radius, 0.0f, seed);
    auto pcl_cloud = full_cloud->toPCL_XYZ();

    // 将点云分为两部分：重叠部分和各自独有的部分
    int overlap_count = static_cast<int>(n * overlap);
    int unique_count = n - overlap_count;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_pts(new pcl::PointCloud<pcl::PointXYZ>);

    source_pts->points.reserve(n);
    target_pts->points.reserve(n);

    // 重叠部分：两份都有
    for (int i = 0; i < overlap_count; ++i) {
        source_pts->points.push_back(pcl_cloud->points[i]);
        target_pts->points.push_back(pcl_cloud->points[i]);
    }

    // 各自独有的部分
    std::mt19937 rng_unique(seed + 12345);
    for (int i = overlap_count; i < overlap_count + unique_count; ++i) {
        source_pts->points.push_back(pcl_cloud->points[i]);
    }
    for (int i = overlap_count + unique_count; i < 2 * overlap_count + unique_count && i < (int)pcl_cloud->size(); ++i) {
        target_pts->points.push_back(pcl_cloud->points[i]);
    }

    // 构造 source（施加变换）
    float angle = angle_deg * static_cast<float>(M_PI) / 180.0f;
    Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(0.5f, 0.3f, 0.0f);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = (translation * rotation).rotation();
    transform.block<3, 1>(0, 3) = (translation * rotation).translation();

    pcl::PointCloud<pcl::PointXYZ> transformed;
    pcl::transformPointCloud(*source_pts, transformed, transform);

    std::vector<ct::PointXYZ> src_pts, tgt_pts;
    src_pts.reserve(transformed.size());
    tgt_pts.reserve(target_pts->size());
    for (const auto& p : transformed.points) src_pts.push_back({p.x, p.y, p.z});
    for (const auto& p : target_pts->points) tgt_pts.push_back({p.x, p.y, p.z});

    auto source_cloud = std::make_shared<ct::Cloud>();
    source_cloud->addPoints(src_pts);
    source_cloud->update();

    auto target_cloud = std::make_shared<ct::Cloud>();
    target_cloud->addPoints(tgt_pts);
    target_cloud->update();

    return {source_cloud, target_cloud, transform};
}

ct::Cloud::Ptr addOutliers(ct::Cloud::Ptr cloud, int n_outliers, float outlier_range, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist(-outlier_range, outlier_range);

    std::vector<ct::PointXYZ> outliers;
    outliers.reserve(n_outliers);

    for (int i = 0; i < n_outliers; ++i) {
        ct::PointXYZ pt;
        pt.x = dist(rng);
        pt.y = dist(rng);
        pt.z = dist(rng);
        outliers.push_back(pt);
    }

    auto result = std::make_shared<ct::Cloud>();
    result->addPoints(outliers);

    // 将原始点云追加到结果中
    auto orig_pcl = cloud->toPCL_XYZ();
    std::vector<ct::PointXYZ> orig_pts;
    orig_pts.reserve(orig_pcl->size());
    for (const auto& p : orig_pcl->points) orig_pts.push_back({p.x, p.y, p.z});
    result->addPoints(orig_pts);
    result->update();
    return result;
}

ClusterPair makeTwoClusters(int n1, int n2, float separation, float noise, unsigned seed) {
    // 两个聚类中心沿 X 轴分隔
    float center1 = -separation / 2;
    float center2 = separation / 2;

    std::mt19937 rng(seed);
    std::normal_distribution<float> noise_dist(0.0f, noise);
    std::uniform_real_distribution<float> xy_dist(-1.0f, 1.0f);

    ClusterPair pair;
    pair.cloud = std::make_shared<ct::Cloud>();
    pair.labels.reserve(n1 + n2);

    std::vector<ct::PointXYZ> all_pts;
    all_pts.reserve(n1 + n2);

    // 簇 1
    for (int i = 0; i < n1; ++i) {
        ct::PointXYZ pt;
        pt.x = center1 + xy_dist(rng) + (noise > 0 ? noise_dist(rng) : 0.0f);
        pt.y = xy_dist(rng) + (noise > 0 ? noise_dist(rng) : 0.0f);
        pt.z = xy_dist(rng) + (noise > 0 ? noise_dist(rng) : 0.0f);
        all_pts.push_back(pt);
        pair.labels.push_back(0);
    }

    // 簇 2
    for (int i = 0; i < n2; ++i) {
        ct::PointXYZ pt;
        pt.x = center2 + xy_dist(rng) + (noise > 0 ? noise_dist(rng) : 0.0f);
        pt.y = xy_dist(rng) + (noise > 0 ? noise_dist(rng) : 0.0f);
        pt.z = xy_dist(rng) + (noise > 0 ? noise_dist(rng) : 0.0f);
        all_pts.push_back(pt);
        pair.labels.push_back(1);
    }

    pair.cloud->addPoints(all_pts);
    pair.cloud->update();
    return pair;
}

ct::Cloud::Ptr applyTransform(ct::Cloud::Ptr cloud, const Eigen::Matrix4f& transform) {
    auto pcl_cloud = cloud->toPCL_XYZ();
    pcl::PointCloud<pcl::PointXYZ> transformed;
    pcl::transformPointCloud(*pcl_cloud, transformed, transform);

    std::vector<ct::PointXYZ> pts;
    pts.reserve(transformed.size());
    for (const auto& p : transformed.points) pts.push_back({p.x, p.y, p.z});

    auto result = std::make_shared<ct::Cloud>();
    result->addPoints(pts);
    result->update();
    return result;
}

// ============ 验证辅助 ============

bool isApproximatelyPlanar(ct::Cloud::Ptr cloud, float max_residual) {
    auto pcl_cloud = cloud->toPCL_XYZ();
    if (pcl_cloud->empty()) return false;

    // 使用 PCA 简化平面拟合：SVD 拟合
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pcl_cloud, centroid);

    // 构建协方差矩阵
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*pcl_cloud, centroid, covariance);

    // 特征分解，最小特征值对应的特征向量就是法线
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance);
    Eigen::Vector3f normal = eigensolver.eigenvectors().col(0);

    // 计算每个点到平面的距离
    float sum_sq = 0.0f;
    for (const auto& pt : pcl_cloud->points) {
        Eigen::Vector3f v(pt.x - centroid[0], pt.y - centroid[1], pt.z - centroid[2]);
        float dist = std::abs(v.dot(normal));
        sum_sq += dist * dist;
    }

    float rms = std::sqrt(sum_sq / pcl_cloud->size());
    return rms <= max_residual;
}

bool isApproximatelySpherical(ct::Cloud::Ptr cloud, float expected_radius, float tolerance) {
    auto pcl_cloud = cloud->toPCL_XYZ();
    if (pcl_cloud->empty()) return false;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pcl_cloud, centroid);

    float sum_sq_error = 0.0f;
    for (const auto& pt : pcl_cloud->points) {
        float dx = pt.x - centroid[0];
        float dy = pt.y - centroid[1];
        float dz = pt.z - centroid[2];
        float r = std::sqrt(dx * dx + dy * dy + dz * dz);
        float err = r - expected_radius;
        sum_sq_error += err * err;
    }

    float rms = std::sqrt(sum_sq_error / pcl_cloud->size());
    return rms <= tolerance;
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

bool matrixApproxEqual(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b, float tolerance) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (std::abs(a(i, j) - b(i, j)) > tolerance) return false;
        }
    }
    return true;
}

}  // namespace test_helpers
