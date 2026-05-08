//
// Distance calculator module — supports C2C, C2M, C2P, and Closest Point Set.
//

#ifndef POINTWORKS_DISTANCECALCULATOR_H
#define POINTWORKS_DISTANCECALCULATOR_H

#include <vector>
#include <functional>
#include <atomic>
#include <string>
#include "core/cloud.h"
#include "core/exports.h"
#include "core/field_types.h"
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pw {

    // M3C2 result
    struct M3C2Result {
        std::vector<float> signed_distances;  // 有符号距离 (per core point)
        std::vector<float> lod_values;        // LOD 置信区间 (compute_lod=true)
        std::vector<float> normals_quality;   // 法线质量 (0~1)
        std::vector<size_t> core_indices;     // Core point indices in reference cloud
        float time_ms = 0;
        bool success = true;
        std::string error_msg;
    };

    // Closest Point Set result
    struct CPSResult {
        Cloud::Ptr projected_cloud;
        float time_ms = 0;
        bool success = true;
        std::string error_msg;
    };

    class DistanceCalculator {
    public:
        // M3C2: Multiscale Model to Model Cloud Comparison
        // Accepts pre-extracted PCL data — caller must extract on main thread for thread safety
        static M3C2Result calculateM3C2(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& refCloud,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& compCloud,
            const pcl::PointCloud<pcl::Normal>::Ptr& existingNormals,
            const M3C2Params& params,
            std::atomic<bool>* cancel = nullptr,
            std::function<void(int)> on_progress = nullptr);

        // C2C: Cloud-to-Cloud distance
        static DistanceResult calculateC2C(const Cloud::Ptr& ref, const Cloud::Ptr& comp,
                                           const C2CParams& params,
                                           std::atomic<bool>* cancel = nullptr,
                                           std::function<void(int)> on_progress = nullptr);

        // C2M: Cloud-to-Mesh signed distance
        static DistanceResult calculateC2M(const Cloud::Ptr& source,
                                           const pcl::PolygonMesh::Ptr& target_mesh,
                                           const C2MParams& params,
                                           std::atomic<bool>* cancel = nullptr,
                                           std::function<void(int)> on_progress = nullptr);

        // C2P: Cloud-to-Primitive distance
        static DistanceResult calculateC2P(const Cloud::Ptr& source,
                                           const C2PParams& params,
                                           std::atomic<bool>* cancel = nullptr,
                                           std::function<void(int)> on_progress = nullptr);

        // Closest Point Set extraction
        static CPSResult extractClosestPoints(const Cloud::Ptr& source,
                                              const Cloud::Ptr& target,
                                              const CPSParams& params,
                                              std::atomic<bool>* cancel = nullptr,
                                              std::function<void(int)> on_progress = nullptr);

        // DEPRECATED: Backward-compatible interface for ChangeDetectPlugin.
        // Internally forwards to calculateC2C().
        static DistanceResult calculate(const Cloud::Ptr& ref, const Cloud::Ptr& comp,
                                         const DistanceParams& params,
                                         std::atomic<bool>* cancel = nullptr,
                                         std::function<void(int)> on_progress = nullptr);
    };

} // namespace pw

#endif //POINTWORKS_DISTANCECALCULATOR_H
