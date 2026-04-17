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

namespace ct {

    // Closest Point Set result
    struct CPSResult {
        Cloud::Ptr projected_cloud;
        float time_ms = 0;
        bool success = true;
        std::string error_msg;
    };

    class DistanceCalculator {
    public:
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

} // namespace ct

#endif //POINTWORKS_DISTANCECALCULATOR_H
