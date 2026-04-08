//
// Created by LBC on 2026/1/6.
//

#ifndef POINTWORKS_VEGFILTER_H
#define POINTWORKS_VEGFILTER_H

#include "core/cloud.h"
#include "core/exports.h"

#include <functional>
#include <atomic>

namespace ct{
    enum class VegIndexType {
        ExG_ExR = 0,
        ExG,
        NGRDI,
        CIVE
    };

    struct VegResult {
        Cloud::Ptr veg_cloud;
        Cloud::Ptr non_veg_cloud;
        float time_ms = 0;
    };

    class VegetationFilter {
    public:
        static VegResult apply(const Cloud::Ptr& cloud,
                                int index_type, double threshold,
                                std::atomic<bool>* cancel = nullptr,
                                std::function<void(int)> on_progress = nullptr);
    };
}


#endif //POINTWORKS_VEGFILTER_H
