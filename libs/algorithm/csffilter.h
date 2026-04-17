//
// Created by LBC on 2026/1/4.
//

#ifndef POINTWORKS_CSFFILTER_H
#define POINTWORKS_CSFFILTER_H

#include "core/exports.h"
#include "core/cloud.h"

#include <functional>
#include <atomic>

namespace ct{

    struct CSFResult {
        Cloud::Ptr ground_cloud;
        Cloud::Ptr off_ground_cloud;
        float time_ms = 0;
    };

    class CSFFilter {
    public:
        static CSFResult apply(const Cloud::Ptr& cloud,
                                bool bSloopSmooth, float time_step, double class_threshold,
                                double cloth_resolution, int rigidness, int iterations,
                                std::atomic<bool>* cancel = nullptr,
                                std::function<void(int)> on_progress = nullptr);
    };
} // namespace ct


#endif //POINTWORKS_CSFFILTER_H
