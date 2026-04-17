//
// Created by LBC on 2024/12/26.
//

#ifndef MODULES_FILTERS_H
#define MODULES_FILTERS_H

#include "core/exports.h"
#include "core/cloud.h"

#include <functional>
#include <atomic>

#include <pcl/Vertices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/surface/mls.h>

namespace ct
{
    typedef pcl::ConditionBase<PointXYZRGBN>    ConditionBase;
    typedef pcl::ConditionAnd<PointXYZRGBN>     ConditionAnd;
    typedef pcl::ConditionOr<PointXYZRGBN>      ConditionOr;
    typedef pcl::FieldComparison<PointXYZRGBN>  FieldComparison;
    typedef pcl::ComparisonOps::CompareOp       CompareOp;

    struct FilterResult {
        Cloud::Ptr result_cloud;
        float time_ms = 0;
    };

    class Filters
    {
    public:
        /* filter */
        static FilterResult PassThrough(const Cloud::Ptr& cloud,
                                        const std::string& field_name,
                                        float limit_min, float limit_max,
                                        bool negative = false,
                                        std::atomic<bool>* cancel = nullptr,
                                        std::function<void(int)> on_progress = nullptr);

        static FilterResult VoxelGrid(const Cloud::Ptr& cloud,
                                       float lx, float ly, float lz,
                                       bool negative = false,
                                       std::atomic<bool>* cancel = nullptr,
                                       std::function<void(int)> on_progress = nullptr);

        static FilterResult ApproximateVoxelGrid(const Cloud::Ptr& cloud,
                                                  float lx, float ly, float lz,
                                                  bool negative = false,
                                                  std::atomic<bool>* cancel = nullptr,
                                                  std::function<void(int)> on_progress = nullptr);

        static FilterResult StatisticalOutlierRemoval(const Cloud::Ptr& cloud,
                                                       int nr_k, double stddev_mult,
                                                       bool negative = false,
                                                       std::atomic<bool>* cancel = nullptr,
                                                       std::function<void(int)> on_progress = nullptr);

        static FilterResult RadiusOutlierRemoval(const Cloud::Ptr& cloud,
                                                  double radius, int min_pts,
                                                  bool negative = false,
                                                  std::atomic<bool>* cancel = nullptr,
                                                  std::function<void(int)> on_progress = nullptr);

        static FilterResult ConditionalRemoval(const Cloud::Ptr& cloud,
                                                ConditionBase::Ptr con,
                                                bool negative = false,
                                                std::atomic<bool>* cancel = nullptr,
                                                std::function<void(int)> on_progress = nullptr);

        static FilterResult GridMinimun(const Cloud::Ptr& cloud,
                                         float resolution,
                                         bool negative = false,
                                         std::atomic<bool>* cancel = nullptr,
                                         std::function<void(int)> on_progress = nullptr);

        static FilterResult LocalMaximum(const Cloud::Ptr& cloud,
                                          float radius,
                                          bool negative = false,
                                          std::atomic<bool>* cancel = nullptr,
                                          std::function<void(int)> on_progress = nullptr);

        static FilterResult ShadowPoints(const Cloud::Ptr& cloud,
                                          float threshold,
                                          bool negative = false,
                                          std::atomic<bool>* cancel = nullptr,
                                          std::function<void(int)> on_progress = nullptr);

        /* sample */
        static FilterResult DownSampling(const Cloud::Ptr& cloud,
                                          float radius,
                                          bool negative = false,
                                          std::atomic<bool>* cancel = nullptr,
                                          std::function<void(int)> on_progress = nullptr);

        static FilterResult UniformSampling(const Cloud::Ptr& cloud,
                                              float radius,
                                              bool negative = false,
                                              std::atomic<bool>* cancel = nullptr,
                                              std::function<void(int)> on_progress = nullptr);

        static FilterResult RandomSampling(const Cloud::Ptr& cloud,
                                            int sample, int seed,
                                            bool negative = false,
                                            std::atomic<bool>* cancel = nullptr,
                                            std::function<void(int)> on_progress = nullptr);

        static FilterResult ReSampling(const Cloud::Ptr& cloud,
                                         float radius, int polynomial_order,
                                         bool negative = false,
                                         std::atomic<bool>* cancel = nullptr,
                                         std::function<void(int)> on_progress = nullptr);

        static FilterResult SamplingSurfaceNormal(const Cloud::Ptr& cloud,
                                                    int sample, int seed, float ratio,
                                                    bool negative = false,
                                                    std::atomic<bool>* cancel = nullptr,
                                                    std::function<void(int)> on_progress = nullptr);

        static FilterResult NormalSpaceSampling(const Cloud::Ptr& cloud,
                                                  int sample, int seed, int bin,
                                                  bool negative = false,
                                                  std::atomic<bool>* cancel = nullptr,
                                                  std::function<void(int)> on_progress = nullptr);
    };
}


#endif //MODULES_FILTERS_H
