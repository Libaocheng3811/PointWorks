#ifndef CT_MODULES_KEYPOINTS_H
#define CT_MODULES_KEYPOINTS_H

#include "core/exports.h"
#include "core/cloud.h"

#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>

#include <functional>
#include <atomic>

namespace ct
{
    typedef pcl::PointXYZI      PointXYZI;
    typedef pcl::PointXYZRGB    PointXYZRGB;
    typedef pcl::PointWithScale PointWithScale;
    typedef pcl::RangeImage     RangeImage;

    struct KeypointResult {
        Cloud::Ptr cloud;
        float time_ms = 0;
    };

    class Keypoints
    {
    public:
        /**
         * @brief NARF 关键点检测
         */
        static KeypointResult NarfKeypoint(const Cloud::Ptr& cloud,
                                            const RangeImage::Ptr& range_image,
                                            float support_size,
                                            std::atomic<bool>* cancel = nullptr,
                                            std::function<void(int)> on_progress = nullptr);

        /**
         * @brief Harris 3D 关键点检测
         * @param response_method 1-HARRIS 2-NOBLE 3-LOWE 4-TOMASI 5-CURVATURE
         */
        static KeypointResult HarrisKeypoint3D(const Cloud::Ptr& cloud,
                                                int response_method, float threshold,
                                                bool non_maxima, bool do_refine,
                                                int k, double radius,
                                                std::atomic<bool>* cancel = nullptr,
                                                std::function<void(int)> on_progress = nullptr);

        /**
         * @brief ISS 3D 关键点检测
         */
        static KeypointResult ISSKeypoint3D(const Cloud::Ptr& cloud,
                                             double resolution,
                                             double gamma_21, double gamma_32,
                                             int min_neighbors, float angle,
                                             int k, double radius,
                                             std::atomic<bool>* cancel = nullptr,
                                             std::function<void(int)> on_progress = nullptr);

        /**
         * @brief SIFT 3D 关键点检测
         */
        static KeypointResult SIFTKeypoint(const Cloud::Ptr& cloud,
                                            float min_scale, int nr_octaves,
                                            int nr_scales_per_octave, float min_contrast,
                                            int k, double radius,
                                            std::atomic<bool>* cancel = nullptr,
                                            std::function<void(int)> on_progress = nullptr);

        /**
         * @brief Trajkovic 3D 关键点检测
         */
        static KeypointResult TrajkovicKeypoint3D(const Cloud::Ptr& cloud,
                                                    int compute_method, int window_size,
                                                    float first_threshold, float second_threshold,
                                                    int k, double radius,
                                                    std::atomic<bool>* cancel = nullptr,
                                                    std::function<void(int)> on_progress = nullptr);
    };
}  // namespace ct

#endif  // CT_MODULES_KEYPOINTS_H
