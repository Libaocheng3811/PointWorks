//
// Created by LBC on 2026/1/6.
//

#include "vegfilter.h"
#include "utils.h"
#include <pcl/filters/extract_indices.h>
#include <omp.h>

namespace ct{

    static double calculateOtsuThreshold(const std::vector<float> &values) {
        if (values.empty()) return 0.0;

        // 1. 找到最大最小值以构建直方图
        float min_val = values[0];
        float max_val = values[0];
        for (float v : values) {
            if (v < min_val) min_val = v;
            if (v > max_val) max_val = v;
        }

        // 如果所有值都一样，返回该值
        if (min_val == max_val) return min_val;

        // 2. 构建直方图 (例如 256 个 bin)
        const int num_bins = 256;
        std::vector<int> histogram(num_bins, 0);
        float range = max_val - min_val;

        for (float v : values) {
            int bin = static_cast<int>((v - min_val) / range * (num_bins - 1));
            histogram[bin]++;
        }

        // 3. Otsu 算法核心
        int total = static_cast<int>(values.size());
        double sum = 0;
        for (int i = 0; i < num_bins; ++i) sum += i * histogram[i];

        double sumB = 0;
        int wB = 0;
        int wF = 0;

        double varMax = 0;
        int threshold_bin = 0;

        for (int i = 0; i < num_bins; ++i) {
            wB += histogram[i];
            if (wB == 0) continue;

            wF = total - wB;
            if (wF == 0) break;

            sumB += (double)(i * histogram[i]);

            double mB = sumB / wB;
            double mF = (sum - sumB) / wF;

            // 类间方差
            double varBetween = (double)wB * (double)wF * (mB - mF) * (mB - mF);

            if (varBetween > varMax) {
                varMax = varBetween;
                threshold_bin = i;
            }
        }

        // 4. 将 bin 映射回原始浮点值
        return min_val + (static_cast<float>(threshold_bin) / (num_bins - 1)) * range;
    }

    VegResult VegetationFilter::apply(const Cloud::Ptr& cloud, int index_type, double threshold,
                                        std::atomic<bool>* cancel,
                                        std::function<void(int)> on_progress) {
        if (!cloud || cloud->empty()) return {};
        if (cancel) cancel->store(false);

        TicToc time; time.tic();

        auto pcl_cloud = cloud->toPCL_XYZRGB();

        size_t num = pcl_cloud->size();
        std::vector<int> veg_indices, non_veg_indices;
        std::vector<float> index_values(num);

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(10);

        // 并行计算植被指数
#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(num); ++i){
            if (cancel && cancel->load()) continue;

            const auto& p = pcl_cloud->points[i];

            //归一化
            float R = p.r, G = p.g, B = p.b;
            float sum = R + G + B;
            float r = (sum > 0) ? R / sum : 0.0f;
            float g = (sum > 0) ? G / sum : 0.0f;
            float b = (sum > 0) ? B / sum : 0.0f;

            float res = 0;
            switch (static_cast<VegIndexType>(index_type)) {
                case VegIndexType::ExG_ExR:
                    res = 3.0f * g - 2.4f * r - b;
                    break;
                case VegIndexType::ExG:
                    res = 2.0f * g - r - b;
                    break;
                case VegIndexType::NGRDI:
                    res = (g + r > 0) ? (g - r) / (g + r) : 0.0f;
                    break;
                case VegIndexType::CIVE:
                    res = 0.441f * r - 0.811f * g + 0.385f * b + 18.787f / 255.0f;
                    break;
            }
            index_values[i] = res;
        }

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(30);

        double actual_threshold = threshold;
        if (static_cast<VegIndexType>(index_type) == VegIndexType::CIVE){
            actual_threshold = calculateOtsuThreshold(index_values);
        }

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(40);

        std::vector<bool> is_veg(num, false);
#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(num); ++i){
            if (cancel && cancel->load()) continue;
            float val = index_values[i];

            if (static_cast<VegIndexType>(index_type) == VegIndexType::CIVE){
                // CIVE: 值越小越像植被
                if (val < actual_threshold) is_veg[i] = true;
            }
            else {
                // ExG, ExR, NGRDI: 值越大越像植被
                if (val >= actual_threshold) is_veg[i] = true;
            }
        }

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(50);

        for (int i = 0; i < static_cast<int>(num); ++i){
            if (is_veg[i]) veg_indices.push_back(i);
            else non_veg_indices.push_back(i);
        }
        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(60);

        // 使用 PCL ExtractIndices 提取点
        auto pcl_cloud_xyzrgbn = cloud->toPCL_XYZRGBN();
        pcl::ExtractIndices<PointXYZRGBN> extract;
        extract.setInputCloud(pcl_cloud_xyzrgbn);

        // 提取植被点
        ct::Cloud::Ptr veg_cloud(new Cloud);
        veg_cloud->setId(cloud->id() + "_veg");
        if (!veg_indices.empty()){
            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_veg(new pcl::PointCloud<PointXYZRGBN>);
            pcl::PointIndices::Ptr v_idx_ptr(new pcl::PointIndices);
            v_idx_ptr->indices = veg_indices;
            extract.setIndices(v_idx_ptr);
            extract.setNegative(false);
            extract.filter(*pcl_veg);
            veg_cloud = Cloud::fromPCL_XYZRGBN(*pcl_veg);
            // 同步自定义字段
            syncAllScalarFields(cloud, veg_cloud, veg_indices);
        }

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(80);

        // 提取非植被点
        ct::Cloud::Ptr non_veg_cloud(new Cloud);
        non_veg_cloud->setId(cloud->id() + "_non_veg");
        if (!non_veg_indices.empty()) {
            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_non_veg(new pcl::PointCloud<PointXYZRGBN>);
            pcl::PointIndices::Ptr nv_idx_ptr(new pcl::PointIndices);
            nv_idx_ptr->indices = non_veg_indices;
            extract.setIndices(nv_idx_ptr);
            extract.setNegative(false);
            extract.filter(*pcl_non_veg);
            non_veg_cloud = Cloud::fromPCL_XYZRGBN(*pcl_non_veg);
            // 同步所有自定义字段
            syncAllScalarFields(cloud, non_veg_cloud, non_veg_indices);
        }

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(100);

        return {veg_cloud, non_veg_cloud, static_cast<float>(time.toc())};
    }

} // namespace ct
