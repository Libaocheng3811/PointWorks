//
// Created by LBC on 2026/1/4.
//

#include "csffilter.h"
#include "utils.h"
#include "CSF.h"
#include "point_cloud.h"
#include <pcl/filters/extract_indices.h>

namespace ct{

    CSFResult CSFFilter::apply(const Cloud::Ptr& cloud,
                                bool bSloopSmooth, float time_step, double class_threshold,
                                double cloth_resolution, int rigidness, int iterations,
                                std::atomic<bool>* cancel,
                                std::function<void(int)> on_progress) {
        if (!cloud || cloud->empty()) return {};
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        // 数据转换 PCL Point -> CSF Point
        auto pclCloud = cloud->toPCL_XYZ();
        std::vector<csf::Point> csf_points;
        csf_points.resize(pclCloud->size());  // 使用 resize 而不是 reserve

#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(pclCloud->size()); ++i) {
            const auto& p = pclCloud->points[i];
            csf_points[i].x = p.x;
            csf_points[i].y = p.y;
            csf_points[i].z = p.z;
        }

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(10);

        // 配置参数
        CSF csf;
        csf.setPointCloud(csf_points);
        csf.params.bSloopSmooth = bSloopSmooth;
        csf.params.time_step = time_step;
        csf.params.class_threshold = class_threshold;
        csf.params.cloth_resolution = cloth_resolution;
        csf.params.rigidness = rigidness;
        csf.params.interations = iterations;

        if (cancel && cancel->load()) return {};

        // 执行滤波
        std::vector<int> groundIndexes, offGroundIndexes;
        csf.do_filtering(groundIndexes, offGroundIndexes, true);

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(60);

        // 使用 fromPCL 方法从 PCL 点云构造 Cloud
        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_ground(new pcl::PointCloud<PointXYZRGBN>);
        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_off_ground(new pcl::PointCloud<PointXYZRGBN>);

        pcl_ground->resize(groundIndexes.size());
        pcl_off_ground->resize(offGroundIndexes.size());

        auto pcl_cloud_xyzrgbn = cloud->toPCL_XYZRGBN();

#pragma omp parallel for
        for (int i = 0; i < (int)groundIndexes.size(); ++i) {
            if (groundIndexes[i] >= 0 && groundIndexes[i] < pcl_cloud_xyzrgbn->size()) {
                pcl_ground->points[i] = pcl_cloud_xyzrgbn->points[groundIndexes[i]];
            }
        }

#pragma omp parallel for
        for (int i = 0; i < (int)offGroundIndexes.size(); ++i) {
            if (offGroundIndexes[i] >= 0 && offGroundIndexes[i] < pcl_cloud_xyzrgbn->size()) {
                pcl_off_ground->points[i] = pcl_cloud_xyzrgbn->points[offGroundIndexes[i]];
            }
        }

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(80);

        ct::Cloud::Ptr ground_cloud = Cloud::fromPCL_XYZRGBN(*pcl_ground);
        ground_cloud->setId(cloud->id() + "_ground");
        syncAllScalarFields(cloud, ground_cloud, groundIndexes);

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(90);

        ct::Cloud::Ptr off_ground_cloud = Cloud::fromPCL_XYZRGBN(*pcl_off_ground);
        off_ground_cloud->setId(cloud->id() + "_off_ground");
        syncAllScalarFields(cloud, off_ground_cloud, offGroundIndexes);

        if (cancel && cancel->load()) return {};
        if (on_progress) on_progress(100);

        return {ground_cloud, off_ground_cloud, static_cast<float>(time.toc())};
    }

} // namespace ct
