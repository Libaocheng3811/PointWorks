#include "algorithm/normals.h"

#include <pcl/console/time.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>

namespace ct
{
    static bool isCanceled(std::atomic<bool>* cancel)
    {
        return cancel && cancel->load();
    }

    static void reportProgress(std::function<void(int)> on_progress, int value)
    {
        if (on_progress) on_progress(value);
    }

    static NormalsResult makeErrorResult(const std::string& msg)
    {
        NormalsResult result;
        result.cloud = nullptr;
        result.time_ms = 0;
        result.error_msg = msg;
        return result;
    }

    NormalsResult Normals::estimate(const Cloud::Ptr& cloud,
                                    int k_search, double radius_search,
                                    float vpx, float vpy, float vpz,
                                    bool reverse,
                                    std::atomic<bool>* cancel,
                                    std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        if (!cloud || cloud->empty())
            return makeErrorResult("Input cloud is null or empty.");

        if (k_search == 0 && radius_search == 0)
            return makeErrorResult("k_search and radius_search cannot both be zero.");

        try
        {
            if (isCanceled(cancel)) return makeErrorResult("Canceled.");

            auto pcl_cloud = cloud->toPCL_XYZRGBN();
            pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
            tree->setInputCloud(pcl_cloud);

            pcl::NormalEstimationOMP<PointXYZRGBN, PointNormal> ne;
            ne.setInputCloud(pcl_cloud);
            ne.setSearchMethod(tree);
            ne.setViewPoint(vpx, vpy, vpz);

            if (k_search > 0) ne.setKSearch(k_search);
            if (radius_search > 0) ne.setRadiusSearch(radius_search);

            reportProgress(on_progress, 10);

            pcl::PointCloud<PointNormal>::Ptr normals(new pcl::PointCloud<PointNormal>);
            ne.compute(*normals);

            if (isCanceled(cancel)) return makeErrorResult("Canceled.");

            reportProgress(on_progress, 80);

            // 将法线合并到原始点云
            pcl::PointCloud<PointXYZRGBN>::Ptr result_cloud(new pcl::PointCloud<PointXYZRGBN>);
            result_cloud->width = pcl_cloud->width;
            result_cloud->height = pcl_cloud->height;
            result_cloud->is_dense = pcl_cloud->is_dense;
            result_cloud->points.resize(pcl_cloud->size());

            for (size_t i = 0; i < pcl_cloud->size(); ++i)
            {
                result_cloud->points[i].x = pcl_cloud->points[i].x;
                result_cloud->points[i].y = pcl_cloud->points[i].y;
                result_cloud->points[i].z = pcl_cloud->points[i].z;
                result_cloud->points[i].r = pcl_cloud->points[i].r;
                result_cloud->points[i].g = pcl_cloud->points[i].g;
                result_cloud->points[i].b = pcl_cloud->points[i].b;
                if (i < normals->size())
                {
                    float sign = reverse ? -1.0f : 1.0f;
                    result_cloud->points[i].normal_x = normals->points[i].normal_x * sign;
                    result_cloud->points[i].normal_y = normals->points[i].normal_y * sign;
                    result_cloud->points[i].normal_z = normals->points[i].normal_z * sign;
                    result_cloud->points[i].curvature = normals->points[i].curvature;
                }
            }

            reportProgress(on_progress, 90);

            // 转回 ct::Cloud
            auto result = Cloud::fromPCL_XYZRGBN(*result_cloud);
            result->setId(cloud->id());
            result->setFilepath(cloud->filepath());
            result->setBox(cloud->box());

            reportProgress(on_progress, 100);

            NormalsResult nr;
            nr.cloud = result;
            nr.time_ms = time.toc();
            return nr;
        }
        catch (const std::exception& e)
        {
            return makeErrorResult(std::string("Exception: ") + e.what());
        }
        catch (...)
        {
            return makeErrorResult("Unknown exception occurred.");
        }
    }

} // namespace ct
