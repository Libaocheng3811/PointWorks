#include <cmath>

#include "algorithm/surface.h"

#include <pcl/console/time.h>
#include <pcl/common/angles.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/impl/poisson.hpp>
#include <pcl/surface/impl/marching_cubes_rbf.hpp>
#include <pcl/surface/impl/marching_cubes_hoppe.hpp>

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

    static SurfaceResult makeErrorResult(const std::string& msg)
    {
        SurfaceResult result;
        result.mesh = nullptr;
        result.time_ms = 0;
        result.error_msg = msg;
        return result;
    }

    SurfaceResult Surface::EarClipping(PolygonMesh::Ptr surface,
                                       std::atomic<bool>* cancel,
                                       std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        SurfaceResult result;
        if (isCanceled(cancel)) return result;
        if (!surface || surface->polygons.empty())
            return makeErrorResult("EarClipping requires a polygon mesh with faces.");

        PolygonMesh::Ptr mesh(new PolygonMesh);

        pcl::EarClipping sur;
        sur.setInputMesh(surface);
        sur.process(*mesh);

        result.mesh = mesh;
        result.time_ms = static_cast<float>(time.toc());
        return result;
    }

    SurfaceResult Surface::GreedyProjectionTriangulation(const Cloud::Ptr& cloud,
                                                          double mu, int nnn, double radius, double min,
                                                          double max, double ep, bool consistent,
                                                          bool consistent_ordering,
                                                          std::atomic<bool>* cancel,
                                                          std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        SurfaceResult result;
        if (isCanceled(cancel)) return result;
        if (!cloud->hasNormals()) return makeErrorResult("GreedyProjectionTriangulation requires normals. Please estimate normals first.");

        reportProgress(on_progress, 10);
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        reportProgress(on_progress, 30);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        PolygonMesh::Ptr mesh(new PolygonMesh);

        pcl::GreedyProjectionTriangulation<PointXYZRGBN> gp3;
        gp3.setInputCloud(pcl_cloud);
        gp3.setSearchMethod(tree);
        gp3.setMu(mu);
        gp3.setMaximumNearestNeighbors(nnn);
        gp3.setMinimumAngle(pcl::deg2rad(min));
        gp3.setMaximumAngle(pcl::deg2rad(max));
        gp3.setMaximumSurfaceAngle(pcl::deg2rad(ep));
        gp3.setSearchRadius(radius);
        gp3.setNormalConsistency(consistent);
        gp3.setConsistentVertexOrdering(consistent_ordering);
        gp3.reconstruct(*mesh);

        reportProgress(on_progress, 100);

        result.mesh = mesh;
        result.time_ms = static_cast<float>(time.toc());
        return result;
    }

    SurfaceResult Surface::GridProjection(const Cloud::Ptr& cloud,
                                          double resolution, int padding_size, int k, int max_binary_search_level,
                                          std::atomic<bool>* cancel,
                                          std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        SurfaceResult result;
        if (isCanceled(cancel)) return result;
        if (!cloud->hasNormals()) return makeErrorResult("GridProjection requires normals. Please estimate normals first.");

        reportProgress(on_progress, 10);
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        reportProgress(on_progress, 30);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        PolygonMesh::Ptr mesh(new PolygonMesh);

        pcl::GridProjection<PointXYZRGBN> gp;
        gp.setInputCloud(pcl_cloud);
        gp.setSearchMethod(tree);
        gp.setResolution(resolution);
        gp.setPaddingSize(padding_size);
        gp.setNearestNeighborNum(k);
        gp.setMaxBinarySearchLevel(max_binary_search_level);
        gp.reconstruct(*mesh);

        reportProgress(on_progress, 100);

        result.mesh = mesh;
        result.time_ms = static_cast<float>(time.toc());
        return result;
    }

    SurfaceResult Surface::Poisson(const Cloud::Ptr& cloud,
                                   int depth, int min_depth, float point_weight, float scale,
                                   int solver_divide, int iso_divide, float samples_per_node,
                                   bool confidence, bool output_polygons, bool manifold,
                                   std::atomic<bool>* cancel,
                                   std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        SurfaceResult result;
        if (isCanceled(cancel)) return result;
        if (!cloud->hasNormals()) return makeErrorResult("Poisson requires normals. Please estimate normals first.");

        reportProgress(on_progress, 10);
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        reportProgress(on_progress, 30);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        PolygonMesh::Ptr mesh(new PolygonMesh);

        pcl::Poisson<PointXYZRGBN> po;
        po.setInputCloud(pcl_cloud);
        po.setSearchMethod(tree);
        po.setDepth(depth);
        po.setMinDepth(min_depth);
        po.setPointWeight(point_weight);
        po.setScale(scale);
        po.setSolverDivide(solver_divide);
        po.setIsoDivide(iso_divide);
        po.setSamplesPerNode(samples_per_node);
        po.setConfidence(confidence);
        po.setOutputPolygons(output_polygons);
        po.setManifold(manifold);
        po.reconstruct(*mesh);

        reportProgress(on_progress, 100);

        result.mesh = mesh;
        result.time_ms = static_cast<float>(time.toc());
        return result;
    }

    SurfaceResult Surface::MarchingCubesRBF(const Cloud::Ptr& cloud,
                                            float iso_level, int res_x, int res_y, int res_z,
                                            float percentage, float epsilon,
                                            std::atomic<bool>* cancel,
                                            std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        SurfaceResult result;
        if (isCanceled(cancel)) return result;
        if (!cloud->hasNormals()) return makeErrorResult("MarchingCubesRBF requires normals. Please estimate normals first.");

        reportProgress(on_progress, 10);
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        reportProgress(on_progress, 30);

        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        PolygonMesh::Ptr mesh(new PolygonMesh);

        pcl::MarchingCubesRBF<PointXYZRGBN> gp;
        gp.setInputCloud(pcl_cloud);
        gp.setSearchMethod(tree);
        gp.setIsoLevel(iso_level);
        gp.setGridResolution(res_x, res_y, res_z);
        gp.setPercentageExtendGrid(percentage);
        gp.setOffSurfaceDisplacement(epsilon);

        reportProgress(on_progress, 40);
        gp.reconstruct(*mesh);

        reportProgress(on_progress, 100);

        result.mesh = mesh;
        result.time_ms = static_cast<float>(time.toc());
        return result;
    }

    SurfaceResult Surface::MarchingCubesHoppe(const Cloud::Ptr& cloud,
                                              float iso_level, int res_x, int res_y, int res_z,
                                              float percentage, float dist_ignore,
                                              std::atomic<bool>* cancel,
                                              std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        SurfaceResult result;
        if (isCanceled(cancel)) return result;
        if (!cloud->hasNormals()) return makeErrorResult("MarchingCubesHoppe requires normals. Please estimate normals first.");

        reportProgress(on_progress, 10);
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        reportProgress(on_progress, 30);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        PolygonMesh::Ptr mesh(new PolygonMesh);

        pcl::MarchingCubesHoppe<PointXYZRGBN> gp;
        gp.setInputCloud(pcl_cloud);
        gp.setSearchMethod(tree);
        gp.setIsoLevel(iso_level);
        gp.setGridResolution(res_x, res_y, res_z);
        gp.setPercentageExtendGrid(percentage);
        gp.setDistanceIgnore(dist_ignore);
        gp.reconstruct(*mesh);

        reportProgress(on_progress, 100);

        result.mesh = mesh;
        result.time_ms = static_cast<float>(time.toc());
        return result;
    }

    SurfaceResult Surface::ConvexHull(const Cloud::Ptr& cloud,
                                      bool value, int dimensio,
                                      std::atomic<bool>* cancel,
                                      std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        SurfaceResult result;
        if (isCanceled(cancel)) return result;
        if (cloud->empty()) return makeErrorResult("ConvexHull requires a non-empty point cloud.");

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        PolygonMesh::Ptr mesh(new PolygonMesh);

        pcl::ConvexHull<PointXYZRGBN> sur;
        sur.setInputCloud(pcl_cloud);
        sur.setSearchMethod(tree);
        sur.setComputeAreaVolume(value);
        sur.setDimension(dimensio);
        sur.reconstruct(*mesh);

        reportProgress(on_progress, 100);

        result.mesh = mesh;
        result.time_ms = static_cast<float>(time.toc());
        return result;
    }

    SurfaceResult Surface::ConcaveHull(const Cloud::Ptr& cloud,
                                       double alpha, bool value, int dimensio,
                                       std::atomic<bool>* cancel,
                                       std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        SurfaceResult result;
        if (isCanceled(cancel)) return result;
        if (cloud->empty()) return makeErrorResult("ConcaveHull requires a non-empty point cloud.");
        if (alpha <= 0) return makeErrorResult("ConcaveHull requires alpha > 0.");

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        PolygonMesh::Ptr mesh(new PolygonMesh);

        pcl::ConcaveHull<PointXYZRGBN> sur;
        sur.setInputCloud(pcl_cloud);
        sur.setSearchMethod(tree);
        sur.setAlpha(alpha);
        sur.setKeepInformation(value);
        sur.setDimension(dimensio);
        sur.reconstruct(*mesh);

        reportProgress(on_progress, 100);

        result.mesh = mesh;
        result.time_ms = static_cast<float>(time.toc());
        return result;
    }

}  // namespace ct
