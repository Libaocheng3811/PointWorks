#ifndef POINTWORKS_SURFACE_VIZ_HELPER_H
#define POINTWORKS_SURFACE_VIZ_HELPER_H

#include "core/cloud.h"
#include "core/exports.h"

#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>

class vtkPolyData;

namespace ct
{

/// 曲面重建结果的可视化数据（含预构建的 VTK polydata）
struct SurfaceResultViz {
    Cloud::Ptr prepared_cloud;
    vtkSmartPointer<vtkPolyData> prepared_polydata;
};

/// 将 SurfaceResult 转换为可渲染数据（在工作线程中调用）
CT_VIZ_EXPORT void prepareSurfaceForRendering(const pcl::PolygonMesh::Ptr& mesh,
                                                SurfaceResultViz& viz);

}  // namespace ct

#endif  // POINTWORKS_SURFACE_VIZ_HELPER_H
