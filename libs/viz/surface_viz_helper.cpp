#include "surface_viz_helper.h"

#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/conversions.h>
#include <vtkPolyDataNormals.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

namespace pw
{

void prepareSurfaceForRendering(const pcl::PolygonMesh::Ptr& mesh,
                                 SurfaceResultViz& viz)
{
    if (!mesh || mesh->polygons.empty()) return;

    // 1. 提取 mesh 顶点为 Cloud
    pcl::PointCloud<PointXYZRGBN> mesh_points;
    pcl::fromPCLPointCloud2(mesh->cloud, mesh_points);
    if (mesh_points.size() > 0) {
        viz.prepared_cloud = Cloud::fromPCL_XYZRGBN(mesh_points);
        viz.prepared_cloud->makeAdaptive();
    }

    // 2. 预构建 VTK polydata（含法线和顶点颜色）
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::VTKUtils::mesh2vtk(*mesh, polydata);
    if (!polydata || polydata->GetNumberOfPoints() == 0) return;

    polydata->GetPointData()->Initialize();

    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetNumberOfTuples(polydata->GetNumberOfPoints());
    unsigned char gray = 204;
    for (vtkIdType i = 0; i < polydata->GetNumberOfPoints(); ++i) {
        colors->SetTuple3(i, gray, gray, gray);
    }
    colors->SetName("MeshColors");
    polydata->GetPointData()->SetScalars(colors);

    vtkSmartPointer<vtkPolyDataNormals> normalGen = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGen->SetInputData(polydata);
    normalGen->ConsistencyOn();
    normalGen->AutoOrientNormalsOn();
    normalGen->NonManifoldTraversalOff();
    normalGen->Update();
    polydata = normalGen->GetOutput();

    if (!polydata->GetPointData()->GetArray("MeshColors")) {
        vtkSmartPointer<vtkUnsignedCharArray> colors2 = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors2->SetNumberOfComponents(3);
        colors2->SetNumberOfTuples(polydata->GetNumberOfPoints());
        for (vtkIdType i = 0; i < polydata->GetNumberOfPoints(); ++i) {
            colors2->SetTuple3(i, gray, gray, gray);
        }
        colors2->SetName("MeshColors");
        polydata->GetPointData()->SetScalars(colors2);
    }

    viz.prepared_polydata = polydata;
}

}  // namespace pw
