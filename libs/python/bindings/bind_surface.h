#pragma once

#include "bind_common.h"
#include <memory>

namespace pcl { struct PolygonMesh; }

// PyMesh — Python 端的网格数据包装
// PCL 交互方法在 mesh_utils.cpp 中实现，避免 PCL/pybind11 宏冲突
class PyMesh
{
public:
    explicit PyMesh(std::shared_ptr<pcl::PolygonMesh> mesh);
    ~PyMesh(); // 让 mesh_utils.cpp 控制析构时 PCL 类型完整

    py::array_t<float> vertices() const;
    py::array_t<int32_t> faces() const;
    size_t numVertices() const;
    size_t numFaces() const;
    std::shared_ptr<pcl::PolygonMesh> meshPtr() const;

private:
    std::shared_ptr<pcl::PolygonMesh> m_mesh;
};

void registerSurfaceBindings(py::module_& m);
