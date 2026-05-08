#pragma once

#include <pcl/PolygonMesh.h>
#include <memory>
#include <string>

namespace pw
{

/**
 * @brief 带纹理的网格数据
 * 保留 pcl::PolygonMesh 用于非纹理操作（如保存）。
 * CloudView 根据 objFilePath 自行解析 MTL 获取所有材质和纹理。
 */
struct TexturedMesh
{
    pcl::PolygonMesh::Ptr mesh;            // 几何面片
    std::string          objFilePath;      // OBJ 文件绝对路径

    explicit operator bool() const { return mesh != nullptr && !mesh->polygons.empty(); }
};

using TexturedMeshPtr = std::shared_ptr<TexturedMesh>;

} // namespace pw
