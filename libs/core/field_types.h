#ifndef POINTWORKS_FIELD_TYPES_H
#define POINTWORKS_FIELD_TYPES_H

#include "exports.h"

#include <string>
#include <vector>
#include <map>

namespace ct
{
    // 字段映射信息结构
    struct FieldInfo {
        std::string name;
        std::string type; // int, float, uint8 etc.
    };

    // 字段映射映射结果
    struct MappingResult {
        // Key: 文件中的字段名, Value: 映射的目标 (如 "Ignore", "Scalar Field", "Red", "Intensity"...)
        std::map<std::string, std::string> field_map;
    };

    // txt格式点云导出配置
    struct TxtExportParams {
        bool has_header = false; // 是否有表头
        char separator = ' '; // 分隔符,默认空格
        int precision = 6; //小数精度
        std::vector<std::string> selected_fields; // 选择导出的字段(按顺序)
    };

    // TXT格式点云导入配置参数
    struct TxtImportParams {
        int skip_lines = 0; // 跳过行数
        char separator = ' ';
        std::map<int, std::string> col_map; // 列索引->属性名
    };

    // ============================================================
    // Distance 参数结构体 (重构后)
    // ============================================================

    // 基础距离参数 (所有距离计算共享)
    struct DistanceParams {
        // DEPRECATED: Legacy method enum — kept for ChangeDetectPlugin backward compatibility.
        // New code should use C2CParams::Method instead.
        enum Method {
            C2C_NEAREST = 0,      // 最近邻距离
            C2C_KNN_MEAN = 1,     // K 近邻平均距离
            C2C_RADIUS_MEAN = 2,  // 半径内平均距离
            C2M_SIGNED = 3,       // 点到网格有符号距离 (预留)
            M3C2 = 4              // M3C2 算法 (预留)
        };
        Method method = C2C_NEAREST;   // DEPRECATED

        int k_knn = 6;                 // DEPRECATED: use C2CParams
        double radius = 0.5;           // DEPRECATED: use C2CParams
        bool flip_normals = false;     // DEPRECATED: use C2MParams

        // 搜索方法
        enum SearchMethod {
            OCTREE = 0,
            KDTREE = 1
        };
        SearchMethod search_method = KDTREE;

        // 最大搜索距离, 0 = 无限制, 超过此距离标记为 NaN
        double max_distance = 0.0;
    };

    // C2C: 点云对点云
    struct C2CParams : DistanceParams {
        enum Method {
            C2C_NEAREST = 0,      // 最近邻距离
            C2C_KNN_MEAN = 1,     // K 近邻平均距离
            C2C_RADIUS_MEAN = 2   // 半径内平均距离
        };
        Method method = C2C_NEAREST;

        int k_knn = 6;            // C2C_KNN_MEAN 参数
        double radius = 0.5;      // C2C_RADIUS_MEAN 参数
    };

    // C2M: 点云对网格
    struct C2MParams : DistanceParams {
        bool signed_distance = true;   // 是否计算有符号距离 (正=外侧, 负=内侧)
        bool flip_normals = false;     // 翻转网格法线方向
        double search_radius = 0.0;    // 搜索距离, 0 = 自动根据点间距计算
    };

    // C2P: 点云对几何基元
    enum class PrimitiveType {
        PLANE = 0,
        SPHERE = 1,
        CYLINDER = 2,
        CONE = 3
    };

    struct C2PParams : DistanceParams {
        PrimitiveType primitive_type = PrimitiveType::PLANE;

        // 平面参数 (ax + by + cz + d = 0)
        struct { double a = 0, b = 0, c = 1, d = 0; } plane_params;

        // 球面参数
        struct { double cx = 0, cy = 0, cz = 0, radius = 1; } sphere_params;

        // 圆柱体参数 (轴线方向 + 轴上一点 + 半径)
        struct {
            double cx = 0, cy = 0, cz = 0;
            double axis_x = 0, axis_y = 0, axis_z = 1;
            double radius = 1;
        } cylinder_params;

        // 圆锥体参数 (顶点 + 轴线方向 + 半角)
        struct {
            double apex_x = 0, apex_y = 0, apex_z = 0;
            double axis_x = 0, axis_y = 0, axis_z = 1;
            double half_angle = 0.5;
        } cone_params;
    };

    // CPS: 最邻近点集提取
    struct CPSParams : DistanceParams {
        bool keep_colors = true;        // 保留源点云颜色
        bool keep_intensity = false;    // 保留源点云强度
        bool keep_scalar_fields = false; // 保留源点云标量场
    };

    // ============================================================
    // 距离计算结果
    // ============================================================

    struct DistanceResult {
        std::vector<float> distances;  // 每个点的距离值 (NaN = 超出 max_distance)
        float time_ms = 0;
        bool success = true;
        std::string error_msg;
    };

} // namespace ct

#endif //POINTWORKS_FIELD_TYPES_H
