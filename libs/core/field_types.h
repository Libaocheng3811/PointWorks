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

    // 距离计算参数
    struct DistanceParams {
        enum Method {
            C2C_NEAREST = 0,    // 最近邻距离
            C2C_KNN_MEAN = 1,   // K近邻平均距离
            C2C_RADIUS_MEAN = 2,// 半径内平均距离
            C2M_SIGNED = 3,     // 点到网格有符号距离 (预留)
            M3C2 = 4            // M3C2 算法 (预留)
        };
        Method method;

        // C2C_KNN_MEAN 参数
        int k_knn = 6;

        // C2C_RADIUS_MEAN 参数
        double radius = 0.5;

        // C2M 参数 (预留)
        bool flip_normals = false;

        // M3C2 参数 (预留)
    };
}

#endif //POINTWORKS_FIELD_TYPES_H
