#include "fileio.h"

#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>

#include <QFileInfo>
#include <QFile>
#include <QTextStream>
#include <QRegularExpression>

#include "E57SimpleReader.h"
#include "E57SimpleWriter.h"

#include "lasreader.hpp"
#include "laswriter.hpp"

#include <fstream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <cfloat>

namespace ct
{

class FastLineParser {
public:
    std::vector<char*> tokens;
    std::string buffer;

    // 根据分隔符解析一行
    void parse(std::istream& file, char separator) {
        tokens.clear();
        if (!std::getline(file, buffer)) return;

        // 如果是空行，直接返回
        if (buffer.empty()) return;
        // 处理 Windows CR
        if (buffer.back() == '\r') buffer.pop_back();

        char* ptr = &buffer[0];
        char* end = ptr + buffer.size();

        // 特殊处理空格分隔符 (处理连续空格)
        if (separator == ' ') {
            while (ptr < end) {
                // 跳过前导空格
                while (ptr < end && std::isspace(*ptr)) ptr++;
                if (ptr >= end) break;

                tokens.push_back(ptr);

                // 找到 token 结尾
                while (ptr < end && !std::isspace(*ptr)) ptr++;
                if (ptr < end) *ptr++ = '\0'; // 截断字符串
            }
        }
        else {
            // 处理显式分隔符 (如逗号)
            while (ptr < end) {
                tokens.push_back(ptr);
                while (ptr < end && *ptr != separator) ptr++;
                if (ptr < end) *ptr++ = '\0';
            }
        }
    }
};

QString getPCLFieldType(uint8_t type)
{
    switch(type) {
        case pcl::PCLPointField::INT8: return "int8";
        case pcl::PCLPointField::UINT8: return "uint8";
        case pcl::PCLPointField::INT16: return "int16";
        case pcl::PCLPointField::UINT16: return "uint16";
        case pcl::PCLPointField::INT32: return "int32";
        case pcl::PCLPointField::UINT32: return "uint32";
        case pcl::PCLPointField::FLOAT32: return "float";
        case pcl::PCLPointField::FLOAT64: return "double";
        default: return "unknown";
    }
}

// 辅助函数：安全地从指针读取值
template<typename T>
T safeRead(const uint8_t* base_ptr, size_t offset, size_t max_size) {
    size_t required = offset + sizeof(T);
    if (required > max_size || offset >= max_size) {
        return T();
    }
    return *reinterpret_cast<const T*>(base_ptr + offset);
}

// 辅助函数：从 Blob 中提取 float 值（带边界检查）
float getFieldValueAsFloat(const uint8_t* data_ptr, int datatype, size_t max_size) {
    using namespace pcl;
    switch(datatype) {
        case PCLPointField::INT8:
            if (max_size >= 1) return (float)(*(int8_t*)data_ptr);
            break;
        case PCLPointField::UINT8:
            if (max_size >= 1) return (float)(*(uint8_t*)data_ptr);
            break;
        case PCLPointField::INT16:
            if (max_size >= 2) return (float)(*(int16_t*)data_ptr);
            break;
        case PCLPointField::UINT16:
            if (max_size >= 2) return (float)(*(uint16_t*)data_ptr);
            break;
        case PCLPointField::INT32:
            if (max_size >= 4) return (float)(*(int32_t*)data_ptr);
            break;
        case PCLPointField::UINT32:
            if (max_size >= 4) return (float)(*(uint32_t*)data_ptr);
            break;
        case PCLPointField::FLOAT32:
            if (max_size >= 4) return *(float*)data_ptr;
            break;
        case PCLPointField::FLOAT64:
            if (max_size >= 8) return (float)(*(double*)data_ptr);
            break;
        default:
            break;
    }
    return 0.0f;  // 默认值
}

// ================================================================
// PLY / PCD 加载
// ================================================================

bool FileIO::loadPLY_PCD(const QString &filename, Cloud::Ptr &cloud) {
    pcl::PCLPointCloud2 blob;
    int res = -1;

    // 先初步读取ply信息到blob中
    if (filename.endsWith(".ply", Qt::CaseInsensitive))
        res = pcl::io::loadPLYFile(filename.toLocal8Bit().toStdString(), blob);
    else
        res = pcl::io::loadPCDFile(filename.toLocal8Bit().toStdString(), blob); // todo 这步十分耗时

    if (res == -1) return false;

    // 准备字段信息
    QList<ct::FieldInfo> fields_info;
    for (const auto &f: blob.fields) {
        ct::FieldInfo fi;
        fi.name = f.name;
        fi.type = getPCLFieldType(f.datatype).toStdString();
        fields_info.append(fi);
    }

    // 请求 UI, mapping_result是用户选择的映射结果，Key是原字段名，Value是映射后的字段名
    std::map<std::string, std::string> mapping_result;
    emit requestFieldMapping(fields_info, mapping_result);
    if (mapping_result.empty()) return false; // 用户取消

    size_t total_points = blob.width * blob.height;
    if (total_points == 0) return false;

    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto& f : blob.fields){
        if (f.name == "x") x_off = f.offset;
        if (f.name == "y") y_off = f.offset;
        if (f.name == "z") z_off = f.offset;
    }
    if (x_off < 0 || y_off < 0 || z_off < 0) return false;

    const uint8_t* raw_data = blob.data.data();
    int step = blob.point_step;
    size_t blob_data_size = blob.data.size();

    // 预扫描计算 BBox
    PointXYZ min_pt(FLT_MAX, FLT_MAX, FLT_MAX);
    PointXYZ max_pt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    // 检查第一个点确定 Shift
    Eigen::Vector3d suggested_shift = Eigen::Vector3d::Zero();

    if (total_points > 0 && blob_data_size >= (size_t)std::max({x_off, y_off, z_off}) + 4) {
        float x0 = *reinterpret_cast<const float*>(raw_data + x_off);
        float y0 = *reinterpret_cast<const float*>(raw_data + y_off);
        float z0 = *reinterpret_cast<const float*>(raw_data + z_off);

        const double THRESHOLD_XY = 10000.0;
        if (std::abs(x0) > THRESHOLD_XY || std::abs(y0) > THRESHOLD_XY) {
            double sx = -std::floor(x0 / 1000.0) * 1000.0;
            double sy = -std::floor(y0 / 1000.0) * 1000.0;
            double sz = 0.0;
            suggested_shift = Eigen::Vector3d(sx, sy, sz);

            bool skipped = false;
            emit requestGlobalShift(Eigen::Vector3d(x0, y0, z0), suggested_shift, skipped);
            if (!skipped) cloud->setGlobalShift(-suggested_shift);
        }
    }

    float shift_x = (float)suggested_shift.x();
    float shift_y = (float)suggested_shift.y();
    float shift_z = (float)suggested_shift.z();

    // 快速扫描计算边界 (OpenMP 加速)
    for (size_t i = 0; i < total_points; ++i) {
        const uint8_t* pt_ptr = raw_data + i * step;
        float x = *reinterpret_cast<const float*>(pt_ptr + x_off) + shift_x;
        float y = *reinterpret_cast<const float*>(pt_ptr + y_off) + shift_y;
        float z = *reinterpret_cast<const float*>(pt_ptr + z_off) + shift_z;

        if (x < min_pt.x) min_pt.x = x;
        if (x > max_pt.x) max_pt.x = x;
        if (y < min_pt.y) min_pt.y = y;
        if (y > max_pt.y) max_pt.y = y;
        if (z < min_pt.z) min_pt.z = z;
        if (z > max_pt.z) max_pt.z = z;
    }

    // 初始化八叉树
    Box box;
    box.width  = (max_pt.x - min_pt.x) * 1.01;
    box.height = (max_pt.y - min_pt.y) * 1.01;
    box.depth  = (max_pt.z - min_pt.z) * 1.01;
    box.translation = Eigen::Vector3f(
            (min_pt.x + max_pt.x) * 0.5f,
            (min_pt.y + max_pt.y) * 0.5f,
            (min_pt.z + max_pt.z) * 0.5f
    );
    cloud->initOctree(box);

    emit progress(10);

    // 准备解析
    struct ExtractInfo {
        enum Type { Scalar, ColorPacked, ColorR, ColorG, ColorB }; // 支持分开的颜色分量
        Type type;
        QString saveName;
        int offset;
        int datatype;
    };
    std::vector<ExtractInfo> tasks;
    bool has_color = false;
    bool has_normal = false;
    int nx_off = -1, ny_off = -1, nz_off = -1;  // 法线分量的偏移量
    int nx_datatype = 0, ny_datatype = 0, nz_datatype = 0;  // 法线分量的数据类型
    int r_off = -1, g_off = -1, b_off = -1;  // 颜色分量的偏移量
    int r_datatype = 0, g_datatype = 0, b_datatype = 0;  // 颜色分量的数据类型

    // 遍历映射表，确定要提取哪些字段
    bool use_packed_color = false;  // 标记是否使用打包颜色
    for (const auto& f : blob.fields) {
        QString fname = QString::fromStdString(f.name);
        if (mapping_result.find(f.name) == mapping_result.end()) continue;

        QString action = QString::fromStdString(mapping_result.at(f.name));
        if (action == "Scalar Field" || action == "Intensity") {
            QString saveName = (action == "Intensity") ? "Intensity" : fname;
            tasks.push_back({ExtractInfo::Scalar, saveName, (int)f.offset, f.datatype});
        }
        else if (action.contains("Color") && !action.contains("Red") && !action.contains("Green") && !action.contains("Blue")) {
            // 打包的颜色字段 (rgb 或 rgba)
            has_color = true;
            use_packed_color = true;
            tasks.push_back({ExtractInfo::ColorPacked, "", (int)f.offset, f.datatype});
        }
        else if (action.contains("Red") || action == "r"){
            has_color = true;
            r_off = f.offset;
            r_datatype = f.datatype;
        }
        else if (action.contains("Green") || action == "g"){
            has_color = true;
            g_off = f.offset;
            g_datatype = f.datatype;
        }
        else if (action.contains("Blue") || action == "b"){
            has_color = true;
            b_off = f.offset;
            b_datatype = f.datatype;
        }
        else if (action.contains("Normal X")){
            has_normal = true;
            nx_off = f.offset;
            nx_datatype = f.datatype;
        }
        else if (action.contains("Normal Y")){
            has_normal = true;
            ny_off = f.offset;
            ny_datatype = f.datatype;
        }
        else if (action.contains("Normal Z")){
            has_normal = true;
            nz_off = f.offset;
            nz_datatype = f.datatype;
        }
    }

    // 优先使用分开的颜色分量，如果没有才使用打包颜色
    if (has_color && !use_packed_color && (r_off < 0 || g_off < 0 || b_off < 0)) {
        // 如果用户选择了颜色但没有完整的 RGB 分量，禁用颜色
        has_color = false;
    }

    if (has_color) cloud->enableColors();
    // 如果检测到有法线字段，预分配法线容器
    if (has_normal) cloud->enableNormals();

    CloudBatch batch;
    batch.reserve(BATCH_SIZE);

    int progress_interval = (total_points > 100) ? (total_points / 100) : 1;

    // 循环处理每个点
    for (size_t i = 0; i < total_points; ++i) {
        if (m_is_canceled) return false;

        // 计算当前点的数据指针和剩余字节数 (安全检查)
        const uint8_t* pt_ptr = raw_data + i * step;
        size_t remaining = blob_data_size - (i * step);
        if (remaining < (size_t)step) break; // 防止越界

        // --- 提取并应用偏移 (XYZ) ---
        float x = *reinterpret_cast<const float*>(pt_ptr + x_off) + shift_x;
        float y = *reinterpret_cast<const float*>(pt_ptr + y_off) + shift_y;
        float z = *reinterpret_cast<const float*>(pt_ptr + z_off) + shift_z;
        batch.points.emplace_back(x, y, z);

        // --- 提取其他属性 (Scalar / Packed Color) ---
        for (const auto& t : tasks) {
            if (t.type == ExtractInfo::Scalar) {
                float val = getFieldValueAsFloat(pt_ptr + t.offset, t.datatype, remaining);
                batch.scalars[t.saveName.toStdString()].push_back(val);
            }
            else if (t.type == ExtractInfo::ColorPacked) {
                // 处理打包颜色 (如 float rgba)
                uint32_t rgb_packed = safeRead<uint32_t>(pt_ptr, t.offset, remaining);
                uint8_t r = (rgb_packed >> 16) & 0xFF;
                uint8_t g = (rgb_packed >> 8) & 0xFF;
                uint8_t b = rgb_packed & 0xFF;

                // 只有当颜色还未被其他字段填充时才添加 (避免冲突)
                if (batch.colors.size() < batch.points.size()) {
                    batch.colors.emplace_back(r, g, b);
                }
            }
        }

        // --- 提取 RGB 分量 (如果存在独立字段) ---
        if (has_color && r_off >= 0 && g_off >= 0 && b_off >= 0) {
            if (batch.colors.size() < batch.points.size()) {
                uint8_t r = static_cast<uint8_t>(getFieldValueAsFloat(pt_ptr + r_off, r_datatype, remaining));
                uint8_t g = static_cast<uint8_t>(getFieldValueAsFloat(pt_ptr + g_off, g_datatype, remaining));
                uint8_t b = static_cast<uint8_t>(getFieldValueAsFloat(pt_ptr + b_off, b_datatype, remaining));
                batch.colors.emplace_back(r, g, b);
            }
        }
        // 补齐颜色 (如果上面两种方式都没提取到，或者数据缺失)
        if (has_color && batch.colors.size() < batch.points.size()) {
            batch.colors.emplace_back(255, 255, 255); // Default White
        }

        // --- 提取法线 (Normal) ---
        if (has_normal && nx_off >= 0) {
            float nx = getFieldValueAsFloat(pt_ptr + nx_off, nx_datatype, remaining);
            float ny = getFieldValueAsFloat(pt_ptr + ny_off, ny_datatype, remaining);
            float nz = getFieldValueAsFloat(pt_ptr + nz_off, nz_datatype, remaining);

            CompressedNormal cn;
            cn.set(Eigen::Vector3f(nx, ny, nz));
            batch.normals.push_back(cn);
        }

        // --- 批次提交 ---
        if (batch.points.size() >= BATCH_SIZE) {
            batch.flushTo(cloud);
        }

        // --- 进度条 ---
        if (i % progress_interval == 0) {
            int p = 10 + (int)(i * 90 / total_points);
            emit progress(p);
        }
    }

    // 提交剩余数据
    if (!batch.empty()) {
        batch.flushTo(cloud);
    }

    cloud->setHasColors(has_color);
    cloud->setHasNormals(has_normal);

    cloud->makeAdaptive();

    return true;
}

// ================================================================
// TXT / XYZ / ASC 加载
// ================================================================

bool FileIO::loadTXT(const QString &filename, Cloud::Ptr &cloud) {
    std::ifstream file(filename.toLocal8Bit().constData());
    if (!file.is_open()) return false;

    // 获取文件大小用于进度条
    file.seekg(0, std::ios::end);
    long long file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // --- 预读与交互配置 ---
    QStringList preview_lines;
    std::string line;
    int preview_count = 0;
    while (std::getline(file, line) && preview_count < 50) {
        if (!line.empty()) {
            preview_lines << QString::fromStdString(line);
            preview_count++;
        }
    }

    // 重置文件指针
    file.clear();
    file.seekg(0);

    ct::TxtImportParams params;
    // 阻塞调用 UI 获取列映射
    emit requestTxtImportSetup(preview_lines, params);

    if (params.col_map.empty()) return false; // 用户取消

    // 验证必要字段 X, Y, Z
    int idx_x = -1, idx_y = -1, idx_z = -1;
    int idx_r = -1, idx_g = -1, idx_b = -1;
    int idx_nx = -1, idx_ny = -1, idx_nz = -1;

    // 标量场映射: vector<pair<col_index, field_name>>
    std::vector<std::pair<int, QString>> scalar_indices;

    for (auto it = params.col_map.begin(); it != params.col_map.end(); ++it) {
        int col = it->first - 1; // 转换为 0-based 索引 (通常 UI 可能是 1-based，请根据实际 TxtImportParams 调整)
        // 假设 params.col_map 的 key 是 0-based 的列索引：
        col = it->first;

        std::string type = it->second;
        if (type == "x") idx_x = col;
        else if (type == "y") idx_y = col;
        else if (type == "z") idx_z = col;
        else if (type == "r") idx_r = col;
        else if (type == "g") idx_g = col;
        else if (type == "b") idx_b = col;
        else if (type == "nx") idx_nx = col;
        else if (type == "ny") idx_ny = col;
        else if (type == "nz") idx_nz = col;
        else if (type != "ignore") {
            scalar_indices.push_back({col, QString::fromStdString(type)});
        }
    }

    if (idx_x < 0 || idx_y < 0 || idx_z < 0) return false;

    // 跳过 header lines
    for(int i=0; i<params.skip_lines; ++i) std::getline(file, line);

    // 记录数据区起始位置
    std::streampos data_start_pos = file.tellg();
    long long data_start_offset = (long long)data_start_pos;

    // =========================================================
    // Pass 1: 快速扫描计算 Bounding Box 和 Global Shift
    // =========================================================
    emit progress(1);

    PointXYZ min_pt(FLT_MAX, FLT_MAX, FLT_MAX);
    PointXYZ max_pt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    FastLineParser parser;
    long long read_bytes = 0;
    size_t scanned_points = 0;

    // 寻找最大列索引，用于安全检查
    int max_req_col = std::max({idx_x, idx_y, idx_z});

    while (file.good()) {
        if (m_is_canceled) return false;

        parser.parse(file, params.separator);
        if (parser.tokens.empty()) {
            if (file.eof()) break;
            continue;
        }

        if ((int)parser.tokens.size() > max_req_col) {
            // 使用 strtof 快速转换
            float x = std::strtof(parser.tokens[idx_x], nullptr);
            float y = std::strtof(parser.tokens[idx_y], nullptr);
            float z = std::strtof(parser.tokens[idx_z], nullptr);

            if (x < min_pt.x) min_pt.x = x;
            if (x > max_pt.x) max_pt.x = x;
            if (y < min_pt.y) min_pt.y = y;
            if (y > max_pt.y) max_pt.y = y;
            if (z < min_pt.z) min_pt.z = z;
            if (z > max_pt.z) max_pt.z = z;

            scanned_points++;
        }

        // 简单的 Pass 1 进度反馈 (1% - 10%)
        if (scanned_points % 50000 == 0) {
            read_bytes += parser.buffer.size(); // 近似
            int p = 1 + (int)(read_bytes * 9 / (file_size - data_start_offset));
            emit progress(p);
        }
    }

    if (scanned_points == 0) return false;

    // =========================================================
    // 计算 Global Shift
    // =========================================================
    Eigen::Vector3d suggested_shift = Eigen::Vector3d::Zero();
    const double THRESHOLD_XY = 10000.0;

    // 检查是否是大坐标
    if (std::abs(min_pt.x) > THRESHOLD_XY || std::abs(min_pt.y) > THRESHOLD_XY) {
        double sx = -std::floor(min_pt.x / 1000.0) * 1000.0;
        double sy = -std::floor(min_pt.y / 1000.0) * 1000.0;
        double sz = 0.0;
        suggested_shift = Eigen::Vector3d(sx, sy, sz);

        bool skipped = false;
        // 交互询问用户是否应用偏移
        emit requestGlobalShift(Eigen::Vector3d(min_pt.x, min_pt.y, min_pt.z), suggested_shift, skipped);

        if (!skipped) cloud->setGlobalShift(-suggested_shift);
    }

    float shift_x = (float)suggested_shift.x();
    float shift_y = (float)suggested_shift.y();
    float shift_z = (float)suggested_shift.z();

    // =========================================================
    // 初始化八叉树
    // =========================================================
    Box box;
    // 应用 Shift 后的包围盒大小
    box.width  = (max_pt.x - min_pt.x) * 1.01;
    box.height = (max_pt.y - min_pt.y) * 1.01;
    box.depth  = (max_pt.z - min_pt.z) * 1.01;
    // 中心点也需要加上 shift (变成局部坐标系下的中心)
    box.translation = Eigen::Vector3f(
            (min_pt.x + max_pt.x) * 0.5f + shift_x,
            (min_pt.y + max_pt.y) * 0.5f + shift_y,
            (min_pt.z + max_pt.z) * 0.5f + shift_z
    );
    cloud->initOctree(box);

    // =========================================================
    // Pass 2: 完整读取与流式加载
    // =========================================================

    file.clear(); // 清除 EOF 标志
    file.seekg(data_start_pos);
    read_bytes = 0;

    // 准备属性开关
    bool has_color = (idx_r >= 0 && idx_g >= 0 && idx_b >= 0);
    bool has_normal = (idx_nx >= 0 && idx_ny >= 0 && idx_nz >= 0);

    if (has_color) cloud->enableColors();
    if (has_normal) cloud->enableNormals();

    // 准备批处理缓冲区
    CloudBatch batch;
    batch.reserve(BATCH_SIZE);

    size_t processed_count = 0;
    int progress_interval = (scanned_points > 100) ? (scanned_points / 100) : 1000;

    while (file.good()) {
        if (m_is_canceled) return false;

        parser.parse(file, params.separator);
        if (parser.tokens.empty()) {
            if (file.eof()) break;
            continue;
        }

        // 解析 XYZ (带 Shift)
        if ((int)parser.tokens.size() > idx_x && (int)parser.tokens.size() > idx_y && (int)parser.tokens.size() > idx_z) {
            float x = std::strtof(parser.tokens[idx_x], nullptr) + shift_x;
            float y = std::strtof(parser.tokens[idx_y], nullptr) + shift_y;
            float z = std::strtof(parser.tokens[idx_z], nullptr) + shift_z;

            batch.points.emplace_back(x, y, z);

            // 解析 RGB
            if (has_color) {
                // 安全检查：防止行尾缺少列
                int r = ((int)parser.tokens.size() > idx_r) ? std::strtof(parser.tokens[idx_r], nullptr) : 0;
                int g = ((int)parser.tokens.size() > idx_g) ? std::strtof(parser.tokens[idx_g], nullptr) : 0;
                int b = ((int)parser.tokens.size() > idx_b) ? std::strtof(parser.tokens[idx_b], nullptr) : 0;
                batch.colors.emplace_back((uint8_t)r, (uint8_t)g, (uint8_t)b);
            }

            // 解析法线
            if (has_normal) {
                float nx = ((int)parser.tokens.size() > idx_nx) ? std::strtof(parser.tokens[idx_nx], nullptr) : 0;
                float ny = ((int)parser.tokens.size() > idx_ny) ? std::strtof(parser.tokens[idx_ny], nullptr) : 0;
                float nz = ((int)parser.tokens.size() > idx_nz) ? std::strtof(parser.tokens[idx_nz], nullptr) : 0;

                CompressedNormal cn;
                cn.set(Eigen::Vector3f(nx, ny, nz));
                batch.normals.push_back(cn);
            }

            // 解析标量场
            for (const auto& kv : scalar_indices) {
                int col = kv.first;
                const QString& name = kv.second;

                float val = 0.0f;
                if ((int)parser.tokens.size() > col) {
                    val = std::strtof(parser.tokens[col], nullptr);
                }
                batch.scalars[name.toStdString()].push_back(val);
            }
        }

        // 批次提交 (每 50w 点)
        if (batch.points.size() >= BATCH_SIZE) {
            batch.flushTo(cloud);
        }

        //  进度条 (10% - 100%)
        processed_count++;
        if (processed_count % progress_interval == 0) {
            int p = 10 + (int)(processed_count * 90 / scanned_points);
            if (p > 100) p = 100;
            emit progress(p);
        }
    }

    // 提交剩余点
    if (!batch.empty()) {
        batch.flushTo(cloud);
    }

    // 更新点云统计信息
    cloud->makeAdaptive();
    emit progress(100);

    return true;
}

// ================================================================
// LAS / LAZ 加载
// ================================================================

bool FileIO::loadLAS(const QString &filename, Cloud::Ptr &cloud) {
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(filename.toLocal8Bit().constData());
    LASreader* lasreader = lasreadopener.open();
    if (!lasreader) return false;

    size_t total_points = lasreader->npoints;
    if (total_points == 0){
        lasreader->close();
        delete lasreader;
        return false;
    }

    int fmt = lasreader->header.point_data_format;
    bool has_color = (fmt == 2 || fmt == 3 || fmt == 5 || fmt == 7);
    if (has_color) cloud->enableColors();

    // 准备流式缓冲区
    CloudBatch batch;
    batch.reserve(BATCH_SIZE);

    bool has_valid_intensity = false;

    // 读取第一个点以确定 Global Shift
    if (!lasreader->read_point()) {
        lasreader->close();
        delete lasreader;
        return false;
    }

    double raw_x = lasreader->point.get_x();
    double raw_y = lasreader->point.get_y();
    double raw_z = lasreader->point.get_z();

    // 计算 Global Shift
    const double THRESHOLD_XY = 10000.0;
    bool is_large = std::abs(raw_x) > THRESHOLD_XY || std::abs(raw_y) > THRESHOLD_XY;
    Eigen::Vector3d suggested_shift = Eigen::Vector3d::Zero();

    if (is_large) {
        double sx = -std::floor(raw_x / 1000.0) * 1000.0;
        double sy = -std::floor(raw_y / 1000.0) * 1000.0;
        double sz = 0.0;

        suggested_shift = Eigen::Vector3d(sx, sy, sz);
        bool skipped = false;
        emit requestGlobalShift(Eigen::Vector3d(raw_x, raw_y, raw_z), suggested_shift, skipped);

        if (!skipped) {
            cloud->setGlobalShift(-suggested_shift);
        }
    }

    float shift_x = static_cast<float>(suggested_shift.x());
    float shift_y = static_cast<float>(suggested_shift.y());
    float shift_z = static_cast<float>(suggested_shift.z());

    // 现在可以根据 Shift 后的范围初始化八叉树了
    Box box;
    box.width  = lasreader->header.max_x - lasreader->header.min_x;
    box.height = lasreader->header.max_y - lasreader->header.min_y;
    box.depth  = lasreader->header.max_z - lasreader->header.min_z;
    // 中心点也需要加上 shift
    double cx = (lasreader->header.min_x + lasreader->header.max_x) * 0.5 + shift_x;
    double cy = (lasreader->header.min_y + lasreader->header.max_y) * 0.5 + shift_y;
    double cz = (lasreader->header.min_z + lasreader->header.max_z) * 0.5 + shift_z;
    box.translation = Eigen::Vector3f(cx, cy, cz);

    // 稍微扩大一点 Box 防止边界浮点误差
    box.width *= 1.01; box.height *= 1.01; box.depth *= 1.01;

    cloud->initOctree(box);

    // 处理第一个点
    {
        float x = static_cast<float>(raw_x) + shift_x;
        float y = static_cast<float>(raw_y) + shift_y;
        float z = static_cast<float>(raw_z) + shift_z;

        batch.points.emplace_back(x, y, z);

        if (has_color) {
            uint8_t r = lasreader->point.rgb[0] >> 8;
            uint8_t g = lasreader->point.rgb[1] >> 8;
            uint8_t b = lasreader->point.rgb[2] >> 8;
            batch.colors.emplace_back(r, g, b);
        }

        float intensity_val = static_cast<float>(lasreader->point.get_intensity());
        batch.scalars["Intensity"].push_back(intensity_val);
        if (intensity_val > 0.0f) has_valid_intensity = true;
    }

    size_t idx = 1;
    int progress_interval = (total_points > 100) ? (total_points / 100) : 1;

    // 循环读取剩余点
    while (lasreader->read_point()) {
        if (m_is_canceled) {
            lasreader->close();
            delete lasreader;
            return false;
        }

        // 坐标
        float x = static_cast<float>(lasreader->point.get_x()) + shift_x;
        float y = static_cast<float>(lasreader->point.get_y()) + shift_y;
        float z = static_cast<float>(lasreader->point.get_z()) + shift_z;

        batch.points.emplace_back(x, y, z);

        // 颜色
        if (has_color) {
            uint8_t r = lasreader->point.rgb[0] >> 8;
            uint8_t g = lasreader->point.rgb[1] >> 8;
            uint8_t b = lasreader->point.rgb[2] >> 8;
            batch.colors.emplace_back(r, g, b);
        }

        // 强度
        float intensity_val = static_cast<float>(lasreader->point.get_intensity());
        batch.scalars["Intensity"].push_back(intensity_val);
        if (!has_valid_intensity && intensity_val > 0.0f) has_valid_intensity = true;

        // 满批次提交
        if (batch.points.size() >= BATCH_SIZE) {
            batch.flushTo(cloud);
        }

        // 进度条
        idx++;
        if (idx % progress_interval == 0) {
            int p = 5 + (int)(idx * 85 / total_points);
            emit progress(p);
        }
    }

    // 提交剩余数据
    if (!batch.empty()) {
        batch.flushTo(cloud);
    }

    // 后处理：如果强度全为 0，移除该字段
    if (!has_valid_intensity) {
        cloud->removeScalarField("Intensity");
    }

    cloud->setHasColors(has_color);

    cloud->makeAdaptive();

    lasreader->close();
    delete lasreader;
    return true;
}

// ================================================================
// E57 加载
// ================================================================

bool FileIO::loadE57(const QString &filename, Cloud::Ptr &cloud) {
    emit progress(5);

    std::string path = filename.toLocal8Bit().toStdString();

    try {
        e57::Reader reader(path, {});
        if (!reader.IsOpen()) return false;

        e57::E57Root file_header;
        reader.GetE57Root(file_header);

        int64_t scan_count = reader.GetData3DCount();
        if (scan_count <= 0) return false;

        emit progress(10);

        // 合并所有扫描站到同一个 Cloud
        // 先扫描所有站获取总点数和包围盒
        size_t total_points = 0;
        double global_min[3] = {DBL_MAX, DBL_MAX, DBL_MAX};
        double global_max[3] = {-DBL_MAX, -DBL_MAX, -DBL_MAX};

        bool has_color = false;
        bool has_intensity = false;

        std::vector<e57::Data3D> headers(scan_count);
        for (int64_t i = 0; i < scan_count; ++i) {
            if (m_is_canceled) return false;
            reader.ReadData3D(i, headers[i]);
            total_points += headers[i].pointCount;
            if (headers[i].pointFields.colorRedField) has_color = true;
            if (headers[i].pointFields.intensityField) has_intensity = true;
        }

        if (total_points == 0) return false;

        // 从所有站的包围盒计算全局包围盒
        for (int64_t i = 0; i < scan_count; ++i) {
            auto& b = headers[i].cartesianBounds;
            global_min[0] = std::min(global_min[0], b.xMinimum);
            global_min[1] = std::min(global_min[1], b.yMinimum);
            global_min[2] = std::min(global_min[2], b.zMinimum);
            global_max[0] = std::max(global_max[0], b.xMaximum);
            global_max[1] = std::max(global_max[1], b.yMaximum);
            global_max[2] = std::max(global_max[2], b.zMaximum);
        }

        // Global Shift 检测
        Eigen::Vector3d suggested_shift = Eigen::Vector3d::Zero();
        if (std::abs(global_min[0]) > 10000.0 || std::abs(global_min[1]) > 10000.0) {
            double sx = -std::floor(global_min[0] / 1000.0) * 1000.0;
            double sy = -std::floor(global_min[1] / 1000.0) * 1000.0;
            suggested_shift = Eigen::Vector3d(sx, sy, 0.0);

            bool skipped = false;
            emit requestGlobalShift(
                Eigen::Vector3d(global_min[0], global_min[1], global_min[2]),
                suggested_shift, skipped);
            if (!skipped) cloud->setGlobalShift(-suggested_shift);
        }

        float shift_x = (float)suggested_shift.x();
        float shift_y = (float)suggested_shift.y();
        float shift_z = (float)suggested_shift.z();

        // 初始化八叉树
        Box box;
        box.width  = (float)((global_max[0] - global_min[0]) * 1.01);
        box.height = (float)((global_max[1] - global_min[1]) * 1.01);
        box.depth  = (float)((global_max[2] - global_min[2]) * 1.01);
        box.translation = Eigen::Vector3f(
            (float)((global_min[0] + global_max[0]) * 0.5),
            (float)((global_min[1] + global_max[1]) * 0.5),
            (float)((global_min[2] + global_max[2]) * 0.5));
        cloud->initOctree(box);

        if (has_color) cloud->enableColors();

        CloudBatch batch;
        batch.reserve(BATCH_SIZE);

        size_t points_loaded = 0;
        int progress_interval = (total_points > 100) ? (size_t)(total_points / 80) : 1;

        // 逐站读取
        for (int64_t si = 0; si < scan_count; ++si) {
            if (m_is_canceled) return false;

            size_t count = headers[si].pointCount;
            if (count == 0) continue;

            // 配置数据缓冲区（缓冲区由 SetUpData3DPointsData 按 count 大小分配）
            e57::Data3DPointsFloat data(headers[si]);

            auto cv_reader = reader.SetUpData3DPointsData(si, count, data);

            // read() 一次性读取该站全部点，返回实际读取数
            unsigned got = cv_reader.read();

            for (size_t i = 0; i < got; ++i) {
                // 跳过无效点
                if (data.cartesianInvalidState && data.cartesianInvalidState[i]) continue;

                float x = (float)data.cartesianX[i] + shift_x;
                float y = (float)data.cartesianY[i] + shift_y;
                float z = (float)data.cartesianZ[i] + shift_z;

                batch.points.emplace_back(x, y, z);

                if (has_color && data.colorRed && data.colorGreen && data.colorBlue) {
                    if (!data.isColorInvalid || !data.isColorInvalid[i]) {
                        batch.colors.emplace_back(
                            (uint8_t)(data.colorRed[i] >> 8),
                            (uint8_t)(data.colorGreen[i] >> 8),
                            (uint8_t)(data.colorBlue[i] >> 8));
                    } else {
                        batch.colors.emplace_back(0, 0, 0);
                    }
                }

                if (batch.points.size() >= BATCH_SIZE) {
                    batch.flushTo(cloud);
                }
            }

            points_loaded += got;
            cv_reader.close();

            if (points_loaded % progress_interval < got) {
                int pct = 10 + (int)(80 * points_loaded / total_points);
                emit progress(pct);
            }
        }

        if (!batch.empty()) batch.flushTo(cloud);

        if (has_color) cloud->setHasColors(true);
        cloud->makeAdaptive();

        reader.Close();
        emit progress(100);
        return true;

    } catch (const e57::E57Exception& e) {
        // 读取失败，忽略
        return false;
    } catch (...) {
        return false;
    }
}

// ================================================================
// LAS 保存
// ================================================================

bool FileIO::saveLAS(const Cloud::Ptr &cloud, const QString &filename) {
    LASwriteOpener laswriteopener;
    laswriteopener.set_file_name(filename.toLocal8Bit().constData());
    LASheader lasheader;

    // 确定格式
    if (cloud->hasColors()) {
        lasheader.point_data_format = 2; // XYZ + RGB + Intensity
        lasheader.point_data_record_length = 26;
    } else {
        lasheader.point_data_format = 0;  // XYZ + Intensity
        lasheader.point_data_record_length = 20;
    }

    // 设置精度因子
    lasheader.x_scale_factor = 0.0001;
    lasheader.y_scale_factor = 0.0001;
    lasheader.z_scale_factor = 0.0001;

    // Global Shift 还原
    Eigen::Vector3d shift = cloud->getGlobalShift();
    if (!cloud->empty()) {
        lasheader.x_offset = cloud->min().x + shift.x();
        lasheader.y_offset = cloud->min().y + shift.y();
        lasheader.z_offset = cloud->min().z + shift.z();
    }

    LASwriter *laswriter = laswriteopener.open(&lasheader);
    if (!laswriter) return false;

    LASpoint laspoint;
    laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, &lasheader);

    // 遍历所有数据块
    const auto& blocks = cloud->getBlocks();

    // 进度统计
    size_t total_points = cloud->size();
    size_t processed_points = 0;
    int progress_interval = (total_points > 100) ? (total_points / 100) : 1;

    for (const auto& block : blocks) {
        if (block->empty()) continue;

        // 预取当前 Block 的 Intensity 数据 (如果存在)
        const std::vector<float>* intensity_ptr = nullptr;
        if (block->m_scalar_fields.find("Intensity") != block->m_scalar_fields.end()) {
            intensity_ptr = &block->m_scalar_fields["Intensity"];
        }

        size_t n = block->size();
        for (size_t i = 0; i < n; ++i) {
            if (m_is_canceled) {
                laswriter->close(); delete laswriter; return false;
            }

            const auto& p = block->m_points[i];
            laspoint.set_x(p.x + shift.x());
            laspoint.set_y(p.y + shift.y());
            laspoint.set_z(p.z + shift.z());

            if (cloud->hasColors() && block->m_colors) {
                const auto& c = (*block->m_colors)[i];
                laspoint.rgb[0] = c.r * 256; // 8bit -> 16bit
                laspoint.rgb[1] = c.g * 256;
                laspoint.rgb[2] = c.b * 256;
            }

            if (intensity_ptr) {
                laspoint.set_intensity(static_cast<uint16_t>((*intensity_ptr)[i]));
            } else {
                laspoint.set_intensity(0);
            }

            laswriter->write_point(&laspoint);

            // 进度条
            processed_points++;
            if (processed_points % progress_interval == 0) {
                emit progress((int)(processed_points * 100 / total_points));
            }
        }
    }

    laswriter->close();
    delete laswriter;
    return true;
}

// ================================================================
// E57 保存
// ================================================================

bool FileIO::saveE57(const Cloud::Ptr &cloud, const QString &filename) {
    std::string path = filename.toLocal8Bit().toStdString();

    // 恢复原始坐标
    Eigen::Vector3d shift = cloud->getGlobalShift();
    bool has_shift = (shift != Eigen::Vector3d::Zero());

    try {
        e57::Writer writer(path, e57::WriterOptions{});
        if (!writer.IsOpen()) return false;

        emit progress(10);

        size_t total = cloud->size();
        if (total == 0) return false;

        // 配置 Data3D 头
        e57::Data3D header;
        header.name = cloud->id();
        header.pointCount = total;

        // 坐标字段（必须）
        header.pointFields.cartesianXField = true;
        header.pointFields.cartesianYField = true;
        header.pointFields.cartesianZField = true;
        header.pointFields.cartesianInvalidStateField = true;

        // 颜色字段（如果有）
        bool has_color = cloud->hasColors();
        if (has_color) {
            header.pointFields.colorRedField = true;
            header.pointFields.colorGreenField = true;
            header.pointFields.colorBlueField = true;
            header.pointFields.isColorInvalidField = true;
            header.colorLimits.colorRedMinimum = 0;
            header.colorLimits.colorRedMaximum = 65535;
            header.colorLimits.colorGreenMinimum = 0;
            header.colorLimits.colorGreenMaximum = 65535;
            header.colorLimits.colorBlueMinimum = 0;
            header.colorLimits.colorBlueMaximum = 65535;
        }

        // 创建 Data3D
        int64_t data_index = writer.NewData3D(header);

        // 分配缓冲区
        e57::Data3DPointsFloat buffers(header);

        // 设置写入器
        auto cv_writer = writer.SetUpData3DPointsData(data_index, total, buffers);

        emit progress(20);

        // 遍历所有 Block 写入点
        const auto& blocks = cloud->getBlocks();
        size_t written = 0;
        int progress_interval = (total > 100) ? (size_t)(total / 70) : 1;

        float sx = (float)shift.x();
        float sy = (float)shift.y();
        float sz = (float)shift.z();

        for (size_t bi = 0; bi < blocks.size(); ++bi) {
            if (m_is_canceled) { writer.Close(); return false; }

            const auto& block = blocks[bi];
            size_t bsize = block->size();

            for (size_t i = 0; i < bsize; ++i) {
                buffers.cartesianX[i] = block->m_points[i].x + sx;
                buffers.cartesianY[i] = block->m_points[i].y + sy;
                buffers.cartesianZ[i] = block->m_points[i].z + sz;
                buffers.cartesianInvalidState[i] = 0;

                if (has_color && block->m_colors) {
                    auto& c = (*block->m_colors)[i];
                    // E57 颜色是 16 位
                    buffers.colorRed[i] = (uint16_t)c.r << 8;
                    buffers.colorGreen[i] = (uint16_t)c.g << 8;
                    buffers.colorBlue[i] = (uint16_t)c.b << 8;
                    buffers.isColorInvalid[i] = 0;
                }
            }

            // 写入这一批
            cv_writer.write(bsize);
            written += bsize;

            if (written % progress_interval < bsize) {
                int pct = 20 + (int)(70 * written / total);
                emit progress(pct);
            }
        }

        cv_writer.close();
        writer.Close();

        emit progress(100);
        return true;

    } catch (const e57::E57Exception&) {
        return false;
    } catch (...) {
        return false;
    }
}

// ================================================================
// TXT 保存
// ================================================================

bool FileIO::saveTXT(const Cloud::Ptr &cloud, const QString &filename) {
    // 收集可用字段 (Metadata)
    QStringList available_fields;
    available_fields << "x" << "y" << "z";
    if (cloud->hasColors()) available_fields << "r" << "g" << "b";
    if (cloud->hasNormals()) available_fields << "nx" << "ny" << "nz";

    // 获取标量场名称
    auto scalar_names_vec = cloud->getScalarFieldNames();
    for (const auto& name : scalar_names_vec) {
        available_fields.append(QString::fromStdString(name));
    }

    // 请求 UI 配置 (阻塞式)
    ct::TxtExportParams params;
    emit requestTxtExportSetup(available_fields, params);
    if (params.selected_fields.empty()) return false; // 用户取消

    // 打开文件
    std::ofstream file(filename.toLocal8Bit().constData());
    if (!file.is_open()) return false;

    // --- 性能优化：设置 1MB 写入缓冲区 ---
    std::vector<char> buffer(1024 * 1024);
    file.rdbuf()->pubsetbuf(buffer.data(), buffer.size());

    // 设置浮点精度
    file << std::fixed << std::setprecision(params.precision);

    // 写入 Header (可选)
    if (params.has_header) {
        for (int i = 0; i < params.selected_fields.size(); i++){
            file << params.selected_fields[i];
            if (i < params.selected_fields.size() - 1) file << params.separator;
        }
        file << "\n";
    }

    // 准备字段描述符 (避免在循环中进行字符串比较)
    struct FieldDesc {
        enum Type { XYZ, RGB, Normal, Scalar };
        Type type;
        int sub_index; // 0=x/r/nx, 1=y/g/ny, 2=z/b/nz
        QString scalar_name; // 仅 Scalar 类型需要
    };

    std::vector<FieldDesc> field_descs;
    field_descs.reserve(params.selected_fields.size());

    for (const std::string& f : params.selected_fields){
        if (f == "x") field_descs.push_back({FieldDesc::XYZ, 0, ""});
        else if (f == "y") field_descs.push_back({FieldDesc::XYZ, 1, ""});
        else if (f == "z") field_descs.push_back({FieldDesc::XYZ, 2, ""});
        else if (f == "r") field_descs.push_back({FieldDesc::RGB, 0, ""});
        else if (f == "g") field_descs.push_back({FieldDesc::RGB, 1, ""});
        else if (f == "b") field_descs.push_back({FieldDesc::RGB, 2, ""});
        else if (f == "nx") field_descs.push_back({FieldDesc::Normal, 0, ""});
        else if (f == "ny") field_descs.push_back({FieldDesc::Normal, 1, ""});
        else if (f == "nz") field_descs.push_back({FieldDesc::Normal, 2, ""});
        else {
            field_descs.push_back({FieldDesc::Scalar, 0, QString::fromStdString(f)});
        }
    }

    // 遍历所有 Block 进行写入
    const auto& blocks = cloud->getBlocks();
    Eigen::Vector3d shift = cloud->getGlobalShift(); // 用于还原大坐标
    char sep = params.separator;

    size_t total_points = cloud->size();
    size_t processed_points = 0;
    int progress_interval = (total_points > 100) ? (total_points / 100) : 1;

    for (const auto& block : blocks) {
        if (block->empty()) continue;

        size_t n = block->size();

        // --- 关键：预取当前 Block 的数据指针 ---
        const auto& pts = block->m_points;
        const auto* colors = block->m_colors.get(); // 可能为 nullptr
        const auto* normals = block->m_normals.get(); // 可能为 nullptr

        // 为每个 Scalar 字段查找当前 Block 对应的 vector 指针
        std::vector<const std::vector<float>*> scalar_ptrs(field_descs.size(), nullptr);
        for (size_t k = 0; k < field_descs.size(); ++k) {
            if (field_descs[k].type == FieldDesc::Scalar) {
                const std::string sname = field_descs[k].scalar_name.toStdString();
                auto sf_it = block->m_scalar_fields.find(sname);
                if (sf_it != block->m_scalar_fields.end()) {
                    scalar_ptrs[k] = &sf_it->second;
                }
            }
        }

        // --- 内层循环：逐点写入 ---
        for (size_t i = 0; i < n; ++i) {
            if (m_is_canceled) {
                file.close();
                return false;
            }

            // 写入用户选择的每一列
            for (size_t k = 0; k < field_descs.size(); ++k) {
                const auto& fd = field_descs[k];

                switch (fd.type) {
                    case FieldDesc::XYZ: {
                        const auto& p = pts[i];
                        if (fd.sub_index == 0) file << (p.x + shift.x());
                        else if (fd.sub_index == 1) file << (p.y + shift.y());
                        else file << (p.z + shift.z());
                        break;
                    }
                    case FieldDesc::RGB: {
                        if (colors) {
                            const auto& c = (*colors)[i];
                            int val = (fd.sub_index == 0) ? c.r : (fd.sub_index == 1 ? c.g : c.b);
                            file << val;
                        } else {
                            file << 0;
                        }
                        break;
                    }
                    case FieldDesc::Normal: {
                        if (normals) {
                            // 需要解压法线
                            Eigen::Vector3f n_vec = (*normals)[i].get();
                            if (fd.sub_index == 0) file << n_vec.x();
                            else if (fd.sub_index == 1) file << n_vec.y();
                            else file << n_vec.z();
                        } else {
                            file << 0;
                        }
                        break;
                    }
                    case FieldDesc::Scalar: {
                        const auto* vec_ptr = scalar_ptrs[k];
                        if (vec_ptr && i < vec_ptr->size()) {
                            file << (*vec_ptr)[i];
                        } else {
                            file << 0;
                        }
                        break;
                    }
                }

                // 写入分隔符或换行
                if (k < field_descs.size() - 1) file << sep;
            }
            file << "\n";

            // 进度条更新
            processed_points++;
            if (processed_points % 5000 == 0) { // 减少 emit 频率
                if (processed_points % progress_interval == 0) {
                    emit progress((int)(processed_points * 100 / total_points));
                }
            }
        }
    }

    file.close();
    return true;
}

// ================================================================
// PLY / PCD 保存
// ================================================================

bool FileIO::savePCL(const Cloud::Ptr &cloud, const QString &filename, bool isBinary) {
    // 准备 PCL 消息对象
    pcl::PCLPointCloud2 msg;
    msg.height = 1;
    msg.width = cloud->size(); // 总点数
    msg.is_dense = false; // 无法保证所有点都有效，保守设为 false
    msg.is_bigendian = false;

    if (msg.width == 0) return false;

    // 动态定义字段 (Fields)
    int current_offset = 0;

    // --- XYZ ---
    {
        pcl::PCLPointField f;
        f.name = "x"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;
        f.name = "y"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;
        f.name = "z"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;
    }

    // --- RGB ---
    bool has_rgb = cloud->hasColors();
    if (has_rgb) {
        pcl::PCLPointField f;
        f.name = "rgb"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;
    }

    // --- Normals ---
    bool has_normals = cloud->hasNormals();
    if (has_normals) {
        pcl::PCLPointField f;
        f.name = "normal_x"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;
        f.name = "normal_y"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;
        f.name = "normal_z"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;
    }

    // --- Scalar Fields (自定义标量场) ---
    std::vector<std::string> scalar_names = cloud->getScalarFieldNames();
    for (const std::string& name : scalar_names) {
        pcl::PCLPointField f;
        f.name = name;
        f.offset = current_offset;
        f.datatype = pcl::PCLPointField::FLOAT32;
        f.count = 1;
        msg.fields.push_back(f);
        current_offset += 4;
    }

    msg.point_step = current_offset;
    msg.row_step = msg.point_step * msg.width;

    // 分配内存 (可能很大，注意 catch 异常)
    try {
        msg.data.resize(msg.row_step);
    } catch (...) {
        return false; // 内存不足
    }

    // 计算每个 Block 的写入起始位置 (Prefix Sum)
    const auto& blocks = cloud->getBlocks();
    std::vector<size_t> block_offsets(blocks.size(), 0);
    size_t running_offset = 0;
    for (size_t i = 0; i < blocks.size(); ++i) {
        block_offsets[i] = running_offset;
        running_offset += blocks[i]->size();
    }

    // 并行填充数据
    Eigen::Vector3d shift = cloud->getGlobalShift();
    float sx = (float)shift.x();
    float sy = (float)shift.y();
    float sz = (float)shift.z();

    uint8_t* base_ptr = msg.data.data();
    int step = msg.point_step;

    // 辅助：获取字段偏移量 (避免循环中查找)
    int off_rgb = -1;
    int off_nx = -1, off_ny = -1, off_nz = -1;
    std::vector<int> off_scalars;

    // 重新扫描一遍 fields 确定偏移
    for (const auto& f : msg.fields) {
        if (f.name == "rgb") off_rgb = f.offset;
        else if (f.name == "normal_x") off_nx = f.offset;
        else if (f.name == "normal_y") off_ny = f.offset;
        else if (f.name == "normal_z") off_nz = f.offset;
        else {
            // 检查是否是标量场
            if (std::find(scalar_names.begin(), scalar_names.end(), f.name) != scalar_names.end()) {
                off_scalars.push_back(f.offset);
            }
        }
    }

    // 开启 OpenMP 并行处理 Block
    std::atomic<size_t> processed_points_counter(0);
    bool write_failed = false;

#pragma omp parallel for
    for (int k = 0; k < (int)blocks.size(); ++k) {
        if (write_failed || m_is_canceled) continue;

        const auto& block = blocks[k];
        if (block->empty()) continue;

        size_t n = block->size();
        size_t start_idx = block_offsets[k];

        // 数据指针预取
        const auto& pts = block->m_points;
        const auto* colors = (has_rgb && block->m_colors) ? block->m_colors.get() : nullptr;
        const auto* normals = (has_normals && block->m_normals) ? block->m_normals.get() : nullptr;

        // 标量场指针预取
        std::vector<const std::vector<float>*> scalar_ptrs;
        if (!scalar_names.empty()) {
            scalar_ptrs.reserve(scalar_names.size());
            for (const std::string& name : scalar_names) {
                auto sf_it = block->m_scalar_fields.find(name);
                if (sf_it != block->m_scalar_fields.end()) {
                    scalar_ptrs.push_back(&sf_it->second);
                } else {
                    scalar_ptrs.push_back(nullptr); // 缺失字段处理
                }
            }
        }

        // 块内循环
        for (size_t i = 0; i < n; ++i) {
            // 计算当前点在 msg.data 中的内存地址
            uint8_t* pt_ptr = base_ptr + (start_idx + i) * step;

            // --- 写入 XYZ ---
            const auto& p = pts[i];
            float x = p.x + sx;
            float y = p.y + sy;
            float z = p.z + sz;

            // 直接 memcpy 比 reinterpret_cast 更安全，且会被编译器优化
            memcpy(pt_ptr + 0, &x, 4);
            memcpy(pt_ptr + 4, &y, 4);
            memcpy(pt_ptr + 8, &z, 4);

            // --- 写入 RGB ---
            if (has_rgb && off_rgb >= 0) {
                uint32_t rgb_val = 0;
                if (colors) {
                    const auto& c = (*colors)[i];
                    rgb_val = (static_cast<uint32_t>(c.r) << 16) |
                              (static_cast<uint32_t>(c.g) << 8)  |
                              (static_cast<uint32_t>(c.b));
                }
                // 虽然是 float 类型字段，但实际存储的是位填充的 int
                memcpy(pt_ptr + off_rgb, &rgb_val, 4);
            }

            // --- 写入 Normals ---
            if (has_normals && off_nx >= 0) {
                float nx = 0, ny = 0, nz = 1;
                if (normals) {
                    Eigen::Vector3f n_vec = (*normals)[i].get();
                    nx = n_vec.x(); ny = n_vec.y(); nz = n_vec.z();
                }
                memcpy(pt_ptr + off_nx, &nx, 4);
                memcpy(pt_ptr + off_ny, &ny, 4);
                memcpy(pt_ptr + off_nz, &nz, 4);
            }

            // --- 写入 Scalars ---
            for (size_t s_idx = 0; s_idx < scalar_ptrs.size(); ++s_idx) {
                float val = 0.0f;
                if (scalar_ptrs[s_idx] && i < scalar_ptrs[s_idx]->size()) {
                    val = (*scalar_ptrs[s_idx])[i];
                }
                memcpy(pt_ptr + off_scalars[s_idx], &val, 4);
            }
        }

            // 简单的进度更新逻辑
            processed_points_counter += n;
    }

    if (m_is_canceled) return false;

    // 调用 PCL 保存函数
    int res = -1;
    if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
        pcl::PCDWriter writer;
        res = writer.write(filename.toLocal8Bit().toStdString(), msg, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), isBinary);
    }
    else if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
        pcl::PLYWriter writer;
        res = writer.write(filename.toLocal8Bit().toStdString(), msg, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), isBinary, false);
    } else {
        // 默认回退到 PCD
        pcl::PCDWriter writer;
        res = writer.write(filename.toLocal8Bit().toStdString(), msg, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), isBinary);
    }

    return (res == 0);
}

} // namespace ct
