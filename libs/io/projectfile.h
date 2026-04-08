#ifndef POINTWORKS_PROJECTFILE_H
#define POINTWORKS_PROJECTFILE_H

#include "exports.h"
#include <QString>
#include <QStringList>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <Eigen/Dense>

namespace ct
{

/// 单个点云的元数据
struct CT_IO_EXPORT CloudEntry
{
    QString uuid;            // Cloud::id()
    QString file_path;       // 原始文件路径
    QString display_name;    // 树节点显示名
    Eigen::Vector3d global_shift;
    QString color_mode;      // Cloud::currentColorMode()
    int point_size = 1;
    float opacity = 1.0f;
    bool is_visible = true;

    QJsonObject toJson() const;
    static CloudEntry fromJson(const QJsonObject& obj);
};

/// 树节点（文件、点云或分组）
struct CT_IO_EXPORT TreeNode
{
    QString type;          // "file", "cloud", "group"
    QString text;          // 显示名
    QString uuid;          // Cloud::id()（仅 cloud 类型）
    QString filepath;      // 原始文件路径（仅 file 类型）
    bool expanded = false;
    bool is_visible = true;
    QList<TreeNode> children;

    QJsonObject toJson() const;
    static TreeNode fromJson(const QJsonObject& obj);
};

/// 相机参数（纯 struct，无 JSON 依赖放在 viz 层）
struct CT_IO_EXPORT CameraParams
{
    double position[3] = {0, 0, 0};
    double focal_point[3] = {0, 0, 0};
    double view_up[3] = {0, 0, 1};
    double clip_near = 0.01;
    double clip_far = 100000.0;
};

/// 视图选项
struct CT_IO_EXPORT ViewOptions
{
    bool show_fps = true;
    bool show_axes = true;
    bool show_id = true;

    QJsonObject toJson() const;
    static ViewOptions fromJson(const QJsonObject& obj);
};

/// 项目数据
struct CT_IO_EXPORT ProjectData
{
    QString version = "1.0";
    QString app_name = "PointWorks";
    QDateTime created_at;
    QDateTime modified_at;

    QList<CloudEntry> clouds;
    QList<TreeNode> tree_roots;
    CameraParams camera;
    ViewOptions view_options;
};

/// 项目文件读写
class CT_IO_EXPORT ProjectFile
{
public:
    /// 保存项目到 JSON 文件
    static bool save(const QString& path, const ProjectData& data);

    /// 从 JSON 文件加载项目
    static bool load(const QString& path, ProjectData& data);

    /// 路径解析：绝对路径有效则用绝对，否则基于项目目录解析相对路径
    static QString resolveFilePath(const QString& projectDir, const QString& filePath);

    /// 将绝对路径转为相对于基准目录的相对路径
    static QString toRelativePath(const QString& baseDir, const QString& absolutePath);

    /// 项目文件扩展名
    static QString fileFilter() { return "PointWorks Project (*.ctp)"; }
    static QString defaultSuffix() { return "ctp"; }
};

} // namespace ct

#endif // POINTWORKS_PROJECTFILE_H
