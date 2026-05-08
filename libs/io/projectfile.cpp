#include "projectfile.h"
#include <QJsonDocument>
#include <QFile>
#include <QFileInfo>
#include <QDir>

namespace pw
{

// ================================================================
// CloudEntry
// ================================================================

QJsonObject CloudEntry::toJson() const
{
    QJsonObject obj;
    obj["uuid"] = uuid;
    obj["file_path"] = file_path;
    obj["display_name"] = display_name;
    obj["color_mode"] = color_mode;
    obj["point_size"] = point_size;
    obj["opacity"] = opacity;
    obj["is_visible"] = is_visible;

    QJsonObject shift;
    shift["x"] = global_shift.x();
    shift["y"] = global_shift.y();
    shift["z"] = global_shift.z();
    obj["global_shift"] = shift;

    // 扩展元数据
    obj["has_mesh"] = has_mesh;
    obj["has_colors"] = has_colors;
    obj["has_normals"] = has_normals;
    if (!cloud_type.isEmpty()) obj["cloud_type"] = cloud_type;
    if (!scalar_fields.isEmpty()) obj["scalar_fields"] = QJsonArray::fromStringList(scalar_fields);

    return obj;
}

CloudEntry CloudEntry::fromJson(const QJsonObject& obj)
{
    CloudEntry e;
    e.uuid = obj["uuid"].toString();
    e.file_path = obj["file_path"].toString();
    e.display_name = obj["display_name"].toString();
    e.color_mode = obj["color_mode"].toString();
    e.point_size = obj["point_size"].toInt(1);
    e.opacity = static_cast<float>(obj["opacity"].toDouble(1.0));
    e.is_visible = obj["is_visible"].toBool(true);

    QJsonObject shift = obj["global_shift"].toObject();
    e.global_shift = Eigen::Vector3d(
        shift["x"].toDouble(0), shift["y"].toDouble(0), shift["z"].toDouble(0));

    // 扩展元数据
    e.has_mesh = obj["has_mesh"].toBool(false);
    e.has_colors = obj["has_colors"].toBool(false);
    e.has_normals = obj["has_normals"].toBool(false);
    e.cloud_type = obj["cloud_type"].toString();

    QJsonArray sf = obj["scalar_fields"].toArray();
    for (const auto& s : sf)
        e.scalar_fields.append(s.toString());

    return e;
}

// ================================================================
// TreeNode
// ================================================================

QJsonObject TreeNode::toJson() const
{
    QJsonObject obj;
    obj["type"] = type;
    obj["text"] = text;
    obj["expanded"] = expanded;
    obj["is_visible"] = is_visible;

    if (!uuid.isEmpty()) obj["uuid"] = uuid;
    if (!filepath.isEmpty()) obj["filepath"] = filepath;

    QJsonArray childArray;
    for (const auto& c : children)
        childArray.append(c.toJson());
    obj["children"] = childArray;

    return obj;
}

TreeNode TreeNode::fromJson(const QJsonObject& obj)
{
    TreeNode n;
    n.type = obj["type"].toString();
    n.text = obj["text"].toString();
    n.expanded = obj["expanded"].toBool(false);
    n.is_visible = obj["is_visible"].toBool(true);
    n.uuid = obj["uuid"].toString();
    n.filepath = obj["filepath"].toString();

    QJsonArray children = obj["children"].toArray();
    for (const auto& c : children) {
        n.children.append(TreeNode::fromJson(c.toObject()));
    }

    return n;
}

// ================================================================
// ViewOptions 序列化（自由函数，供 ProjectFile 使用）
// ================================================================

static QJsonObject viewOptionsToJson(const ViewOptions& opts)
{
    QJsonObject obj;
    obj["show_fps"] = opts.show_fps;
    obj["show_axes"] = opts.show_axes;
    obj["show_id"] = opts.show_id;

    // 背景色
    obj["use_gradient_bg"] = opts.use_gradient_bg;
    QJsonArray bg, bg2;
    bg.append(opts.bg_color[0]); bg.append(opts.bg_color[1]); bg.append(opts.bg_color[2]);
    bg2.append(opts.bg_color2[0]); bg2.append(opts.bg_color2[1]); bg2.append(opts.bg_color2[2]);
    obj["bg_color"] = bg;
    obj["bg_color2"] = bg2;

    return obj;
}

static ViewOptions viewOptionsFromJson(const QJsonObject& obj)
{
    ViewOptions o;
    o.show_fps = obj["show_fps"].toBool(true);
    o.show_axes = obj["show_axes"].toBool(true);
    o.show_id = obj["show_id"].toBool(true);

    // 背景色
    o.use_gradient_bg = obj["use_gradient_bg"].toBool(true);
    QJsonArray bg = obj["bg_color"].toArray();
    if (bg.size() == 3) {
        o.bg_color[0] = bg[0].toDouble();
        o.bg_color[1] = bg[1].toDouble();
        o.bg_color[2] = bg[2].toDouble();
    }
    QJsonArray bg2 = obj["bg_color2"].toArray();
    if (bg2.size() == 3) {
        o.bg_color2[0] = bg2[0].toDouble();
        o.bg_color2[1] = bg2[1].toDouble();
        o.bg_color2[2] = bg2[2].toDouble();
    }

    return o;
}

// ================================================================
// ProjectFile
// ================================================================

bool ProjectFile::save(const QString& path, const ProjectData& data)
{
    QJsonObject root;
    root["version"] = data.version;
    root["app_name"] = data.app_name;
    root["created_at"] = data.created_at.toString(Qt::ISODate);
    root["modified_at"] = data.modified_at.toString(Qt::ISODate);

    // cloud entries
    QJsonArray clouds;
    for (const auto& c : data.clouds)
        clouds.append(c.toJson());
    root["cloud_entries"] = clouds;

    // tree structure
    QJsonArray roots;
    for (const auto& r : data.tree_roots)
        roots.append(r.toJson());
    root["tree_roots"] = roots;

    // camera
    QJsonObject cam;
    QJsonArray pos, focal, up;
    pos.append(data.camera.position[0]);
    pos.append(data.camera.position[1]);
    pos.append(data.camera.position[2]);
    focal.append(data.camera.focal_point[0]);
    focal.append(data.camera.focal_point[1]);
    focal.append(data.camera.focal_point[2]);
    up.append(data.camera.view_up[0]);
    up.append(data.camera.view_up[1]);
    up.append(data.camera.view_up[2]);
    cam["position"] = pos;
    cam["focal_point"] = focal;
    cam["view_up"] = up;
    cam["clip_near"] = data.camera.clip_near;
    cam["clip_far"] = data.camera.clip_far;
    root["camera"] = cam;

    // view options
    root["view_options"] = viewOptionsToJson(data.view_options);

    QJsonDocument doc(root);
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    file.write(doc.toJson(QJsonDocument::Indented));
    return true;
}

bool ProjectFile::load(const QString& path, ProjectData& data)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &err);
    if (err.error != QJsonParseError::NoError)
        return false;

    QJsonObject root = doc.object();
    data.version = root["version"].toString("1.0");
    data.app_name = root["app_name"].toString();
    data.created_at = QDateTime::fromString(root["created_at"].toString(), Qt::ISODate);
    data.modified_at = QDateTime::fromString(root["modified_at"].toString(), Qt::ISODate);

    // clouds
    QJsonArray clouds = root["cloud_entries"].toArray();
    for (const auto& c : clouds)
        data.clouds.append(CloudEntry::fromJson(c.toObject()));

    // tree
    QJsonArray roots = root["tree_roots"].toArray();
    for (const auto& r : roots)
        data.tree_roots.append(TreeNode::fromJson(r.toObject()));

    // camera
    QJsonObject cam = root["camera"].toObject();
    QJsonArray pos = cam["position"].toArray();
    QJsonArray focal = cam["focal_point"].toArray();
    QJsonArray up = cam["view_up"].toArray();
    if (pos.size() == 3) {
        data.camera.position[0] = pos[0].toDouble();
        data.camera.position[1] = pos[1].toDouble();
        data.camera.position[2] = pos[2].toDouble();
    }
    if (focal.size() == 3) {
        data.camera.focal_point[0] = focal[0].toDouble();
        data.camera.focal_point[1] = focal[1].toDouble();
        data.camera.focal_point[2] = focal[2].toDouble();
    }
    if (up.size() == 3) {
        data.camera.view_up[0] = up[0].toDouble();
        data.camera.view_up[1] = up[1].toDouble();
        data.camera.view_up[2] = up[2].toDouble();
    }
    data.camera.clip_near = cam["clip_near"].toDouble(0.01);
    data.camera.clip_far = cam["clip_far"].toDouble(100000.0);

    // view options
    data.view_options = viewOptionsFromJson(root["view_options"].toObject());

    return true;
}

QString ProjectFile::resolveFilePath(const QString& projectDir, const QString& filePath)
{
    QFileInfo fi(filePath);
    if (fi.isAbsolute() && fi.exists())
        return filePath;

    QString relative = projectDir + "/" + filePath;
    if (QFileInfo::exists(relative))
        return QDir::cleanPath(relative);

    // fallback: 尝试只取文件名在项目目录下搜索
    QString nameOnly = projectDir + "/" + fi.fileName();
    if (QFileInfo::exists(nameOnly))
        return QDir::cleanPath(nameOnly);

    return filePath; // 原样返回，让调用者处理缺失
}

QString ProjectFile::toRelativePath(const QString& baseDir, const QString& absolutePath)
{
    QDir base(baseDir);
    QString rel = base.relativeFilePath(absolutePath);
    // 如果无法转换为相对路径（不同盘符等），返回绝对路径
    if (rel.startsWith(".."))
        return absolutePath;
    return rel;
}

} // namespace pw
