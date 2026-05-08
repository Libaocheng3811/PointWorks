#ifndef POINTWORKS_CLOUD_H
#define POINTWORKS_CLOUD_H

#include "cloudtype.h"
#include "octree.h"
#include "exports.h"
#include "colormap.h"

#include <string>
#include <unordered_map>
#include <random>
#include <memory>
#include <vector>

#define CLOUD_TYPE_XYZ       "xyz"
#define CLOUD_TYPE_XYZRGB    "XYZRGB"
#define CLOUD_TYPE_XYZN      "XYZNormal"
#define CLOUD_TYPE_XYZRGBN   "XYZRGBNormal"

#define BOX_PRE_FLAG         "-box"
#define NORMALS_PRE_FLAG     "-normals"

namespace pw
{
    class PW_EXPORT Cloud :public std::enable_shared_from_this<Cloud>
    {
    public:
        using Ptr = std::shared_ptr<Cloud>;
        using ConstPtr = std::shared_ptr<const Cloud>;

        Cloud();
        ~Cloud();

        // 禁用拷贝，启用移动
        Cloud(const Cloud& other) = delete;
        Cloud& operator=(const Cloud& other) = delete;
        Cloud(Cloud&& other) = delete;
        Cloud& operator=(Cloud&& other) = delete;

        void swap(Cloud& other);

        // ===== 配置接口 =====
        static CloudConfig calculateAdaptiveConfig(size_t totalPoints);

        void makeAdaptive();

        void setConfig(const CloudConfig& config) { m_config = config; }
        const CloudConfig& getConfig() const { return m_config; }

        // ===== 八叉树构建与数据加载 =====
        void initOctree(const Box& globalBox);

        void addPoint(const PointXYZ& pt, const ColorRGB* color = nullptr, const CompressedNormal* normal = nullptr);

        void addPoints(const std::vector<PointXYZ>& pts,
                       const std::vector<ColorRGB>* colors = nullptr,
                       const std::vector<CompressedNormal>* normals = nullptr,
                       const std::unordered_map<std::string, std::vector<float>>* scalars = nullptr);

        void addPoint(const PointXYZRGBN& pt);

        void generateLOD();

        // ===== 核心访问接口 =====
        const std::vector<CloudBlock::Ptr>& getBlocks() const { return m_all_blocks; }
        OctreeNode* getOctreeRoot() const { return m_octree_root.get(); }

        /**
         * @brief 查找包含指定坐标的叶子 block（用于拾取时获取标量场数据）
         */
        CloudBlock* findBlockForPoint(float x, float y, float z) const;

        /**
         * @brief 获取第一个点坐标（安全访问，无需了解 block 布局）
         * @return true 成功获取，false 点云为空
         */
        bool getFirstPoint(PointXYZ& out) const;

        // ===== 容量接口 =====
        size_t size() const;
        bool empty() const;
        void clear();

        // ===== 属性管理 =====
        bool hasColors() const { return m_has_rgb; };
        bool hasNormals() const { return m_has_normals; };
        void setHasColors(bool has_colors) { m_has_rgb = has_colors; };
        void setHasNormals(bool has_normals) { m_has_normals = has_normals; };

        void enableColors();
        void enableNormals();
        void disableColors();
        void disableNormals();

        // ===== 标量场管理  =====
        void addScalarField(const std::string& name, const std::vector<float>& data);
        bool removeScalarField(const std::string& name);
        void clearScalarFields();
        bool hasScalarField(const std::string& name) const;
        std::vector<std::string> getScalarFieldNames() const;
        const std::vector<float>* getScalarField(const std::string& name) const;

        void updateColorByField(const std::string& field_name);

        void updateColorByField(const std::string& field_name,
                                float display_min, float display_max,
                                ColormapType colormap = ColormapType::JET,
                                bool show_nan_as_grey = true);

        bool getFieldRange(const std::string& field_name,
                           float& out_min, float& out_max) const;

        // ===== 颜色操作 =====
        void setCloudColor(const ColorRGB& rgb);
        void setCloudColor(const std::string& axis);
        void updateLODColorRecursive(OctreeNode* node, const ColorRGB& rgb);
        const std::string& currentColorMode() const { return m_current_color_mode; }

        void backupColors();
        void restoreColors();
        bool isColorModified() const { return m_color_modified; }

        // ===== PCL 兼容导出 =====
        pcl::PointCloud<PointXYZ>::Ptr toPCL_XYZ() const;
        pcl::PointCloud<PointXYZRGB>::Ptr toPCL_XYZRGB() const;
        pcl::PointCloud<PointXYZRGBN>::Ptr toPCL_XYZRGBN() const;

        // 按全局索引列表提取子集（避免全量深拷贝）
        pcl::PointCloud<PointXYZRGB>::Ptr toPCL_XYZRGB(const std::vector<int>& indices) const;

        // 按全局索引从原始 Cloud 提取子集，保留颜色/法线/标量场（内存高效，避免 PCL 中间转换）
        Ptr extractByIndices(const std::vector<int>& indices) const;

        static Ptr fromPCL_XYZRGBN(const pcl::PointCloud<PointXYZRGBN>& pcl_cloud,
                                    const Eigen::Vector3d& global_shift = Eigen::Vector3d::Zero());
        static Ptr fromPCL_XYZRGB(const pcl::PointCloud<PointXYZRGB>& pcl_cloud,
                                  const Eigen::Vector3d& global_shift = Eigen::Vector3d::Zero());
        static Ptr fromPCL_XYZ(const pcl::PointCloud<PointXYZ>& pcl_cloud,
                               const Eigen::Vector3d& global_shift = Eigen::Vector3d::Zero());

        // ===== 几何与元数据 =====
        void update();

        const std::string& id() const {return m_id;}
        void setId(const std::string& id) {m_id = id;}

        std::string boxId() const {return m_id + BOX_PRE_FLAG;}
        std::string normalId() const {return m_id + NORMALS_PRE_FLAG;}

        Box box() const {return m_box;}
        void setBox(const Box& box) {m_box = box;}

        ColorRGB boxColor() const {return m_box_rgb;}
        void setBoxColor(const ColorRGB& rgb) {m_box_rgb = rgb;}

        ColorRGB normalColor() const {return m_normals_rgb;}
        void setNormalColor(const ColorRGB& rgb) {m_normals_rgb = rgb;}

        const std::string& filepath() const {return m_filepath;}
        void setFilepath(const std::string& filepath) {m_filepath = filepath;}

        int pointSize() const {return m_point_size;}
        void setPointSize(int size) {m_point_size = size;}

        float opacity() const {return m_opacity;}
        void setOpacity(float opacity) {m_opacity = opacity;}

        float resolution() const {return m_resolution;}
        void setResolution(float res) { m_resolution = res; }

        const std::string& type() const {return m_type;}

        Eigen::Vector3f center() const {return m_box.translation;}

        PointXYZ min() const {return m_min;}
        PointXYZ max() const {return m_max;}

        void setGlobalShift(const Eigen::Vector3d& shift) { m_global_shift = shift; }
        Eigen::Vector3d getGlobalShift() const { return m_global_shift; }

        double volume() const { return m_box.width * m_box.height * m_box.depth; }

        ////////////////////////////////////////////////////////////////////
       // 算法接口
        Ptr clone() const;

        void append(const Cloud& cloud);

        Cloud& operator +=(const Cloud& cloud);

        std::uint32_t getPointColorForSave(size_t index) const;

        void removeInvalidPoints();
        void invalidateCache();
        void invalidatePCLCache();
        void invalidateScalarCache();

    private:
        float computeResolution(int sampleCount = 1000);
        void initColorTable();
        void copyFrom(const Cloud& other);
        CloudBlock* insertPointToOctree(OctreeNode* node, const PointXYZ& pt,
                                 const ColorRGB* color, const CompressedNormal* normal);

        void splitNode(OctreeNode* node);

        void updateRecursive(OctreeNode* node, PointXYZ& min_pt, PointXYZ& max_pt, size_t& count);

        std::unique_ptr<OctreeNode> cloneOctreeRecursive(const OctreeNode* src_node, OctreeNode* parent);

        void generateLODRecursive(OctreeNode* node);

        void refreshLODColorsFromBlocks(OctreeNode* node);

        bool isStructureSplit() const;
    private:
        // ===== 核心数据（私有）=====
        CloudConfig m_config;
        OctreeNode::Ptr m_octree_root;
        std::vector<CloudBlock::Ptr> m_all_blocks;
        size_t m_point_count = 0;
        int m_max_depth = 8;

        OctreeNode* m_last_insert_node = nullptr;

        mutable std::unordered_map<std::string, std::vector<float>> m_scalar_cache;

        mutable pcl::PointCloud<PointXYZ>::Ptr m_cached_xyz;
        mutable pcl::PointCloud<PointXYZRGB>::Ptr m_cached_xyzrgb;
        mutable pcl::PointCloud<PointXYZRGBN>::Ptr m_cached_xyzrgbn;

        // ===== 元数据 =====
        std::string m_id;
        std::string m_filepath;
        Box m_box;

        ColorRGB m_box_rgb;
        ColorRGB m_normals_rgb;

        int m_point_size = 1;
        float m_opacity = 1.0f;
        float m_resolution = 0.0f;

        PointXYZ m_min;
        PointXYZ m_max;

        Eigen::Vector3d m_global_shift = Eigen::Vector3d::Zero();

        bool m_has_rgb = false;
        bool m_has_normals = false;
        std::string m_type = CLOUD_TYPE_XYZ;

        bool m_color_modified = false;
        std::string m_current_color_mode = "RGB (Default)";

        static std::vector<float> s_jet_lut;

        mutable std::mt19937 m_rng{std::random_device{}()};
    };
}

#endif //POINTWORKS_CLOUD_H
