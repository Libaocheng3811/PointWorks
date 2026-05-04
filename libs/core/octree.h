#ifndef POINTWORKS_OCTREE_H
#define POINTWORKS_OCTREE_H

#include "cloudtype.h"
#include <vector>
#include <memory>
#include <array>
#include <unordered_map>
#include <string>

namespace ct{

    struct LODPoint {
        float x, y, z;
        uint8_t r, g, b;
        uint8_t _pad;  // 显式填充到 16B，避免编译器隐式 padding
    };

    /**
     * @brief 数据块 (Data Block) - 八叉树叶子负载
     */
    class CloudBlock
    {
    public:
        using Ptr = std::shared_ptr<CloudBlock>;

        CloudBlock() {
            m_box.width = m_box.height = m_box.depth = 0;
        }

        // --- 核心数据存储 ---
        std::vector<pcl::PointXYZ> m_points;
        std::unique_ptr<std::vector<ct::ColorRGB>> m_colors;
        std::unique_ptr<std::vector<ct::ColorRGB>> m_backup_colors;
        std::unique_ptr<std::vector<ct::CompressedNormal>> m_normals;
        std::unordered_map<std::string, std::vector<float>> m_scalar_fields;

        // 标量场指针缓存，避免 addPoint 逐点哈希查找
        std::vector<std::pair<std::string, std::vector<float>*>> m_scalar_ptrs;
        void rebuildScalarPtrCache() {
            m_scalar_ptrs.clear();
            m_scalar_ptrs.reserve(m_scalar_fields.size());
            for (auto& kv : m_scalar_fields)
                m_scalar_ptrs.emplace_back(kv.first, &kv.second);
        }

        // --- 空间属性 ---
        Box m_box;

        // --- 渲染状态 ---
        bool m_is_visible = true;
        bool m_is_dirty = true;                 // 标记是否需要重新构建 VTK 缓存

        // --- 辅助方法 ---
        size_t size() const { return m_points.size(); }
        bool empty() const { return m_points.empty(); }

        void addPoint(const pcl::PointXYZ& pt) {
            m_points.push_back(pt);
            if (m_colors) m_colors->push_back(ct::Color::White);

            if (m_normals) m_normals->push_back(ct::CompressedNormal());

            if (!m_scalar_ptrs.empty()) {
                for (auto& [name, ptr] : m_scalar_ptrs)
                    ptr->push_back(0.0f);
            }
            m_is_dirty = true;
        }

        void addPoint(const pcl::PointXYZ& pt,
                      const ct::ColorRGB* color,
                      const ct::CompressedNormal* normal,
                      const std::unordered_map<std::string, float>* scalars = nullptr)
        {
            m_points.push_back(pt);

            if (color && !m_colors) {
                m_colors = std::make_unique<std::vector<ColorRGB>>();
                if (!m_points.empty()) {
                    m_colors->resize(m_points.size() - 1, ct::Color::White);
                }
            }
            if (m_colors){
                m_colors->push_back(color ? *color : ct::Color::White);
            }

            if (normal && !m_normals) {
                m_normals = std::make_unique<std::vector<CompressedNormal>>();
                if (!m_points.empty()) {
                    m_normals->resize(m_points.size() - 1);
                }
            }
            if (m_normals) {
                m_normals->push_back(normal ? *normal : ct::CompressedNormal());
            }

            // 同步标量场（使用预缓存指针避免逐点哈希）
            if (!m_scalar_ptrs.empty()) {
                for (auto& [name, ptr] : m_scalar_ptrs) {
                    float val = 0.0f;
                    if (scalars) {
                        auto sc_it = scalars->find(name);
                        if (sc_it != scalars->end()) val = sc_it->second;
                    }
                    ptr->push_back(val);
                }
            }
            m_is_dirty = true;
        }

        void clear() {
            m_points.clear();

            if (m_colors) m_colors->clear();
            if (m_normals) m_normals->clear();
            if (m_backup_colors) m_backup_colors->clear();
            for (auto it = m_scalar_fields.begin(); it != m_scalar_fields.end(); ++it) {
                it->second.clear();
            }

            m_is_dirty = true;
        }

        void registerScalarField(const std::string& name) {
            if (m_scalar_fields.find(name) == m_scalar_fields.end()) {
                std::vector<float>& vec = m_scalar_fields[name];

                if (!m_points.empty()) {
                    vec.resize(m_points.size(), 0.0f);
                }
                rebuildScalarPtrCache();
            }
        }

        void markDirty() { m_is_dirty = true; }
    };

    /**
     * @brief 八叉树节点 (Octree Node)
     */
    class OctreeNode
    {
    public:
        using Ptr = std::shared_ptr<OctreeNode>;

        OctreeNode(const Box& box, int depth, OctreeNode* parent = nullptr)
                : m_box(box), m_depth(depth), m_parent(parent)
        {
        }

        ~OctreeNode() = default;

        OctreeNode* m_parent = nullptr;
        std::unique_ptr<OctreeNode> m_children[8];

        Box m_box;
        int m_depth;
        size_t m_total_points_in_node = 0;

        CloudBlock::Ptr m_block = nullptr;

        std::vector<LODPoint> m_lod_points;

        std::shared_ptr<void> m_vtk_lod_polydata;  // VTK LOD 缓存（由 OctreeRenderer 管理）
        bool m_lod_dirty = true;

        void clearLOD() {
            m_lod_points.clear();
            m_lod_points.shrink_to_fit();
            m_vtk_lod_polydata.reset();
            m_lod_dirty = true;
        }

        bool isLeaf() const { return m_block != nullptr; }

        bool hasChildren() const {
            for(int i=0; i<8; ++i) if(m_children[i]) return true;
            return false;
        }

        int getChildIndex(const pcl::PointXYZ& pt) const {
            float cx = m_box.translation.x();
            float cy = m_box.translation.y();
            float cz = m_box.translation.z();
            int index = 0;
            if (pt.x >= cx) index |= 1;
            if (pt.y >= cy) index |= 2;
            if (pt.z >= cz) index |= 4;
            return index;
        }
    };
} // namespace ct

#endif //POINTWORKS_OCTREE_H
