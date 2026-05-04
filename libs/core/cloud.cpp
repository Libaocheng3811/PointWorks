#include "cloud.h"
#include "common.h"

#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <random>
#include <numeric>
#include <queue>
#include <algorithm>
#include <limits>
#include <omp.h>

namespace ct
{
    std::vector<float> Cloud::s_jet_lut;

    /**
    * @brief 辅助函数：计算 vector<PointXYZ> 的包围盒
    * @note 弥补 PCL 缺少对原生 vector 支持的缺陷
    */
    void getMinMax3D(const std::vector<PointXYZ>& pts, PointXYZ& min_pt, PointXYZ& max_pt)
    {
        if (pts.empty()) {
            min_pt = max_pt = PointXYZ(0, 0, 0);
            return;
        }

        // 初始化为极值
        min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
        max_pt.x = max_pt.y = max_pt.z = -FLT_MAX;

        // 简单的线性遍历
        for (const auto& p : pts) {
            if (p.x < min_pt.x) min_pt.x = p.x;
            if (p.x > max_pt.x) max_pt.x = p.x;
            if (p.y < min_pt.y) min_pt.y = p.y;
            if (p.y > max_pt.y) max_pt.y = p.y;
            if (p.z < min_pt.z) min_pt.z = p.z;
            if (p.z > max_pt.z) max_pt.z = p.z;
        }
    }

    // ===== 构造/析构 =====
    Cloud::Cloud()
        : m_point_count(0),
          m_max_depth(8),
          m_id("cloud"),
          m_box_rgb(Color::White),
          m_normals_rgb(Color::White)
    {
    }

    Cloud::~Cloud() = default;

    void Cloud::swap(ct::Cloud &other) {
        // 使用 std::swap 交换所有成员
        std::swap(m_octree_root, other.m_octree_root);
        std::swap(m_all_blocks, other.m_all_blocks);
        std::swap(m_point_count, other.m_point_count);
        std::swap(m_max_depth, other.m_max_depth);
        std::swap(m_scalar_cache, other.m_scalar_cache);

        // 渲染缓存
        std::swap(m_cached_xyz, other.m_cached_xyz);
        std::swap(m_cached_xyzrgb, other.m_cached_xyzrgb);
        std::swap(m_cached_xyzrgbn, other.m_cached_xyzrgbn);
        std::swap(m_cache_type, other.m_cache_type);

        // 元数据
        std::swap(m_id, other.m_id);
        std::swap(m_filepath, other.m_filepath);
        std::swap(m_box, other.m_box);
        std::swap(m_box_rgb, other.m_box_rgb);
        std::swap(m_normals_rgb, other.m_normals_rgb);
        std::swap(m_point_size, other.m_point_size);
        std::swap(m_opacity, other.m_opacity);
        std::swap(m_resolution, other.m_resolution);
        std::swap(m_min, other.m_min);
        std::swap(m_max, other.m_max);
        std::swap(m_global_shift, other.m_global_shift);
        std::swap(m_has_rgb, other.m_has_rgb);
        std::swap(m_has_normals, other.m_has_normals);
        std::swap(m_type, other.m_type);
        std::swap(m_color_modified, other.m_color_modified);
    }

    CloudConfig Cloud::calculateAdaptiveConfig(size_t totalPoints)
    {
        CloudConfig config;

        // 第一级：直通模式 (Passthrough)
        // 如果点数极少 (< 200万)，不需要八叉树，直接当做一个大块处理
        if (totalPoints < ct::AutoOctreeConfig::MIN_POINTS_FOR_OCTREE) {
            config.enableOctree = false;
            config.maxPointsPerBlock = totalPoints + 1000; // 确保不分裂
            config.maxDepth = 0;
            config.maxLODPoints = 0; // 没有 LOD
            return config;
        }

        config.enableOctree = true;

        // 第二级：分块大小自适应
        // 目标：让叶子节点数量保持在 1000 ~ 5000 之间，以平衡剔除效率和 DrawCall
        const size_t targetBlockCount = ct::AutoOctreeConfig::TARGET_BLOCK_COUNT;
        size_t idealBlockSize = totalPoints / targetBlockCount;

        // 钳位到硬件友好区间 [10k, 500k]
        if (idealBlockSize < ct::AutoOctreeConfig::MIN_BLOCK_SIZE) idealBlockSize = ct::AutoOctreeConfig::MIN_BLOCK_SIZE;
        if (idealBlockSize > ct::AutoOctreeConfig::MAX_BLOCK_SIZE) idealBlockSize = ct::AutoOctreeConfig::MAX_BLOCK_SIZE;

        config.maxPointsPerBlock = idealBlockSize;

        // 第三级：LOD 密度自适应
        // LOD 点数通常是 Block 点数的 20% ~ 50%
        // 如果 Block 很大，LOD 也要大，否则远看会变空
        config.maxLODPoints = static_cast<size_t>(idealBlockSize * ct::AutoOctreeConfig::LOD_POINT_RATIO);

        // LOD 硬上限 (防止单个 DrawCall 过大)
        if (config.maxLODPoints > ct::AutoOctreeConfig::MAX_LOD_SIZE) config.maxLODPoints = ct::AutoOctreeConfig::MAX_LOD_SIZE;
        if (config.maxLODPoints < ct::AutoOctreeConfig::MIN_LOD_SIZE) config.maxLODPoints = ct::AutoOctreeConfig::MIN_LOD_SIZE;

        // 估算深度
        // 简单的对数估算: log8(Total / BlockSize)
        // 但通常 8 层足够应对大部分场景
        config.maxDepth = ct::AutoOctreeConfig::DEFAULT_MAX_DEPTH;

        return config;
    }

    void Cloud::makeAdaptive()
    {
        // 1. 确保基础统计数据（如点数、包围盒）是最新的
        this->update();

        size_t count = this->size();
        if (count == 0) return;

        // 2. 调用静态算法计算建议配置
        CloudConfig config = Cloud::calculateAdaptiveConfig(count);

        // 检查现状：如果内存里的树结构已经是分裂状态（说明算法构建时产生了八叉树），
        // 那么即使点数很少（比如200万），也必须强制启用 Octree 和 LOD，
        // 否则渲染器遍历到内部节点时会因为 maxLODPoints=0 而显示空洞。
        if (isStructureSplit()) {
            if (!config.enableOctree) {
                // 强制修正为八叉树模式
                config.enableOctree = true;
                config.maxDepth = ct::AutoOctreeConfig::DEFAULT_MAX_DEPTH; // 恢复默认深度

                config.maxPointsPerBlock = ct::AutoOctreeConfig::DEFAULT_BLOCK_SIZE;
            }

            size_t calcLOD = static_cast<size_t>(config.maxPointsPerBlock * ct::AutoOctreeConfig::LOD_POINT_RATIO);

            // 应用上下限钳位 (Clamping)
            if (calcLOD < ct::AutoOctreeConfig::MIN_LOD_SIZE) calcLOD = ct::AutoOctreeConfig::MIN_LOD_SIZE;
            if (calcLOD > ct::AutoOctreeConfig::MAX_LOD_SIZE) calcLOD = ct::AutoOctreeConfig::MAX_LOD_SIZE;

            config.maxLODPoints = calcLOD;
        }

        // 4. 应用配置
        this->setConfig(config);

        // 5. 生成 LOD (仅当需要时)
        if (config.enableOctree && config.maxLODPoints > 0) {
            this->generateLOD();
        }

        // 6. 全局 shrink_to_fit — 回收 reserve 造成的 capacity 浪费
        for (auto& block : m_all_blocks) {
            if (block->empty()) continue;
            block->m_points.shrink_to_fit();
            if (block->m_colors) block->m_colors->shrink_to_fit();
            if (block->m_normals) block->m_normals->shrink_to_fit();
            for (auto& kv : block->m_scalar_fields) {
                kv.second.shrink_to_fit();
            }
        }

        // 7. 再次更新以确保状态一致
        this->update();
    }

    void Cloud::initOctree(const ct::Box &globalBox) {
        // 清空现有数据
        clear();

        m_box = globalBox; // 设置全局包围盒

        // 创建根节点
        m_octree_root = std::make_shared<OctreeNode>(globalBox, 0, nullptr);

        // 根节点初始时也是叶子节点，拥有一个空的 Block
        m_octree_root->m_block = std::make_shared<CloudBlock>();
        m_octree_root->m_block->m_points.reserve(m_config.maxPointsPerBlock);

        // 记录到扁平化列表中
        m_all_blocks.push_back(m_octree_root->m_block);
    }

    void Cloud::addPoint(const PointXYZ& pt, const ColorRGB* color, const CompressedNormal* normal)
    {
        if (!m_octree_root) {
            Box defaultBox;
            defaultBox.width = 10000.0; defaultBox.height = 10000.0; defaultBox.depth = 10000.0;
            defaultBox.translation = Eigen::Vector3f(0,0,0);
            initOctree(defaultBox);
        }

        // 从根节点开始递归插入
        insertPointToOctree(m_octree_root.get(), pt, color, normal);
        m_point_count++;

        // 标记颜色已被修改 (如果添加了带颜色的点)
        if (color) m_color_modified = true;
    }

    CloudBlock* Cloud::insertPointToOctree(OctreeNode* node, const PointXYZ& pt,
                                           const ColorRGB* color, const CompressedNormal* normal)
    {
        // ---------------------------------------------------------
        // 情况 1: 当前是叶子节点 (Leaf Node)
        // ---------------------------------------------------------
        if (node->isLeaf()) {
            // 判断是否需要分裂：
            // 1. 块是否已满 (根据配置的 maxPointsPerBlock)
            // 2. 深度是否已达上限 (根据配置的 maxDepth)
            // 注意：如果 enableOctree=false，通常 maxPointsPerBlock 会设得很大，这里就不会触发分裂
            bool is_full = node->m_block->size() >= m_config.maxPointsPerBlock;
            bool reach_depth = node->m_depth >= m_config.maxDepth;

            if (!is_full || reach_depth) {
                // 未满或无法再分 -> 直接存入当前 Block
                node->m_block->addPoint(pt, color, normal, nullptr);
                return node->m_block.get();
            } else {
                // 已满且可分 -> 执行分裂
                // splitNode 内部会将当前 Block 的点分发给子节点，并将当前节点转为内部节点
                splitNode(node);

                // 分裂后，当前节点变成内部节点，递归调用自己重新插入该点
                return insertPointToOctree(node, pt, color, normal);
            }
        }
            // ---------------------------------------------------------
            // 情况 2: 当前是内部节点 (Internal Node)
            // ---------------------------------------------------------
        else {
            // 统计经过该节点的总点数 (用于采样概率计算)
            node->m_total_points_in_node++;

            // =========================================================
            // 动态 LOD 采样逻辑 (蓄水池采样 Reservoir Sampling)
            // =========================================================
            // 只有当配置允许生成 LOD 时才执行
            if (m_config.maxLODPoints > 0) {

                // 构造 LOD 点 (包含坐标和颜色)
                LODPoint lod_pt;
                lod_pt.x = pt.x; lod_pt.y = pt.y; lod_pt.z = pt.z;
                lod_pt._pad = 0;
                if (color) {
                    lod_pt.r = color->r; lod_pt.g = color->g; lod_pt.b = color->b;
                } else {
                    lod_pt.r = 255; lod_pt.g = 255; lod_pt.b = 255;
                }

                // 阶段 A: 蓄水池未满，直接添加
                if (node->m_lod_points.size() < m_config.maxLODPoints) {
                    node->m_lod_points.push_back(lod_pt);
                    node->m_lod_dirty = true;
                }
                    // 阶段 B: 蓄水池已满，进行随机替换
                    // 以 k/n 的概率决定是否保留当前点 (k=容量, n=当前总数)
                else {
                    size_t capacity = m_config.maxLODPoints;
                    size_t current_n = node->m_total_points_in_node;

                    std::uniform_int_distribution<size_t> dist_n(0, current_n - 1);
                    if (dist_n(m_rng) < capacity) {
                        std::uniform_int_distribution<size_t> dist_k(0, capacity - 1);
                        node->m_lod_points[dist_k(m_rng)] = lod_pt;
                        node->m_lod_dirty = true;
                    }
                }
            }

            // =========================================================
            // 向下路由 (Routing)
            // =========================================================
            int childIdx = node->getChildIndex(pt);

            // 如果对应子节点不存在，按需创建
            if (!node->m_children[childIdx]) {
                // 计算子节点的包围盒
                Box childBox;
                childBox.width = node->m_box.width * 0.5f;
                childBox.height = node->m_box.height * 0.5f;
                childBox.depth = node->m_box.depth * 0.5f;

                // 计算子节点中心偏移
                // childIdx bits: 0=x, 1=y, 2=z
                float dx = (childIdx & 1) ? 1.0f : -1.0f;
                float dy = (childIdx & 2) ? 1.0f : -1.0f;
                float dz = (childIdx & 4) ? 1.0f : -1.0f;

                childBox.translation = node->m_box.translation + Eigen::Vector3f(
                        dx * childBox.width * 0.5f,
                        dy * childBox.height * 0.5f,
                        dz * childBox.depth * 0.5f
                );

                // 创建新节点
                auto newChild = std::make_unique<OctreeNode>(childBox, node->m_depth + 1, node);

                // 新节点初始化为叶子，分配 Block
                newChild->m_block = std::make_shared<CloudBlock>();

                // 【优化】根据配置预留内存，避免频繁 realloc
                if (m_config.maxPointsPerBlock > 0) {
                    newChild->m_block->m_points.reserve(m_config.maxPointsPerBlock);
                }

                // 继承父节点的属性状态 (如是否启用了颜色/法线)
                if (m_has_rgb) newChild->m_block->m_colors = std::make_unique<std::vector<ColorRGB>>();
                if (m_has_normals) newChild->m_block->m_normals = std::make_unique<std::vector<CompressedNormal>>();

                // 处理颜色和法线的预留 (可选，如果内存允许)
                if (m_has_rgb && m_config.maxPointsPerBlock > 0)
                    newChild->m_block->m_colors->reserve(m_config.maxPointsPerBlock);

                // 如果当前插入的点带有颜色，确保新 Block 启用颜色
                if (color && !newChild->m_block->m_colors) {
                    newChild->m_block->m_colors = std::make_unique<std::vector<ColorRGB>>();
                    m_has_rgb = true;
                }
                if (normal && !newChild->m_block->m_normals) {
                    newChild->m_block->m_normals = std::make_unique<std::vector<CompressedNormal>>();
                    m_has_normals = true;
                }

                // 注册到全局列表
                m_all_blocks.push_back(newChild->m_block);

                node->m_children[childIdx] = std::move(newChild);
            }

            // 递归插入子节点
            return insertPointToOctree(node->m_children[childIdx].get(), pt, color, normal);
        }
    }

    void Cloud::splitNode(OctreeNode* node)
    {
        m_last_insert_node = nullptr;
        // 只有叶子节点才能分裂
        if (!node->isLeaf()) return;

        // 1. 获取原 Block 的数据所有权，并将当前节点标记为内部节点
        auto oldBlock = node->m_block;
        node->m_block = nullptr; // 移除引用，当前节点变为 Internal Node

        size_t n_points = oldBlock->size();

        // 准备 LOD 参数
        size_t lod_capacity = m_config.maxLODPoints;

        // 如果启用了 LOD，预分配内存以提升性能
        if (lod_capacity > 0) {
            node->m_lod_points.reserve(lod_capacity);
        }

        // 2. 遍历原 Block 中的所有点，分发到子节点并提取 LOD
        for (size_t i = 0; i < n_points; ++i) {
            const PointXYZ& pt = oldBlock->m_points[i];

            // =========================================================
            // A. LOD 提取 (蓄水池采样 Reservoir Sampling)
            // =========================================================
            // 逻辑：将旧块中的点作为“流”，从中均匀抽取 lod_capacity 个点保留在父节点
            if (lod_capacity > 0) {

                // 构造 LOD 点对象
                LODPoint lod_pt;
                lod_pt.x = pt.x; lod_pt.y = pt.y; lod_pt.z = pt.z;
                lod_pt._pad = 0;

                // 获取颜色
                if (oldBlock->m_colors) {
                    const auto& c = (*oldBlock->m_colors)[i];
                    lod_pt.r = c.r; lod_pt.g = c.g; lod_pt.b = c.b;
                } else {
                    lod_pt.r = 255; lod_pt.g = 255; lod_pt.b = 255;
                }

                // 采样逻辑
                if (node->m_lod_points.size() < lod_capacity) {
                    // 蓄水池未满，直接填入
                    node->m_lod_points.push_back(lod_pt);
                } else {
                    // 蓄水池已满，以 (capacity / (i+1)) 的概率随机替换
                    std::uniform_int_distribution<size_t> dist_i(0, i);
                    if (dist_i(m_rng) < lod_capacity) {
                        std::uniform_int_distribution<size_t> dist_k(0, lod_capacity - 1);
                        node->m_lod_points[dist_k(m_rng)] = lod_pt;
                    }
                }

                // 标记 LOD 数据变脏，需要更新渲染
                node->m_lod_dirty = true;
            }

            // =========================================================
            // B. 子节点分发 (Distribution)
            // =========================================================
            int childIdx = node->getChildIndex(pt);

            // 如果子节点不存在，创建它
            if (!node->m_children[childIdx]) {
                // 计算子节点包围盒
                Box childBox;
                childBox.width = node->m_box.width * 0.5f;
                childBox.height = node->m_box.height * 0.5f;
                childBox.depth = node->m_box.depth * 0.5f;

                float dx = (childIdx & 1) ? 1.0f : -1.0f;
                float dy = (childIdx & 2) ? 1.0f : -1.0f;
                float dz = (childIdx & 4) ? 1.0f : -1.0f;

                childBox.translation = node->m_box.translation + Eigen::Vector3f(
                        dx * childBox.width * 0.5f,
                        dy * childBox.height * 0.5f,
                        dz * childBox.depth * 0.5f
                );

                // 创建子节点
                auto newChild = std::make_unique<OctreeNode>(childBox, node->m_depth + 1, node);
                newChild->m_block = std::make_shared<CloudBlock>();

                // 【配置驱动】根据 Config 预留内存，避免后续 push_back 导致频繁扩容
                if (m_config.maxPointsPerBlock > 0) {
                    newChild->m_block->m_points.reserve(m_config.maxPointsPerBlock);
                }

                // 同步属性状态 (颜色/法线/标量场注册)
                if (m_has_rgb) {
                    newChild->m_block->m_colors = std::make_unique<std::vector<ColorRGB>>();
                    if (m_config.maxPointsPerBlock > 0) newChild->m_block->m_colors->reserve(m_config.maxPointsPerBlock);
                }
                if (m_has_normals) {
                    newChild->m_block->m_normals = std::make_unique<std::vector<CompressedNormal>>();
                    if (m_config.maxPointsPerBlock > 0) newChild->m_block->m_normals->reserve(m_config.maxPointsPerBlock);
                }

                // 同步标量场定义
                if (!oldBlock->m_scalar_fields.empty()) {
                    for(auto it = oldBlock->m_scalar_fields.begin(); it != oldBlock->m_scalar_fields.end(); ++it) {
                        newChild->m_block->registerScalarField(it->first);
                        // 标量场 vector 也可以 reserve
                        if (m_config.maxPointsPerBlock > 0) {
                            newChild->m_block->m_scalar_fields[it->first].reserve(m_config.maxPointsPerBlock);
                        }
                    }
                }

                // 如果旧块有备份颜色，子块也需要初始化备份容器（虽然此时为空，待数据移入后可能需要处理）
                // 通常 split 发生在加载阶段，备份颜色可能还不存在。如果是在编辑阶段 split，则需要处理。
                if (oldBlock->m_backup_colors) {
                    newChild->m_block->m_backup_colors = std::make_unique<std::vector<ColorRGB>>();
                }

                // 注册到全局列表
                m_all_blocks.push_back(newChild->m_block);
                node->m_children[childIdx] = std::move(newChild);
            }

            // =========================================================
            // C. 数据移动 (Move Data)
            // =========================================================
            auto childBlock = node->m_children[childIdx]->m_block;

            // 1. 基础坐标
            childBlock->m_points.push_back(pt);

            // 2. 颜色
            if (oldBlock->m_colors && childBlock->m_colors) {
                childBlock->m_colors->push_back((*oldBlock->m_colors)[i]);
            } else if (childBlock->m_colors) {
                // 异常保护：补白
                childBlock->m_colors->push_back(Color::White);
            }

            // 3. 备份颜色 (如果在编辑模式下分裂)
            if (oldBlock->m_backup_colors && childBlock->m_backup_colors) {
                childBlock->m_backup_colors->push_back((*oldBlock->m_backup_colors)[i]);
            }

            // 4. 法线
            if (oldBlock->m_normals && childBlock->m_normals) {
                childBlock->m_normals->push_back((*oldBlock->m_normals)[i]);
            } else if (childBlock->m_normals) {
                childBlock->m_normals->push_back(CompressedNormal());
            }

            // 5. 标量场 (批量处理所有字段)
            if (!oldBlock->m_scalar_fields.empty()) {
                // 遍历 oldBlock 的所有标量场字段
                auto it_old = oldBlock->m_scalar_fields.begin();
                for (; it_old != oldBlock->m_scalar_fields.end(); ++it_old) {
                    float val = it_old->second[i];
                    // childBlock 必定已注册该字段 (在创建节点时已同步)
                    childBlock->m_scalar_fields[it_old->first].push_back(val);
                }
            }

            // 标记子节点 Block 为脏 (需要上传 GPU)
            childBlock->markDirty();
        }

        // 3. 清理旧块资源
        oldBlock->clear();
        // 标记旧块为脏 (虽然它空了，但需要通知 Renderer 移除对应的显存资源)
        oldBlock->m_is_dirty = true;

        // 从全局 block 列表中移除空块，避免 m_all_blocks 膨胀
        m_all_blocks.erase(
            std::remove(m_all_blocks.begin(), m_all_blocks.end(), oldBlock),
            m_all_blocks.end()
        );
    }

    void Cloud::addPoints(const std::vector<PointXYZ>& pts,
                          const std::vector<ColorRGB>* colors,
                          const std::vector<CompressedNormal>* normals,
                          const std::unordered_map<std::string, std::vector<float>>* scalars)
    {
        if (pts.empty()) return;

        // 懒初始化
        if (!m_octree_root) {
            // 计算包围盒
            PointXYZ min_pt, max_pt;
            getMinMax3D(pts, min_pt, max_pt);
            Eigen::Vector3f center(min_pt.x, min_pt.y, min_pt.z); // 大致以起点为中心

            // TODO 能不能动态设置包围盒
            Box box;
            box.width = 10000.0;  // 10 km
            box.height = 10000.0;
            box.depth = 10000.0;
            box.translation = center;
            initOctree(box);
        }

        // 预处理标量场：检查维度是否一致
        if (scalars) {
            for (auto it = scalars->begin(); it != scalars->end(); ++it) {
                if (it->second.size() != pts.size()) {
                    continue;
                }
            }
        }

        size_t n = pts.size();

        // 串行插入 (因为 insertPointToOctree 可能触发 split，涉及树结构修改，不适合并行)
        for (size_t i = 0; i < n; ++i) {
            const ColorRGB* c = (colors && i < colors->size()) ? &(*colors)[i] : nullptr;
            const CompressedNormal* nm = (normals && i < normals->size()) ? &(*normals)[i] : nullptr;

            // 快速路径：如果上一次插入的叶子节点未满且包含当前点，直接插入
            CloudBlock* targetBlock = nullptr;
            if (m_last_insert_node && m_last_insert_node->isLeaf()
                && m_last_insert_node->m_block
                && m_last_insert_node->m_block->size() < m_config.maxPointsPerBlock
                && m_last_insert_node->getChildIndex(pts[i]) == m_last_insert_node->getChildIndex(pts[i])) {
                // 检查点是否在缓存节点的 AABB 内
                const auto& b = m_last_insert_node->m_box;
                float cx = b.translation.x(), cy = b.translation.y(), cz = b.translation.z();
                float hw = b.width * 0.5f, hh = b.height * 0.5f, hd = b.depth * 0.5f;
                const auto& p = pts[i];
                if (p.x >= cx - hw && p.x < cx + hw
                    && p.y >= cy - hh && p.y < cy + hh
                    && p.z >= cz - hd && p.z < cz + hd) {
                    m_last_insert_node->m_block->addPoint(p, c, nm, nullptr);
                    targetBlock = m_last_insert_node->m_block.get();
                }
            }

            // 慢速路径：完整递归插入
            if (!targetBlock) {
                targetBlock = insertPointToOctree(m_octree_root.get(), pts[i], c, nm);
                // 更新缓存（仅缓存叶子节点，split 后 m_last_insert_node 会被清空）
                if (m_last_insert_node && m_last_insert_node->isLeaf()) {
                    // splitNode 中会清空 m_last_insert_node，所以这里安全
                }
                // 找到当前点最终落入的叶子节点
                OctreeNode* leaf = m_octree_root.get();
                while (leaf && !leaf->isLeaf()) {
                    int idx = leaf->getChildIndex(pts[i]);
                    leaf = leaf->m_children[idx].get();
                }
                m_last_insert_node = leaf;
            }

            // 补充标量数据 (如果有)
            if (scalars && targetBlock) {

                int block_size = targetBlock->size();
                size_t idx_in_block = targetBlock->size() - 1;

                for (auto it = scalars->begin(); it != scalars->end(); ++it) {
                    const std::string& name = it->first;
                    float val = it->second[i];

                    // 确保 Block 有这个字段 (自动注册)
                    targetBlock->registerScalarField(name);

                    // 写入数据
                    targetBlock->m_scalar_fields[name][idx_in_block] = val;
                }
            }
        }

        m_point_count += n;
        if (colors) {
            m_color_modified = true;
            m_has_rgb = true;
        }
        if (normals) {
            m_has_normals = true;
        }
    }

    void Cloud::addPoint(const ct::PointXYZRGBN &pt) {
        ColorRGB color(pt.r, pt.g, pt.b);
        CompressedNormal normal;
        normal.set(Eigen::Vector3f(pt.normal_x, pt.normal_y, pt.normal_z));
        addPoint(PointXYZ(pt.x, pt.y, pt.z), &color, &normal);
    }

    void ct::Cloud::generateLOD()
    {
        if (!m_octree_root) return;

        // 自底向上递归生成
        generateLODRecursive(m_octree_root.get());
    }

    // ===== 容量接口 =====
    size_t Cloud::size() const
    {
        return m_point_count;
    }

    bool Cloud::empty() const
    {
        return m_point_count == 0;
    }

    bool Cloud::getFirstPoint(PointXYZ& out) const
    {
        if (m_all_blocks.empty()) return false;
        const auto& pts = m_all_blocks.front()->m_points;
        if (pts.empty()) return false;
        out = pts.front();
        return true;
    }

    void Cloud::clear()
    {
        m_octree_root.reset();
        m_all_blocks.clear();
        m_point_count = 0;
        m_last_insert_node = nullptr;

        m_has_rgb = false;
        m_has_normals = false;

        // 清理缓存
        m_cached_xyz.reset();
        m_cached_xyzrgb.reset();
        m_cached_xyzrgbn.reset();
    }

    void Cloud::enableColors()
    {
        if (m_has_rgb) return;

        // 遍历所有块，为每个块分配颜色内存
        for (auto& block : m_all_blocks) {
            if (!block->m_colors) {
                block->m_colors = std::make_unique<std::vector<ColorRGB>>();
                // 如果块里已经有点了，需要补齐白色
                if (!block->m_points.empty()) {
                    block->m_colors->resize(block->size(), Color::White);
                }
            }
        }
        m_has_rgb = true;
        invalidateCache();
    }

    void Cloud::enableNormals()
    {
        if (m_has_normals) return;

        for (auto& block : m_all_blocks) {
            if (!block->m_normals) {
                block->m_normals = std::make_unique<std::vector<CompressedNormal>>();
                if (!block->m_points.empty()) {
                    block->m_normals->resize(block->size()); // 默认构造为 0
                }
            }
        }
        m_has_normals = true;
        invalidateCache();
    }

    void Cloud::disableColors()
    {
        for (auto& block : m_all_blocks) {
            block->m_colors.reset();
        }
        m_has_rgb = false;
        invalidateCache();
    }

    void Cloud::disableNormals()
    {
        for (auto& block : m_all_blocks) {
            block->m_normals.reset();
        }
        m_has_normals = false;
        invalidateCache();
    }

    // 清空缓存
    void Cloud::invalidateCache()
    {
        m_cache_type = PCLCacheType::None;
        m_cached_xyz.reset();
        m_cached_xyzrgb.reset();
        m_cached_xyzrgbn.reset();
        m_scalar_cache.clear();
    }

    pcl::PointCloud<PointXYZ>::Ptr Cloud::toPCL_XYZ() const
    {
        if (m_cache_type == PCLCacheType::XYZ && m_cached_xyz) return m_cached_xyz;

        // 互斥：清除其他类型缓存
        m_cached_xyzrgb.reset();
        m_cached_xyzrgbn.reset();

        m_cached_xyz = std::make_shared<pcl::PointCloud<PointXYZ>>();
        m_cached_xyz->reserve(m_point_count);

        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;
            m_cached_xyz->insert(m_cached_xyz->end(), block->m_points.begin(), block->m_points.end());
        }

        m_cache_type = PCLCacheType::XYZ;
        return m_cached_xyz;
    }

    pcl::PointCloud<PointXYZRGB>::Ptr Cloud::toPCL_XYZRGB() const
    {
        if (m_cache_type == PCLCacheType::XYZRGB && m_cached_xyzrgb) return m_cached_xyzrgb;

        m_cached_xyz.reset();
        m_cached_xyzrgbn.reset();

        m_cached_xyzrgb = std::make_shared<pcl::PointCloud<PointXYZRGB>>();
        m_cached_xyzrgb->resize(m_point_count);

        size_t global_idx = 0;

        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;

            size_t n = block->size();
            const auto& pts = block->m_points;
            const auto* cols = block->m_colors.get();

#pragma omp parallel for if(n > 10000)
            for (int i = 0; i < (int)n; ++i) {
                auto& dst = m_cached_xyzrgb->points[global_idx + i];
                const auto& src_pt = pts[i];

                dst.x = src_pt.x; dst.y = src_pt.y; dst.z = src_pt.z;

                if (cols) {
                    const auto& c = (*cols)[i];
                    dst.r = c.r; dst.g = c.g; dst.b = c.b;
                } else {
                    dst.r = 255; dst.g = 255; dst.b = 255;
                }
            }
            global_idx += n;
        }

        m_cache_type = PCLCacheType::XYZRGB;
        return m_cached_xyzrgb;
    }

    pcl::PointCloud<PointXYZRGBN>::Ptr Cloud::toPCL_XYZRGBN() const
    {
        if (m_cache_type == PCLCacheType::XYZRGBN && m_cached_xyzrgbn) return m_cached_xyzrgbn;

        m_cached_xyz.reset();
        m_cached_xyzrgb.reset();

        m_cached_xyzrgbn = std::make_shared<pcl::PointCloud<PointXYZRGBN>>();
        m_cached_xyzrgbn->resize(m_point_count);

        size_t global_idx = 0;

        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;

            size_t n = block->size();
            const auto& pts = block->m_points;
            const auto* cols = block->m_colors.get();
            const auto* norms = block->m_normals.get();

#pragma omp parallel for if(n > 10000)
            for (int i = 0; (int)i < n; ++i) {
                auto& dst = m_cached_xyzrgbn->points[global_idx + i];
                const auto& src_pt = pts[i];

                dst.x = src_pt.x; dst.y = src_pt.y; dst.z = src_pt.z;

                if (cols) {
                    const auto& c = (*cols)[i];
                    dst.r = c.r; dst.g = c.g; dst.b = c.b;
                } else {
                    dst.r = 255; dst.g = 255; dst.b = 255;
                }

                if (norms) {
                    Eigen::Vector3f n_vec = (*norms)[i].get();
                    dst.normal_x = n_vec.x(); dst.normal_y = n_vec.y(); dst.normal_z = n_vec.z();
                } else {
                    dst.normal_x = 0; dst.normal_y = 0; dst.normal_z = 0;
                }
            }
            global_idx += n;
        }

        m_cache_type = PCLCacheType::XYZRGBN;
        return m_cached_xyzrgbn;
    }

    pcl::PointCloud<PointXYZRGB>::Ptr Cloud::toPCL_XYZRGB(const std::vector<int>& indices) const
    {
        // 预排序索引以启用单遍 block 遍历 (O(n) 而非 O(n*B))
        std::vector<std::pair<int, size_t>> sorted;
        sorted.reserve(indices.size());
        for (size_t qi = 0; qi < indices.size(); ++qi)
            sorted.emplace_back(indices[qi], qi);
        std::sort(sorted.begin(), sorted.end());

        auto result = std::make_shared<pcl::PointCloud<PointXYZRGB>>();
        result->resize(indices.size());

        size_t block_offset = 0;
        size_t block_idx = 0;

        for (const auto& [gidx, orig_qi] : sorted) {
            if (gidx < 0) continue;

            // 向前推进 block_offset 直到覆盖 gidx
            while (block_idx < m_all_blocks.size()) {
                size_t bs = m_all_blocks[block_idx]->size();
                if (gidx < (int)(block_offset + bs)) break;
                block_offset += bs;
                ++block_idx;
            }

            if (block_idx >= m_all_blocks.size()) break;

            auto& block = m_all_blocks[block_idx];
            size_t local = gidx - block_offset;
            if (local >= block->size()) continue;

            auto& dst = result->points[orig_qi];
            dst.x = block->m_points[local].x;
            dst.y = block->m_points[local].y;
            dst.z = block->m_points[local].z;

            if (block->m_colors) {
                dst.r = (*block->m_colors)[local].r;
                dst.g = (*block->m_colors)[local].g;
                dst.b = (*block->m_colors)[local].b;
            } else {
                dst.r = 255; dst.g = 255; dst.b = 255;
            }
        }

        result->width = indices.size();
        result->height = 1;
        return result;
    }

    Cloud::Ptr Cloud::extractByIndices(const std::vector<int>& indices) const
    {
        if (indices.empty()) return std::make_shared<Cloud>();

        // 预排序索引以启用单遍 block 遍历
        std::vector<std::pair<int, size_t>> sorted;
        sorted.reserve(indices.size());
        for (size_t qi = 0; qi < indices.size(); ++qi) {
            if (indices[qi] >= 0) sorted.emplace_back(indices[qi], qi);
        }
        std::sort(sorted.begin(), sorted.end());

        // 第一遍：计算包围盒
        PointXYZ min_pt(FLT_MAX, FLT_MAX, FLT_MAX);
        PointXYZ max_pt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        size_t block_offset = 0;
        size_t block_idx = 0;
        for (const auto& [gidx, orig_qi] : sorted) {
            while (block_idx < m_all_blocks.size()) {
                size_t bs = m_all_blocks[block_idx]->size();
                if (gidx < (int)(block_offset + bs)) break;
                block_offset += bs;
                ++block_idx;
            }
            if (block_idx >= m_all_blocks.size()) break;

            auto& block = m_all_blocks[block_idx];
            size_t local = gidx - block_offset;
            if (local >= block->size()) continue;

            const auto& p = block->m_points[local];
            if (p.x < min_pt.x) min_pt.x = p.x;
            if (p.y < min_pt.y) min_pt.y = p.y;
            if (p.z < min_pt.z) min_pt.z = p.z;
            if (p.x > max_pt.x) max_pt.x = p.x;
            if (p.y > max_pt.y) max_pt.y = p.y;
            if (p.z > max_pt.z) max_pt.z = p.z;
        }

        // 构建结果 Cloud
        Cloud::Ptr result(new Cloud);
        result->setGlobalShift(m_global_shift);

        Box box;
        box.width = max_pt.x - min_pt.x;
        box.height = max_pt.y - min_pt.y;
        box.depth = max_pt.z - min_pt.z;
        if (box.width < 1e-6) box.width = 1.0;
        if (box.height < 1e-6) box.height = 1.0;
        if (box.depth < 1e-6) box.depth = 1.0;
        box.translation = Eigen::Vector3f(
            min_pt.x + box.width / 2, min_pt.y + box.height / 2, min_pt.z + box.depth / 2);

        result->initOctree(box);
        if (m_has_rgb) result->enableColors();
        if (m_has_normals) result->enableNormals();

        // 收集标量场名称
        std::vector<std::string> scalar_names;
        if (!m_scalar_cache.empty()) {
            for (const auto& kv : m_scalar_cache) scalar_names.push_back(kv.first);
        }
        for (const auto& blk : m_all_blocks) {
            for (const auto& kv : blk->m_scalar_fields) {
                bool found = false;
                for (const auto& n : scalar_names) if (n == kv.first) { found = true; break; }
                if (!found) scalar_names.push_back(kv.first);
            }
        }

        // 批量提取：按 block 分组收集点
        size_t batch_size = 50000;
        std::vector<PointXYZ> pts; pts.reserve(batch_size);
        std::vector<ColorRGB> colors; colors.reserve(batch_size);
        std::vector<CompressedNormal> normals; normals.reserve(batch_size);
        std::unordered_map<std::string, std::vector<float>> scalar_batches;

        block_offset = 0;
        block_idx = 0;
        size_t prev_block_idx = SIZE_MAX;

        for (const auto& [gidx, orig_qi] : sorted) {
            // 向前推进 block_offset 直到覆盖 gidx
            while (block_idx < m_all_blocks.size()) {
                size_t bs = m_all_blocks[block_idx]->size();
                if (gidx < (int)(block_offset + bs)) break;
                block_offset += bs;
                ++block_idx;
            }
            if (block_idx >= m_all_blocks.size()) break;

            auto& block = m_all_blocks[block_idx];
            size_t local = gidx - block_offset;
            if (local >= block->size()) continue;

            // 如果切换了 block，先 flush 当前批次
            if (block_idx != prev_block_idx && !pts.empty()) {
                result->addPoints(pts,
                    m_has_rgb ? &colors : nullptr,
                    m_has_normals ? &normals : nullptr,
                    scalar_batches.empty() ? nullptr : &scalar_batches);
                pts.clear(); colors.clear(); normals.clear();
                scalar_batches.clear();
            }
            prev_block_idx = block_idx;

            pts.push_back(block->m_points[local]);

            if (m_has_rgb && block->m_colors) {
                colors.push_back((*block->m_colors)[local]);
            }

            if (m_has_normals && block->m_normals) {
                normals.push_back((*block->m_normals)[local]);
            }

            // 标量场
            for (const auto& sname : scalar_names) {
                auto it = block->m_scalar_fields.find(sname);
                if (it != block->m_scalar_fields.end() && local < it->second.size()) {
                    scalar_batches[sname].push_back(it->second[local]);
                } else {
                    scalar_batches[sname].push_back(0.0f);
                }
            }

            if (pts.size() >= batch_size) {
                result->addPoints(pts,
                    m_has_rgb ? &colors : nullptr,
                    m_has_normals ? &normals : nullptr,
                    scalar_batches.empty() ? nullptr : &scalar_batches);
                pts.clear(); colors.clear(); normals.clear();
                scalar_batches.clear();
            }
        }

        // flush 剩余
        if (!pts.empty()) {
            result->addPoints(pts,
                m_has_rgb ? &colors : nullptr,
                m_has_normals ? &normals : nullptr,
                scalar_batches.empty() ? nullptr : &scalar_batches);
        }

        result->update();
        return result;
    }

    void Cloud::addScalarField(const std::string& name, const std::vector<float>& data)
    {
        if (data.size() != m_point_count) return;

        // 需要将这一个大 vector 切分到各个 Block 中
        size_t global_offset = 0;

        for (auto& block : m_all_blocks) {
            if (block->empty()) continue;

            size_t n = block->size();
            // 获取 Block 内部对应字段的引用 (会自动创建)
            block->registerScalarField(name);
            std::vector<float>& block_data = block->m_scalar_fields[name];

            // 拷贝数据片段
            std::copy(data.begin() + global_offset,
                      data.begin() + global_offset + n,
                      block_data.begin());

            global_offset += n;
        }

        // 清理一下导出缓存，因为数据变了
        m_scalar_cache.erase(name);
    }

    bool Cloud::removeScalarField(const std::string& name)
    {
        bool found = false;
        for (auto& block : m_all_blocks) {
            if (block->m_scalar_fields.erase(name) > 0) {
                found = true;
            }
            block->rebuildScalarPtrCache();
        }
        m_scalar_cache.erase(name);
        return found;
    }

    void Cloud::clearScalarFields()
    {
        for (auto& block : m_all_blocks) {
            block->m_scalar_fields.clear();
            block->rebuildScalarPtrCache();
            block->m_scalar_fields.clear();
        }
        m_scalar_cache.clear();
    }

    bool Cloud::hasScalarField(const std::string& name) const
    {
        // 只要第一个非空 Block 有这个字段，就认为有
        for (const auto& block : m_all_blocks) {
            if (!block->empty()) {
                return block->m_scalar_fields.find(name) != block->m_scalar_fields.end();
            }
        }
        return false;
    }

    std::vector<std::string> Cloud::getScalarFieldNames() const
    {
        // 从第一个非空 Block 获取 keys
        for (const auto& block : m_all_blocks) {
            if (!block->empty()) {
                std::vector<std::string> names;
                names.reserve(block->m_scalar_fields.size());
                for (const auto& pair : block->m_scalar_fields) {
                    names.push_back(pair.first);
                }
                return names;
            }
        }
        return {};
    }

    const std::vector<float>* Cloud::getScalarField(const std::string& name) const
    {
        // 检查缓存中是否已有拼接好的数据
        auto cache_it = m_scalar_cache.find(name);
        if (cache_it != m_scalar_cache.end()) {
            if (cache_it->second.size() == m_point_count) {
                return &cache_it->second;
            }
        }

        // 如果没有，或者缓存无效，执行拼接
        if (!hasScalarField(name)) return nullptr;

        // LRU 上限：超过 3 个字段时清除最早的一个
        if (m_scalar_cache.size() >= 3) {
            auto oldest = m_scalar_cache.begin();
            oldest->second.clear();
            oldest->second.shrink_to_fit();
            m_scalar_cache.erase(oldest);
        }

        std::vector<float>& full_data = m_scalar_cache[name]; // 创建/覆盖缓存条目
        full_data.resize(m_point_count);

        size_t global_offset = 0;
        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;

            // 查找 Block 内的字段
            auto it = block->m_scalar_fields.find(name);
            if (it != block->m_scalar_fields.end()) {
                const std::vector<float>& src = it->second;
                std::copy(src.begin(), src.end(), full_data.begin() + global_offset);
            } else {
                // 如果某个 Block 缺失该字段 (异常情况)，填充 0
                std::fill_n(full_data.begin() + global_offset, block->size(), 0.0f);
            }
            global_offset += block->size();
        }

        return &full_data;
    }

    void Cloud::setCloudColor(const ColorRGB& rgb)
    {
        if (!m_has_rgb) enableColors();
        backupColors(); // 自动备份

#pragma omp parallel for
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];
            if (block->empty()) continue;

            // 直接批量赋值，比循环快
            std::fill(block->m_colors->begin(), block->m_colors->end(), rgb);

            block->m_is_dirty = true; // 确保标记为脏
            block->m_is_dirty = true; // 释放旧缓存
        }
        if (m_octree_root) {
            updateLODColorRecursive(m_octree_root.get(), rgb);
        }

        m_color_modified = true;
        invalidateCache(); // 通知视图更新
    }

    void Cloud::setCloudColor(const std::string& axis)
    {
        if (empty()) return;
        if (!m_has_rgb) enableColors();
        backupColors();

        float min_v = 0.0f, max_v = 0.0f;
        if (axis == "x") { min_v = m_min.x; max_v = m_max.x; }
        else if (axis == "y") { min_v = m_min.y; max_v = m_max.y; }
        else if (axis == "z") { min_v = m_min.z; max_v = m_max.z; }
        else return;

        float range = max_v - min_v;
        if (range < 1e-6f) range = 1.0f;

        // HSV 转 RGB lambda
        auto hsv2rgb = [](float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) {
            float c = v * s;
            float x = c * (1 - std::abs(std::fmod(h * 6, 2.0f) - 1));
            float m = v - c;
            float r_ = 0, g_ = 0, b_ = 0;

            if (h < 1.0/6.0) { r_=c; g_=x; b_=0; }
            else if (h < 2.0/6.0) { r_=x; g_=c; b_=0; }
            else if (h < 3.0/6.0) { r_=0; g_=c; b_=x; }
            else if (h < 4.0/6.0) { r_=0; g_=x; b_=c; }
            else if (h < 5.0/6.0) { r_=x; g_=0; b_=c; }
            else { r_=c; g_=0; b_=x; }

            r = static_cast<uint8_t>((r_ + m) * 255);
            g = static_cast<uint8_t>((g_ + m) * 255);
            b = static_cast<uint8_t>((b_ + m) * 255);
        };

        // 1. 更新 Block (叶子节点)
#pragma omp parallel for
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];
            if (block->empty()) continue;

            size_t n = block->size();
            for (size_t i = 0; i < n; ++i) {
                const auto& pt = block->m_points[i];
                float val = (axis == "x") ? pt.x : (axis == "y") ? pt.y : pt.z;

                float norm = (val - min_v) / range;
                if(norm < 0) norm = 0; if(norm > 1) norm = 1;
                float hue = (1.0f - norm) * 0.66f;

                hsv2rgb(hue, 1.0f, 1.0f, (*block->m_colors)[i].r, (*block->m_colors)[i].g, (*block->m_colors)[i].b);
            }
            block->m_is_dirty = true;
            block->markDirty();
        }

        // =========================================================
        // 【高性能方案 B】递归原地更新 LOD 颜色
        // =========================================================
        // 既然 LOD 点本身就有坐标，我们直接遍历树计算颜色，无需 generateLOD 重建

        // 定义递归 Lambda (需要包含 <functional>)
        std::function<void(OctreeNode*)> updateLOD = [&](OctreeNode* node) {
            if (!node) return;

            // 1. 处理当前节点的 LOD 点
            if (!node->m_lod_points.empty()) {
                for (auto& p : node->m_lod_points) {
                    float val = (axis == "x") ? p.x : (axis == "y") ? p.y : p.z;
                    float norm = (val - min_v) / range;
                    if(norm < 0) norm = 0; if(norm > 1) norm = 1;

                    float hue = (1.0f - norm) * 0.66f;

                    hsv2rgb(hue, 1.0f, 1.0f, p.r, p.g, p.b);
                }
                node->m_lod_dirty = true;
                node->m_vtk_lod_polydata.reset();
            }

            // 2. 递归子节点
            for (int i = 0; i < 8; ++i) {
                if (node->m_children[i]) {
                    updateLOD(node->m_children[i].get());
                }
            }
        };

        // 执行递归
        if (m_octree_root && m_config.enableOctree) {
            updateLOD(m_octree_root.get());
        }
        // =========================================================

        m_current_color_mode = axis;
        m_color_modified = true;
        invalidateCache();
    }

    void Cloud::updateLODColorRecursive(OctreeNode* node, const ColorRGB& rgb)
    {
        if (!node) return;

        // 更新当前节点的 LOD 数据颜色
        if (!node->m_lod_points.empty()) {
            for (auto& p : node->m_lod_points) {
                p.r = rgb.r;
                p.g = rgb.g;
                p.b = rgb.b;
            }
            // 标记 LOD 脏，强制 VTK 重新上传
            node->m_lod_dirty = true;
            node->m_vtk_lod_polydata.reset();
        }

        // 递归子节点
        for (int i = 0; i < 8; ++i) {
            if (node->m_children[i]) {
                updateLODColorRecursive(node->m_children[i].get(), rgb);
            }
        }
    }

    void Cloud::updateColorByField(const std::string& field_name)
    {
        if (!hasScalarField(field_name)) return;
        if (s_jet_lut.empty()) initColorTable();
        if (!m_has_rgb) enableColors();
        backupColors();

        // 1. 计算全局 Min/Max
        float min_v = FLT_MAX;
        float max_v = -FLT_MAX;

#pragma omp parallel
        {
            float local_min = FLT_MAX;
            float local_max = -FLT_MAX;

#pragma omp for
            for (int k = 0; k < m_all_blocks.size(); ++k) {
                auto& block = m_all_blocks[k];
                auto sf_it = block->m_scalar_fields.find(field_name);
                if (block->empty() || sf_it == block->m_scalar_fields.end()) continue;

                const std::vector<float>& data = sf_it->second;
                for (float v : data) {
                    if (v < local_min) local_min = v;
                    if (v > local_max) local_max = v;
                }
            }

#pragma omp critical
            {
                if (local_min < min_v) min_v = local_min;
                if (local_max > max_v) max_v = local_max;
            }
        }

        float range = max_v - min_v;
        if (range < 1e-6f) range = 1.0f;
        const float* pLut = s_jet_lut.data();
        int lut_size = 1024;

        // 2. 应用颜色
#pragma omp parallel for
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];
            auto sf_it = block->m_scalar_fields.find(field_name);
            if (block->empty() || sf_it == block->m_scalar_fields.end()) continue;
            if (!block->m_colors) continue;

            const std::vector<float>& data = sf_it->second;
            size_t n = block->size();
            size_t cn = block->m_colors->size();
            size_t count = std::min(n, cn);

            for (size_t i = 0; i < count; ++i) {
                float v = data[i];
                if (std::isnan(v)) continue;

                float norm = (v - min_v) / range;
                if (norm < 0.0f) norm = 0.0f;
                if (norm > 1.0f) norm = 1.0f;

                int idx = static_cast<int>(norm * (lut_size - 1));
                float lutVal = pLut[idx];
                uint32_t packed = *reinterpret_cast<const uint32_t*>(&lutVal);

                (*block->m_colors)[i].r = (packed >> 16) & 0xFF;
                (*block->m_colors)[i].g = (packed >> 8) & 0xFF;
                (*block->m_colors)[i].b = packed & 0xFF;
            }
            block->m_is_dirty = true;
            block->markDirty();
        }

        // 同步更新LOD节点颜色（原地刷新，避免全量重建）
        if (m_octree_root && m_config.enableOctree) {
            this->refreshLODColorsFromBlocks(m_octree_root.get());
        }
        m_current_color_mode = field_name;
        m_color_modified = true;
        invalidateCache();
    }

    void Cloud::updateColorByField(const std::string& field_name,
                                    float display_min, float display_max,
                                    ColormapType colormap,
                                    bool show_nan_as_grey)
    {
        if (!hasScalarField(field_name)) return;
        if (!m_has_rgb) enableColors();
        backupColors();

        const auto& lut = getColormapLUT(colormap);
        const float* pLut = lut.data();
        int lut_size = static_cast<int>(lut.size());

        float range = display_max - display_min;
        if (range < 1e-6f) range = 1.0f;

        const uint8_t grey_r = 180, grey_g = 180, grey_b = 180;

#pragma omp parallel for
        for (int k = 0; k < static_cast<int>(m_all_blocks.size()); ++k) {
            auto& block = m_all_blocks[k];
            auto sf_it = block->m_scalar_fields.find(field_name);
            if (block->empty() || sf_it == block->m_scalar_fields.end()) continue;
            if (!block->m_colors) continue;

            const std::vector<float>& data = sf_it->second;
            size_t n = block->size();
            size_t cn = block->m_colors->size();
            size_t count = std::min(n, cn);

            for (size_t i = 0; i < count; ++i) {
                float v = data[i];

                if (std::isnan(v)) {
                    if (show_nan_as_grey)
                        (*block->m_colors)[i] = {grey_r, grey_g, grey_b};
                    continue;
                }

                if (show_nan_as_grey && (v < display_min || v > display_max)) {
                    (*block->m_colors)[i] = {grey_r, grey_g, grey_b};
                } else {
                    float norm = (v - display_min) / range;
                    if (norm < 0.0f) norm = 0.0f;
                    if (norm > 1.0f) norm = 1.0f;
                    int idx = static_cast<int>(norm * (lut_size - 1));
                    float lutVal = pLut[idx];
                    uint32_t packed;
                    std::memcpy(&packed, &lutVal, sizeof(packed));
                    (*block->m_colors)[i].r = (packed >> 16) & 0xFF;
                    (*block->m_colors)[i].g = (packed >> 8) & 0xFF;
                    (*block->m_colors)[i].b = packed & 0xFF;
                }
                block->m_is_dirty = true;
                block->markDirty();
            }
        }

        if (m_octree_root && m_config.enableOctree) {
            this->refreshLODColorsFromBlocks(m_octree_root.get());
        }
        m_current_color_mode = field_name;
        m_color_modified = true;
        invalidateCache();
    }

    bool Cloud::getFieldRange(const std::string& field_name,
                              float& out_min, float& out_max) const
    {
        if (!hasScalarField(field_name)) return false;

        float min_v = FLT_MAX;
        float max_v = -FLT_MAX;

        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;
            auto it = block->m_scalar_fields.find(field_name);
            if (it == block->m_scalar_fields.end()) continue;

            const auto& data = it->second;
            for (float v : data) {
                if (std::isnan(v)) continue;
                if (v < min_v) min_v = v;
                if (v > max_v) max_v = v;
            }
        }

        if (min_v > max_v) return false;
        out_min = min_v;
        out_max = max_v;
        return true;
    }

    void Cloud::update()
    {
        if (empty() || !m_octree_root) {
            m_min = m_max = PointXYZ(0,0,0);
            return;
        }

        PointXYZ real_min(FLT_MAX, FLT_MAX, FLT_MAX);
        PointXYZ real_max(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        size_t total = 0;

        updateRecursive(m_octree_root.get(), real_min, real_max, total);

        m_min = real_min;
        m_max = real_max;
        m_point_count = total; // 修正总点数

        // 更新中心点
        Eigen::Vector3f center = 0.5f * (m_min.getVector3fMap() + m_max.getVector3fMap());
        m_box.pose.translation() = center;
        m_box.translation = center;

        m_box.width = m_max.x - m_min.x;
        m_box.height = m_max.y - m_min.y;
        m_box.depth = m_max.z - m_min.z;

        // 防止包围盒过扁（可选）
        if (m_box.width < 0.1) m_box.width = 0.1;
        if (m_box.height < 0.1) m_box.height = 0.1;
        if (m_box.depth < 0.1) m_box.depth = 0.1;

        // 同步根节点包围盒（仅在根节点未分裂时安全）
        // 子节点的包围盒在 splitNode 中从父节点 m_box 派生，不能随意修改
        if (m_octree_root->isLeaf()) {
            m_octree_root->m_box = m_box;
        }

        // 更新类型字符串
        if (m_has_normals && m_has_rgb) m_type = CLOUD_TYPE_XYZRGBN;
        else if (m_has_normals) m_type = CLOUD_TYPE_XYZN;
        else if (m_has_rgb) m_type = CLOUD_TYPE_XYZRGB;
        else m_type = CLOUD_TYPE_XYZ;

        // 自动估算分辨率
        if (m_resolution <= 0.0001f && m_point_count > 0) {
            m_resolution = computeResolution();
        }
    }

    float Cloud::computeResolution(int sampleCount)
    {
        if (m_all_blocks.empty()) return 0.0f;

        // 策略：随机选取一个点数较多的 Block，计算其局部密度作为全局代表
        // 这样可以避免为千万级大点云构建全局 KdTree，性能极高 (ms 级)
        CloudBlock::Ptr targetBlock = nullptr;
        for (const auto& block : m_all_blocks) {
            if (block && block->size() > 100) {
                targetBlock = block;
                break;
            }
        }

        if (!targetBlock) targetBlock = m_all_blocks[0];
        if (targetBlock->empty()) return 0.0f;

        size_t n = targetBlock->size();
        auto cloud_pcl = std::make_shared<pcl::PointCloud<PointXYZ>>();
        cloud_pcl->points.assign(targetBlock->m_points.begin(), targetBlock->m_points.end());
        cloud_pcl->width = n;
        cloud_pcl->height = 1;

        pcl::KdTreeFLANN<PointXYZ> kdtree;
        kdtree.setInputCloud(cloud_pcl);

        double totalDist = 0;
        int validPoints = 0;
        int samples = std::min((int)n, sampleCount);

        std::vector<int> indices(2);
        std::vector<float> dists(2);

        for (int i = 0; i < samples; ++i) {
            // 随机采样
            int idx = rand() % n;
            // 搜索最近的 2 个点 (第 0 个是点自己)
            if (kdtree.nearestKSearch(cloud_pcl->points[idx], 2, indices, dists) > 1) {
                totalDist += std::sqrt(dists[1]);
                validPoints++;
            }
        }

        return (validPoints > 0) ? (float)(totalDist / validPoints) : 0.0f;
    }

    void Cloud::updateRecursive(OctreeNode* node, PointXYZ& min_pt, PointXYZ& max_pt, size_t& count)
    {
        if (!node) return;

        if (node->isLeaf()) {
            // 叶子节点：统计 Block 数据
            auto block = node->m_block;
            if (!block || block->empty()) return;

            count += block->size();

            // 这里简单串行处理
            for (const auto& p : block->m_points) {
                if (p.x < min_pt.x) min_pt.x = p.x;
                if (p.x > max_pt.x) max_pt.x = p.x;
                if (p.y < min_pt.y) min_pt.y = p.y;
                if (p.y > max_pt.y) max_pt.y = p.y;
                if (p.z < min_pt.z) min_pt.z = p.z;
                if (p.z > max_pt.z) max_pt.z = p.z;
            }

            // 顺便更新 Block 的 AABB（如果需要的话）
            // block->updateBoundingBox();
        }
        else {
            // 内部节点：递归
            for (int i = 0; i < 8; ++i) {
                updateRecursive(node->m_children[i].get(), min_pt, max_pt, count);
            }
        }
    }

    void Cloud::backupColors()
    {
#pragma omp parallel for
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];

            if (block->empty() || !block->m_colors) continue;
            if (block->m_backup_colors) continue; // 只保留第一次备份（原始颜色）

            block->m_backup_colors = std::make_unique<std::vector<ColorRGB>>(*block->m_colors);
        }
    }

    void Cloud::restoreColors()
    {
        bool any_restored = false;

#pragma omp parallel for reduction(|:any_restored)
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];

            // 只有当存在备份时才恢复
            if (block->m_backup_colors) {
                // 如果当前 m_colors 被释放了（比如调用过 disableColors），需要重新分配
                if (!block->m_colors) {
                    block->m_colors = std::make_unique<std::vector<ColorRGB>>();
                }

                *block->m_colors = *block->m_backup_colors;
                if (!block->m_colors->empty()) any_restored = true;
            }
        }

        if (any_restored) {
            m_has_rgb = true; // 确保 Cloud 状态正确
            m_color_modified = false; // 颜色已恢复到原始状态（或上一次备份的状态）
            if (m_octree_root && m_config.enableOctree) {
                this->generateLOD();
            }
            m_current_color_mode = "RGB (Default)";
            invalidateCache();
        }
    }

    Cloud::Ptr Cloud::clone() const
    {
        Cloud::Ptr result(new Cloud);
        result->copyFrom(*this);
        return result;
    }

    void Cloud::copyFrom(const Cloud& other)
    {
        // 清空当前对象
        this->clear();

        // 基础属性拷贝
        this->m_id = other.m_id + "_clone";
        this->m_filepath = other.m_filepath;
        this->m_box = other.m_box;
        this->m_box_rgb = other.m_box_rgb;
        this->m_normals_rgb = other.m_normals_rgb;
        this->m_point_size = other.m_point_size;
        this->m_opacity = other.m_opacity;
        this->m_resolution = other.m_resolution;
        this->m_min = other.m_min;
        this->m_max = other.m_max;
        this->m_global_shift = other.m_global_shift;
        this->m_has_rgb = other.m_has_rgb;
        this->m_has_normals = other.m_has_normals;
        this->m_type = other.m_type;
        this->m_max_depth = other.m_max_depth;

        // 递归深拷贝八叉树结构
        // 这一步会自动重建 m_all_blocks 列表
        if (other.m_octree_root) {
            this->m_octree_root = cloneOctreeRecursive(other.m_octree_root.get(), nullptr);
        }

        // 复制统计数据
        this->m_point_count = other.m_point_count;
        this->m_color_modified = other.m_color_modified;
    }

    std::unique_ptr<OctreeNode> Cloud::cloneOctreeRecursive(const OctreeNode* src_node, OctreeNode* parent)
    {
        if (!src_node) return nullptr;

        // 克隆当前节点 (Metadata)
        auto new_node = std::make_unique<OctreeNode>(src_node->m_box, src_node->m_depth, parent);
        new_node->m_total_points_in_node = src_node->m_total_points_in_node;

        // 如果有 LOD 数据，也一并拷贝
        new_node->m_lod_points = src_node->m_lod_points;

        // 如果是叶子节点，深拷贝 Block 数据
        if (src_node->isLeaf() && src_node->m_block) {
            new_node->m_block = std::make_shared<CloudBlock>();

            const auto& src_block = src_node->m_block;
            auto& dst_block = new_node->m_block;

            // --- 核心数据拷贝 ---
            dst_block->m_points = src_block->m_points;
            dst_block->m_box = src_block->m_box;

            // 颜色 (深拷贝 vector)
            if (src_block->m_colors)
                dst_block->m_colors = std::make_unique<std::vector<ColorRGB>>(*src_block->m_colors);

            // 法线
            if (src_block->m_normals)
                dst_block->m_normals = std::make_unique<std::vector<CompressedNormal>>(*src_block->m_normals);

            // 标量场 (std::map 具有值语义，直接赋值会触发深拷贝)
            dst_block->m_scalar_fields = src_block->m_scalar_fields;
            dst_block->rebuildScalarPtrCache();

            // 备份颜色
            if (src_block->m_backup_colors)
                dst_block->m_backup_colors = std::make_unique<std::vector<ColorRGB>>(*src_block->m_backup_colors);

            // 渲染状态
            dst_block->m_is_visible = src_block->m_is_visible;
            dst_block->m_is_dirty = true; // 新数据，标记为脏，以便上传 GPU

            // 将新 Block 注册到 Cloud 的扁平化列表中
            this->m_all_blocks.push_back(dst_block);
        }

        // 递归克隆子节点
        for (int i = 0; i < 8; ++i) {
            if (src_node->m_children[i]) {
                // 递归调用，并将返回的 unique_ptr 所有权释放给裸指针数组 m_children
                // 注意：这里传入 new_node.get() 作为 parent
                new_node->m_children[i] = cloneOctreeRecursive(src_node->m_children[i].get(), new_node.get());
            }
        }

        return new_node;
    }

    void Cloud::generateLODRecursive(OctreeNode* node)
    {
        if (!node) return;

        // 如果是叶子节点，清空其 LOD 数据
        // 叶子节点直接渲染 Block，不需要 LOD 代理点；LOD 仅存在于内部节点
        if (node->isLeaf()) {
            node->clearLOD();
            return;
        }

        // 先递归处理所有子节点（自底向上构建）
        for (int i = 0; i < 8; ++i) {
            if (node->m_children[i]) {
                generateLODRecursive(node->m_children[i].get());
            }
        }

        // 收集候选点 (Candidates)
        // 我们需要从 8 个子节点中汇聚点，然后选出代表当前节点的 LOD

        // 如果当前节点已有足够的 LOD 数据（来自加载时蓄水池采样），跳过重建
        size_t target_lod_size = m_config.maxLODPoints;
        if (target_lod_size == 0) return;

        if (!node->m_lod_points.empty() &&
            node->m_lod_points.size() >= target_lod_size * 0.8) {
            for (int i = 0; i < 8; ++i) {
                if (node->m_children[i]) generateLODRecursive(node->m_children[i].get());
            }
            return;
        }

        std::vector<LODPoint> candidates;

        // 预估容量：8个子节点，每个最多贡献 maxLODPoints 个点
        candidates.reserve(target_lod_size * 8);

        for (int i = 0; i < 8; ++i) {
            OctreeNode* child = node->m_children[i].get();
            if (!child) continue;

            if (child->isLeaf()) {
                // --- 情况 A: 子节点是叶子 (取 Block 数据) ---
                auto block = child->m_block;
                if (!block || block->empty()) continue;

                size_t block_size = block->size();

                // 为了防止内存爆炸，如果 Block 很大，不要把所有点都拿来
                // 我们只需要采集一部分代表点。
                // 假设我们希望从每个子节点至少拿到 target_lod_size 的点用于混合（如果够的话）
                // 计算采样步长：
                size_t step = 1;
                if (block_size > target_lod_size) {
                    step = block_size / target_lod_size;
                }
                if (step < 1) step = 1;

                for (size_t k = 0; k < block_size; k += step) {
                    LODPoint p;
                    const auto& src_pt = block->m_points[k];
                    p.x = src_pt.x; p.y = src_pt.y; p.z = src_pt.z;
                    p._pad = 0;

                    if (block->m_colors) {
                        const auto& c = (*block->m_colors)[k];
                        p.r = c.r; p.g = c.g; p.b = c.b;
                    } else {
                        p.r = 255; p.g = 255; p.b = 255;
                    }
                    candidates.push_back(p);
                }
            }
            else {
                // --- 情况 B: 子节点是内部节点 (取 LOD 数据) ---
                // 直接继承子节点已经生成的 LOD 点
                // 子节点的 LOD 已经是下采样过的，数量不超过 target_lod_size
                candidates.insert(candidates.end(),
                                  child->m_lod_points.begin(),
                                  child->m_lod_points.end());
            }
        }

        // 对候选点进行最终降采样 (Downsampling)
        // 如果收集到的点总数超过了当前节点的预算，随机打乱并截断
        if (candidates.size() > target_lod_size) {
            // 使用随机洗牌来实现均匀采样，避免只保留了前几个子节点的点
            static std::random_device rd;
            static std::mt19937 g(rd());
            std::shuffle(candidates.begin(), candidates.end(), g);

            candidates.resize(target_lod_size);
        }

        // 存入当前节点
        // 使用 std::move 避免拷贝
        node->m_lod_points = std::move(candidates);
        node->m_lod_dirty = true;

        // 清理旧的 VTK 缓存，等待渲染器上传
        node->m_vtk_lod_polydata.reset();
    }

    // 查找包含目标坐标的叶子 block（递归八叉树查找）
    static CloudBlock* findLeafBlockForPoint(OctreeNode* node, float x, float y, float z)
    {
        if (!node) return nullptr;
        if (node->isLeaf()) return node->m_block.get();

        int index = 0;
        if (x >= node->m_box.translation.x()) index |= 1;
        if (y >= node->m_box.translation.y()) index |= 2;
        if (z >= node->m_box.translation.z()) index |= 4;

        return findLeafBlockForPoint(node->m_children[index].get(), x, y, z);
    }

    void Cloud::refreshLODColorsFromBlocks(OctreeNode* node)
    {
        if (!node || node->isLeaf()) return;

        if (!node->m_lod_points.empty()) {
            for (auto& p : node->m_lod_points) {
                CloudBlock* block = findLeafBlockForPoint(node, p.x, p.y, p.z);
                if (!block || !block->m_colors || block->empty()) {
                    p.r = 255; p.g = 255; p.b = 255;
                    continue;
                }

                const auto& pts = block->m_points;
                const auto& cols = *block->m_colors;
                size_t n = pts.size();

                // 采样搜索：最多检查 256 个点，对 LOD 近似着色足够
                size_t step = (n > 256) ? (n / 256) : 1;
                size_t best_idx = 0;
                float best_dist = std::numeric_limits<float>::max();

                for (size_t i = 0; i < n; i += step) {
                    float dx = pts[i].x - p.x;
                    float dy = pts[i].y - p.y;
                    float dz = pts[i].z - p.z;
                    float dist = dx*dx + dy*dy + dz*dz;
                    if (dist < best_dist) {
                        best_dist = dist;
                        best_idx = i;
                    }
                }
                p.r = cols[best_idx].r;
                p.g = cols[best_idx].g;
                p.b = cols[best_idx].b;
            }

            node->m_lod_dirty = true;
            node->m_vtk_lod_polydata.reset();
        }

        for (int i = 0; i < 8; ++i) {
            if (node->m_children[i]) {
                refreshLODColorsFromBlocks(node->m_children[i].get());
            }
        }
    }

    bool Cloud::isStructureSplit() const {
        if (!m_octree_root) return false;
        // 如果根节点不是叶子（说明有子节点），那就是分裂了
        return !m_octree_root->isLeaf();
    }

    void Cloud::append(const Cloud& other)
    {
        if (other.empty()) return;

        if (other.hasColors()) this->enableColors();
        if (other.hasNormals()) this->enableNormals();

        // 遍历源对象的所有 Block
        for (const auto& block : other.m_all_blocks) {
            if (block->empty()) continue;

            // 基础属性
            const std::vector<ColorRGB>* c = (block->m_colors) ? block->m_colors.get() : nullptr;
            const std::vector<CompressedNormal>* n = (block->m_normals) ? block->m_normals.get() : nullptr;

            // 标量场 (unordered_map 指针)
            const std::unordered_map<std::string, std::vector<float>>* s =
                    (!block->m_scalar_fields.empty()) ? &block->m_scalar_fields : nullptr;

            // 批量插入
            this->addPoints(block->m_points, c, n, s);
        }

        update();

        // 缓存失效
        invalidateCache();
        invalidateCache();
    }

    Cloud& Cloud::operator+=(const Cloud& rhs)
    {
        append(rhs);
        return *this;
    }

    Cloud::Ptr Cloud::fromPCL_XYZRGBN(const pcl::PointCloud<PointXYZRGBN>& pcl_cloud,
                                       const Eigen::Vector3d& global_shift)
    {
        Cloud::Ptr cloud(new Cloud);
        if (pcl_cloud.empty()) return cloud;
        cloud->setGlobalShift(global_shift);

        // 计算包围盒以初始化八叉树
        PointXYZRGBN min_pt, max_pt;
        pcl::getMinMax3D<PointXYZRGBN>(pcl_cloud, min_pt, max_pt);

        Box box;
        box.width = max_pt.x - min_pt.x;
        box.height = max_pt.y - min_pt.y;
        box.depth = max_pt.z - min_pt.z;
        if (box.width < 1e-6) box.width = 1.0;
        if (box.height < 1e-6) box.height = 1.0;
        if (box.depth < 1e-6) box.depth = 1.0;
        box.translation = Eigen::Vector3f(min_pt.x + box.width/2, min_pt.y + box.height/2, min_pt.z + box.depth/2);

        cloud->initOctree(box);
        cloud->enableColors();
        cloud->enableNormals();

        // 转换数据格式以便批量添加
        // 为了避免一次性拷贝所有数据造成内存峰值，我们可以分批次添加
        size_t n = pcl_cloud.size();
        size_t batch_size = 50000;

        std::vector<PointXYZ> pts; pts.reserve(batch_size);
        std::vector<ColorRGB> colors; colors.reserve(batch_size);
        std::vector<CompressedNormal> normals; normals.reserve(batch_size);

        for (size_t i = 0; i < n; ++i) {
            const auto& src = pcl_cloud.points[i];

            pts.emplace_back(src.x, src.y, src.z);
            colors.emplace_back(src.r, src.g, src.b);

            CompressedNormal cn;
            cn.set(Eigen::Vector3f(src.normal_x, src.normal_y, src.normal_z));
            normals.push_back(cn);

            if (pts.size() >= batch_size) {
                cloud->addPoints(pts, &colors, &normals);
                pts.clear(); colors.clear(); normals.clear();
            }
        }

        // 添加剩余的
        if (!pts.empty()) {
            cloud->addPoints(pts, &colors, &normals);
        }

        cloud->update();
        return cloud;
    }

    Cloud::Ptr Cloud::fromPCL_XYZRGB(const pcl::PointCloud<PointXYZRGB> &pcl_cloud,
                                     const Eigen::Vector3d& global_shift) {
        Cloud::Ptr cloud(new Cloud);
        if (pcl_cloud.empty()) return cloud;
        cloud->setGlobalShift(global_shift);

        // 计算包围盒以初始化八叉树
        PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D<PointXYZRGB>(pcl_cloud, min_pt, max_pt);

        Box box;
        box.width = max_pt.x - min_pt.x;
        box.height = max_pt.y - min_pt.y;
        box.depth = max_pt.z - min_pt.z;
        if (box.width < 1e-6) box.width = 1.0;
        if (box.height < 1e-6) box.height = 1.0;
        if (box.depth < 1e-6) box.depth = 1.0;
        box.translation = Eigen::Vector3f(min_pt.x + box.width/2, min_pt.y + box.height/2, min_pt.z + box.depth/2);

        cloud->initOctree(box);
        cloud->enableColors();
        cloud->enableNormals();

        // 转换数据格式以便批量添加
        // 为了避免一次性拷贝所有数据造成内存峰值，我们可以分批次添加
        size_t n = pcl_cloud.size();
        size_t batch_size = 50000;

        std::vector<PointXYZ> pts; pts.reserve(batch_size);
        std::vector<ColorRGB> colors; colors.reserve(batch_size);


        for (size_t i = 0; i < n; ++i) {
            const auto& src = pcl_cloud.points[i];

            pts.emplace_back(src.x, src.y, src.z);
            colors.emplace_back(src.r, src.g, src.b);


            if (pts.size() >= batch_size) {
                cloud->addPoints(pts, &colors);
                pts.clear(); colors.clear();
            }
        }

        // 添加剩余的
        if (!pts.empty()) {
            cloud->addPoints(pts, &colors);
        }

        cloud->update();
        return cloud;
    }

    Cloud::Ptr Cloud::fromPCL_XYZ(const pcl::PointCloud<PointXYZ>& pcl_cloud,
                                   const Eigen::Vector3d& global_shift)
    {
        Cloud::Ptr cloud(new Cloud);
        if (pcl_cloud.empty()) return cloud;
        cloud->setGlobalShift(global_shift);

        PointXYZ min_pt, max_pt;
        pcl::getMinMax3D<PointXYZ>(pcl_cloud, min_pt, max_pt);

        Box box;
        box.width = max_pt.x - min_pt.x;
        box.height = max_pt.y - min_pt.y;
        box.depth = max_pt.z - min_pt.z;
        if (box.width < 1e-6) box.width = 1.0;
        if (box.height < 1e-6) box.height = 1.0;
        if (box.depth < 1e-6) box.depth = 1.0;
        box.translation = Eigen::Vector3f(min_pt.x + box.width/2, min_pt.y + box.height/2, min_pt.z + box.depth/2);

        cloud->initOctree(box);

        size_t n = pcl_cloud.size();
        size_t batch_size = 50000;
        std::vector<PointXYZ> pts; pts.reserve(batch_size);

        for (size_t i = 0; i < n; ++i) {
            const auto& src = pcl_cloud.points[i];
            pts.emplace_back(src.x, src.y, src.z);

            if (pts.size() >= batch_size) {
                cloud->addPoints(pts);
                pts.clear();
            }
        }

        if (!pts.empty()) {
            cloud->addPoints(pts);
        }

        cloud->update();
        return cloud;
    }

    void Cloud::removeInvalidPoints()
    {
        // 遍历所有 Block，移除 NaN
        for (auto& block : m_all_blocks) {
            if (block->empty()) continue;

            size_t write_idx = 0;
            size_t read_idx = 0;
            size_t n = block->size();

            for (; read_idx < n; ++read_idx) {
                const auto& p = block->m_points[read_idx];
                if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                    if (write_idx != read_idx) {
                        block->m_points[write_idx] = p;
                        if (block->m_colors) (*block->m_colors)[write_idx] = (*block->m_colors)[read_idx];
                        if (block->m_normals) (*block->m_normals)[write_idx] = (*block->m_normals)[read_idx];
                        // scalars...
                    }
                    write_idx++;
                }
            }

            // Resize
            block->m_points.resize(write_idx);
            if (block->m_colors) block->m_colors->resize(write_idx);
            if (block->m_normals) block->m_normals->resize(write_idx);
        }

        invalidateCache();
        update();
    }

    // 初始化颜色映射表 (Jet Colormap)
    void Cloud::initColorTable()
    {
        // 静态成员只需初始化一次
        if (!s_jet_lut.empty()) return;

        s_jet_lut.resize(1024);

        for (int i = 0; i < 1024; ++i) {
            float v = (float)i / 1023.0f;
            uint8_t r = 0, g = 0, b = 0;

            if (v < 0.25f) { r=0; g=(uint8_t)(255*4*v); b=255; }
            else if (v < 0.5f) { r=0; g=255; b=(uint8_t)(255*(1-4*(v-0.25))); }
            else if (v < 0.75f) { r=(uint8_t)(255*4*(v-0.5)); g=255; b=0; }
            else { r=255; g=(uint8_t)(255*(1-4*(v-0.75))); b=0; }

            uint32_t packed = (r << 16) | (g << 8) | b;
            s_jet_lut[i] = *reinterpret_cast<float*>(&packed);
        }
    }

    // TODO 获取指定全局索引点的颜色 (用于保存文件)
    // 注意：这是一个 O(N) 操作，如果在循环中对每个点都调用这个函数，保存大文件会非常慢！
    // 强烈建议：在保存逻辑中直接遍历 Block，而不是遍历全局索引。
    // 但为了保持接口兼容性，我们先实现它。
    std::uint32_t Cloud::getPointColorForSave(size_t index) const
    {
        // 快速检查
        if (!m_has_rgb) return (255<<16) | (255<<8) | 255; // White

        // 线性查找 Block (性能瓶颈)
        // 优化：保存算法应该重构为遍历 Block
        size_t current_offset = 0;

        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;

            size_t block_size = block->size();

            // 检查索引是否在当前 Block 内
            if (index < current_offset + block_size) {
                size_t local_idx = index - current_offset;

                if (block->m_colors) {
                    const ColorRGB& c = (*block->m_colors)[local_idx];
                    return (c.r << 16) | (c.g << 8) | c.b;
                } else {
                    return (255<<16) | (255<<8) | 255;
                }
            }

            current_offset += block_size;
        }

        // 索引越界
        return (255<<16) | (255<<8) | 255;
    }

} // namespace ct
