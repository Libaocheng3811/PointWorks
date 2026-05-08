//
// Created by LBC on 2026/1/28.
//

#ifndef POINTWORKS_OCTREERENDERER_H
#define POINTWORKS_OCTREERENDERER_H

#include "core/cloud.h"
#include "core/octree.h"

#include <vtkSmartPointer.h>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>

// VTK 前向声明
class vtkRenderer;
class vtkActor;

namespace pw{
    /**
    * @brief 高性能八叉树 LOD 渲染器
    * @details 策略：
    * 1. SSE (Screen Space Error) 遍历：根据屏幕投影大小决定渲染精细 Block 还是粗糙 LOD。
    * 2. 动态合并 (Dynamic Merging)：每帧将所有可见点合并到一个巨大的 vtkPolyData 中，只使用 1 个 Actor。
    */
    class OctreeRenderer{
    public:
        OctreeRenderer(Cloud::Ptr cloud, vtkRenderer* renderer);
        ~OctreeRenderer();

        void setVisibility(bool visible);
        bool isVisible() const { return m_visible; }

        void update();
        void invalidateCache();
        void invalidateDirtyActors();

        // 设置交互状态 (true=正在拖拽/缩放, false=静止)
        void setInteractionState(bool is_interacting);

        std::vector<vtkActor*> getActiveActors() const;
        CloudBlock::Ptr getBlockFromActor(vtkActor* actor);
        pw::Cloud::Ptr getCloud() const {return m_cloud; }

    private:
        // 遍历上下文
        struct TraversalContext {
            const double* planes;
            Eigen::Vector3f camPos;
            float pixelsPerUnit;
            std::vector<OctreeNode*>* visibleNodes;

            size_t currentPointCount = 0; // 当前帧已收集的点数
            size_t maxPointBudget = 0;    // 最大允许点数
            float threshold = 0.0f;       // 当前使用的屏幕阈值
        };

        struct PriorityNode {
            OctreeNode* node;
            float screenSize;

            // 优先级比较：screenSize 大的在堆顶 (优先分裂)
            bool operator<(const PriorityNode& other) const {
                return screenSize < other.screenSize;
            }
        };

        // 为节点创建或获取缓存的 Actor
        vtkActor* getOrCreateActor(OctreeNode* node, bool is_lod);

        // 缓存驱逐：移除长时间不可见的 Actor 以释放 GPU 资源
        void evictHiddenActors();

        // 辅助计算
        bool isBoxInFrustum(const Box& box, const double* planes);
        float projectSize(const Box& box, const Eigen::Vector3f& camPos, float pixelsPerUnit);

    private:
        Cloud::Ptr m_cloud;
        vtkRenderer* m_vtk_renderer;
        bool m_visible = true;

        // --- 核心缓存 ---
        // Key: Node指针, Value: 对应的 VTK Actor
        // 我们不需要区分是 LOD 还是 Block，因为一个 Node 同一时间只显示一种状态
        // 但为了安全，我们可以让 Node 自己持有 Actor (侵入式) 或者这里用 Map (非侵入式)
        // 这里使用 Map 避免修改 OctreeNode 头文件太过频繁
        std::unordered_map<OctreeNode*, vtkSmartPointer<vtkActor>> m_actor_cache;

        // 记录上一帧显示的节点，用于差量更新
        std::unordered_set<OctreeNode*> m_current_visible_nodes;

        // 相机状态
        Eigen::Vector3f m_last_cam_pos;
        double m_last_cam_dir[3];
        bool m_force_update = false;

        int m_last_point_size = -1;
        float m_last_opacity = -1.0f;

        // 交互状态与动态阈值
        bool m_is_interacting = false;
        bool m_first_update = true;  // 首次 update 标记，限制 Actor 创建量避免 UI 卡死
        // 分裂阈值，屏幕上小于这个像素宽度的方块，就不再加载细节，直接看 LOD。值越小细节越丰富，CPU 遍历压力越大。
        float m_base_threshold = 100.0f; // 基础阈值 (像素)
        // 限制同屏渲染的最大点数
        size_t m_point_budget = 20000000; // 默认 1000万点

        // 缓存驱逐
        std::unordered_map<OctreeNode*, int> m_hidden_frames; // 每个隐藏节点的连续不可见帧数
        size_t m_max_cached_actors = 500;                     // 自适应驱逐阈值
        static constexpr int EVICTION_FRAMES = 5;            // 连续隐藏多少帧后可驱逐
    };

}// namespace pw

#endif //POINTWORKS_OCTREERENDERER_H
