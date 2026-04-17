//
// Created by LBC on 2026/1/28.
//

#include "octreerenderer.h"

#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkCamera.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkProperty.h>
#include <vtkIdTypeArray.h>
#include <omp.h>

namespace ct {

// 计算节点在屏幕上的投影大小（近似值）
    float screenSizeInPixels(const Box &box, const Eigen::Vector3f &camPos, vtkRenderer *renderer) {
        // 获取节点大小
        float size = std::max({box.width, box.height, box.depth});

        // 计算距离
        float dist = (box.translation - camPos).norm();
        if (dist < 1e-6f) return 1e6f;  // 非常近，返回大值

        // 简化的投影大小计算：size / distance * 视野因子
        // 这假设标准FOV，实际应该使用相机FOV
        float projected = size / dist;

        // 获取渲染器大小来转换为像素
        int *winSize = renderer->GetSize();
        float minDim = std::min(winSize[0], winSize[1]);

        // 放大因子，让投影值更直观
        return projected * minDim * 0.5f;
    }

    OctreeRenderer::OctreeRenderer(Cloud::Ptr cloud, vtkRenderer *renderer)
            : m_cloud(cloud), m_vtk_renderer(renderer) {
        // 初始化时读取 CloudConfig
        if (m_cloud) {
            // 读取预算
            m_point_budget = m_cloud->getConfig().pointBudget; // 默认 1000w
            if (m_point_budget == 0) m_point_budget = 10000000; // 保底

            // 如果是直通模式(无八叉树)，不需要阈值
            // 如果有八叉树，设置一个合理的基础阈值
            m_base_threshold = 80.0f;

            m_last_point_size = m_cloud->pointSize();
            m_last_opacity = m_cloud->opacity();
        }
    }

    OctreeRenderer::~OctreeRenderer() {
        // 移除所有 Actor
        if (m_vtk_renderer) {
            for (auto& pair : m_actor_cache) {
                m_vtk_renderer->RemoveActor(pair.second);
            }
        }
        m_actor_cache.clear();
    }

    void OctreeRenderer::setVisibility(bool visible) {
        m_visible = visible;
        for (auto& pair : m_actor_cache) {
            // 如果整体不可见，隐藏所有；否则只显示当前活跃列表里的
            if (!visible) {
                pair.second->SetVisibility(0);
            } else if (m_current_visible_nodes.count(pair.first)) {
                pair.second->SetVisibility(1);
            }
        }
    }

    void OctreeRenderer::invalidateCache() {
        // 强制清理所有 Actor，通常用于颜色改变或点云重置
        for (auto& pair : m_actor_cache) {
            m_vtk_renderer->RemoveActor(pair.second);
        }
        m_actor_cache.clear();
        m_current_visible_nodes.clear();
        m_force_update = true;
    }

    void OctreeRenderer::setInteractionState(bool is_interacting)
    {
        if (m_is_interacting != is_interacting) {
            m_is_interacting = is_interacting;
            // 状态改变意味着渲染策略改变，强制更新一帧
            m_force_update = true;
            // 注意：这里不直接调用 update()，而是设置标记等待下一次 RenderEvent
        }
    }

    std::vector<vtkActor*> OctreeRenderer::getActiveActors() const {
        std::vector<vtkActor*> actors;
        for (auto* node : m_current_visible_nodes) {
            if (m_actor_cache.count(node)) {
                actors.push_back(m_actor_cache.at(node));
            }
        }
        return actors;
    }

    CloudBlock::Ptr OctreeRenderer::getBlockFromActor(vtkActor* actor) {
        // 反查 (性能较低，仅用于拾取，平时不调用)
        for (auto& pair : m_actor_cache) {
            if (pair.second == actor) {
                return pair.first->m_block; // 注意：如果是 LOD 节点，这里 m_block 可能为空或是父节点
            }
        }
        return nullptr;
    }

    void OctreeRenderer::update()
    {
        if (!m_cloud || !m_visible) return;

        // 属性同步
        if (m_last_point_size != m_cloud->pointSize() || m_last_opacity != m_cloud->opacity()) {
            m_last_point_size = m_cloud->pointSize();
            m_last_opacity = m_cloud->opacity();
            for (auto& pair : m_actor_cache) {
                pair.second->GetProperty()->SetPointSize(m_last_point_size);
                pair.second->GetProperty()->SetOpacity(m_last_opacity);
            }
            m_force_update = true;
        }

        OctreeNode* root = m_cloud->getOctreeRoot(); // 获取根节点

        // 直通模式处理
        if (!m_cloud->getConfig().enableOctree && root && root->isLeaf()) {
            if (m_current_visible_nodes.empty()) {
                vtkActor* actor = getOrCreateActor(root, false);
                if (actor) {
                    actor->SetVisibility(1);
                    m_current_visible_nodes.insert(root);
                }
            }
            return;
        }

        if (!root) return;

        // 获取相机参数
        vtkCamera* cam = m_vtk_renderer->GetActiveCamera();
        double* pos = cam->GetPosition();
        double* dir = cam->GetDirectionOfProjection();
        Eigen::Vector3f camPos(pos[0], pos[1], pos[2]);

        // 相机未动检测
        bool camChanged = (m_last_cam_pos - camPos).norm() > 0.01 ||
                          std::abs(m_last_cam_dir[0] - dir[0]) > 0.001;

        if (!camChanged && !m_force_update) return;

        m_last_cam_pos = camPos;
        m_last_cam_dir[0] = dir[0]; m_last_cam_dir[1] = dir[1]; m_last_cam_dir[2] = dir[2];
        m_force_update = false;

        // 准备投影参数
        double planes[24];
        cam->GetFrustumPlanes(m_vtk_renderer->GetTiledAspectRatio(), planes);

        int* winSize = m_vtk_renderer->GetSize();
        int height = winSize[1];
        if (height < 1) height = 1;
        double fov = cam->GetViewAngle();
        float pixelsPerUnit = height / (2.0f * std::tan(fov * 0.5 * 0.0174532925));

        // =========================================================
        // 动态策略配置
        // =========================================================
        float effective_threshold;
        size_t effective_budget;

        if (m_is_interacting) {
            // 交互模式：阈值大，预算低
            effective_threshold = 300.0f;
            effective_budget = 5000000;
        } else {
            // 静止模式：阈值小，预算高
            effective_threshold = m_base_threshold; // e.g. 80.0f
            effective_budget = m_point_budget;      // e.g. 1000w
        }

        // 3. 基于优先级的遍历
        std::vector<OctreeNode*> visibleNodeVector;
        visibleNodeVector.reserve(2000);

        std::priority_queue<PriorityNode> queue;

        // 根节点入队
        float rootSize = projectSize(m_cloud->getOctreeRoot()->m_box, camPos, pixelsPerUnit);
        queue.push({m_cloud->getOctreeRoot(), rootSize});

        size_t current_points = 0;

        while (!queue.empty()) {
            PriorityNode top = queue.top();
            queue.pop();

            OctreeNode* node = top.node;
            float size = top.screenSize;

            // 视锥体剔除
            if (!isBoxInFrustum(node->m_box, planes)) continue;

            // 【关键修复】预算检查
            // 我们不再因为超预算而 break，而是用它来决定“是否继续分裂”
            bool budget_allows_split = (current_points < effective_budget);

            // 分裂决策：
            // 1. 屏幕投影够大 (需要细节)
            // 2. 有子节点 (可以分裂)
            // 3. 还有预算
            bool should_split = (size > effective_threshold) && node->hasChildren() && budget_allows_split;

            if (should_split) {
                // 分裂：将子节点加入队列，争取更好的画质
                for (int i = 0; i < 8; ++i) {
                    if (node->m_children[i]) {
                        float childSize = projectSize(node->m_children[i]->m_box, camPos, pixelsPerUnit);
                        queue.push({node->m_children[i], childSize});
                    }
                }
            }
            else {
                // 渲染：
                // 情况 A: 已经是叶子，没法分了 -> 渲染
                // 情况 B: 距离够远(size小)，不需要分 -> 渲染 LOD
                // 情况 C: 【重点】虽然离得近，但没预算了 -> 只能渲染当前的 LOD 兜底！
                // 这样保证了所有视野内的块，至少会画出一个 LOD，绝不会出现“空洞”。

                visibleNodeVector.push_back(node);

                // 统计点数
                if (node->isLeaf()) {
                    if (node->m_block) current_points += node->m_block->size();
                } else {
                    current_points += node->m_lod_points.size();
                }
            }
        }

        // 4. 差异化更新 Actor (Diff Update)
        std::unordered_set<OctreeNode*> next_visible_set;
        next_visible_set.reserve(visibleNodeVector.size());

        for (OctreeNode* node : visibleNodeVector) {
            next_visible_set.insert(node);
            // 新增显示
            if (m_current_visible_nodes.find(node) == m_current_visible_nodes.end()) {
                vtkActor* actor = getOrCreateActor(node, !node->isLeaf());
                if (actor) actor->SetVisibility(1);
            }
        }

        for (OctreeNode* oldNode : m_current_visible_nodes) {
            // 隐藏旧的
            if (next_visible_set.find(oldNode) == next_visible_set.end()) {
                auto it = m_actor_cache.find(oldNode);
                if (it != m_actor_cache.end()) {
                    it->second->SetVisibility(0);
                }
            }
        }

        m_current_visible_nodes = std::move(next_visible_set);
    }

    vtkActor* OctreeRenderer::getOrCreateActor(OctreeNode* node, bool is_lod)
    {
        auto it = m_actor_cache.find(node);
        if (it != m_actor_cache.end()) return it->second;

        // 创建数据
        auto points = vtkSmartPointer<vtkPoints>::New();
        auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");

        if (is_lod) {
            const auto& src = node->m_lod_points;
            size_t n = src.size();
            points->SetNumberOfPoints(n);
            colors->SetNumberOfTuples(n);

            // 即使是 LOD，如果点数很多，也可以考虑并行
            float* ptr_xyz = static_cast<float*>(points->GetVoidPointer(0));
            unsigned char* ptr_rgb = colors->GetPointer(0);

            for (size_t i = 0; i < n; ++i) {
                const auto& p = src[i];
                ptr_xyz[0]=p.x; ptr_xyz[1]=p.y; ptr_xyz[2]=p.z; ptr_xyz+=3;
                ptr_rgb[0]=p.r; ptr_rgb[1]=p.g; ptr_rgb[2]=p.b; ptr_rgb+=3;
            }
        }
        else {
            auto block = node->m_block;
            if (!block) return nullptr;
            size_t n = block->size();
            points->SetNumberOfPoints(n);
            colors->SetNumberOfTuples(n);

            float* ptr_xyz = static_cast<float*>(points->GetVoidPointer(0));
            unsigned char* ptr_rgb = colors->GetPointer(0);
            const auto& pts = block->m_points;
            const auto* cols = (block->m_colors) ? block->m_colors.get() : nullptr;

            // Block 数据拷贝通常是大数据量，使用 OpenMP 加速
            // 注意：VTK 指针操作并非线程安全，但写入不同的内存地址是安全的
            // 这里的 ptr_xyz 是原始 float 指针
#pragma omp parallel for
            for (int i = 0; i < (int)n; ++i) {
                const auto& p = pts[i];
                // 手动计算偏移
                float* local_xyz = ptr_xyz + i * 3;
                unsigned char* local_rgb = ptr_rgb + i * 3;

                local_xyz[0]=p.x; local_xyz[1]=p.y; local_xyz[2]=p.z;
                if(cols) {
                    local_rgb[0]=(*cols)[i].r; local_rgb[1]=(*cols)[i].g; local_rgb[2]=(*cols)[i].b;
                } else {
                    local_rgb[0]=255; local_rgb[1]=255; local_rgb[2]=255;
                }
            }
        }

        auto polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);
        polyData->GetPointData()->SetScalars(colors);

        // 构造顶点 Cells
        vtkNew<vtkIdTypeArray> cells;
        cells->SetNumberOfValues(points->GetNumberOfPoints() * 2);
        vtkIdType* ids = cells->GetPointer(0);

        // 这里也可以并行填充
#pragma omp parallel for
        for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
            ids[i*2] = 1;
            ids[i*2+1] = i;
        }

        vtkNew<vtkCellArray> cellArray;
        cellArray->SetCells(points->GetNumberOfPoints(), cells);
        polyData->SetVerts(cellArray);

        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polyData);
        mapper->SetScalarModeToUsePointData();
        mapper->SetColorModeToDirectScalars();
        mapper->StaticOn(); // 开启静态优化

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetPointSize(m_cloud->pointSize());
        actor->GetProperty()->SetOpacity(m_cloud->opacity());
        actor->SetVisibility(0);

        m_vtk_renderer->AddActor(actor);
        m_actor_cache[node] = actor;

        return actor;
    }

    bool OctreeRenderer::isBoxInFrustum(const Box& box, const double* planes)
    {
        // AABB 视锥体剔除
        // 获取 Box 的 8 个角点
        double minX = box.translation.x() - box.width * 0.5;
        double maxX = box.translation.x() + box.width * 0.5;
        double minY = box.translation.y() - box.height * 0.5;
        double maxY = box.translation.y() + box.height * 0.5;
        double minZ = box.translation.z() - box.depth * 0.5;
        double maxZ = box.translation.z() + box.depth * 0.5;

        // 检查 6 个平面
        for (int i = 0; i < 6; ++i) {
            const double* p = planes + i * 4; // nx, ny, nz, d

            // p-vertex (方向一致的最远点)
            // 如果 p-vertex 在平面外（距离 < 0），则整个 Box 在平面外
            double px = (p[0] > 0) ? maxX : minX;
            double py = (p[1] > 0) ? maxY : minY;
            double pz = (p[2] > 0) ? maxZ : minZ;

            double dist = p[0]*px + p[1]*py + p[2]*pz + p[3];
            if (dist < 0) return false; // Cull
        }
        return true;
    }

    float OctreeRenderer::projectSize(const Box& box, const Eigen::Vector3f& camPos, float pixelsPerUnit)
    {
        // 计算包围盒中心到相机的距离
        float dist = (box.translation - camPos).norm();
        if (dist < 1e-6f) return 1e6f;

        // 近似投影大小：(包围盒最大边长 / 距离) * 像素密度
        float maxSize = std::max({box.width, box.height, box.depth});
        return (maxSize / dist) * pixelsPerUnit;
    }

} // namespace ct
