#ifndef POINTWORKS_CLOUDVIEW_H
#define POINTWORKS_CLOUDVIEW_H

#include "core/cloud.h"
#include "core/exports.h"
#include "io/projectfile.h"
#include "octreerenderer.h"

#include "QVTKOpenGLNativeWidget.h"
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkSmartPointer.h>
#include <vtkCallbackCommand.h>

class vtkBillboardTextActor3D;
#include "viewcube.h"

#include <QMap>

#include <memory>

namespace ct {

    struct PointXY
    {
        PointXY(int x, int y) : x(x), y(y) {}
        bool operator==(const PointXY& pt) const
        {
            return (this->x == pt.x) && (this->y == pt.y);
        }
        bool operator!=(const PointXY& pt) const { return !(*this == pt); }
        float x = 0.0f;
        float y = 0.0f;
    };

    struct PickResult {
        bool valid = false;          // 是否拾取成功
        ct::PointXYZRGBN point;      // 拾取点的坐标、颜色、法线
        ct::Cloud::Ptr cloud;        // 所属的点云对象

        // 标量场数据 (Key: 字段名, Value: 数值)
        QMap<QString, float> scalars;
    };

    class CT_VIZ_EXPORT CloudView : public QVTKOpenGLNativeWidget{
        Q_OBJECT
    public:
        explicit CloudView(QWidget* parent = nullptr);

        ~CloudView() override;

        ////////////////////////////////////////////////////////
        // add
        /**
         * @brief 添加点云
         */
        void addPointCloud(const Cloud::Ptr& cloud);

        /**
         * @brief 从深度图中添加点云
         */
        void addPointCloudFromRangeImage(const pcl::RangeImage::Ptr &image, const QString& id, const ColorRGB& rgb = Color::White);

        /**
         * @brief 添加点云包围盒
         */
        void addBox(const Cloud::Ptr& cloud);

        /**
         * @brief 添加点云法线
         * todo 法线的显示需要适配，暂时保留PCL 接口用于调试，或者后续重写为基于 Octree 的法线渲染
         */
        void addPointCloudNormals(const Cloud::Ptr& cloud, int level, float scale);

        /**
         * @brief 添加点对的对应关系
         */
        void addCorrespondences(const Cloud::Ptr& source_points, const Cloud::Ptr& target_points,
                                const pcl::CorrespondencesPtr& correspondences, const QString& id = "correspondences",
                                int line_width = 1);

        /**
         * @brief 添加多边形
         */
        void addPolygon(const Cloud::Ptr& cloud,
                        const QString& id = "polygon", const ColorRGB& rgb = Color::White);

        /**
         * @brief 添加箭头
         */
        void addArrow(const PointXYZRGBN& pt1, const PointXYZRGBN& pt2,
                      const QString& id = "arrow", bool display_length = false, const ColorRGB& rgb = Color::White);

        /**
         * @brief 添加立方体
         */
        void addCube(const pcl::ModelCoefficients::Ptr& coefficients,
                     const QString& id = "cube");

        /**
         * @brief 添加立方体
         */
        void addCube(const PointXYZRGBN& min, PointXYZRGBN& max,
                     const QString& id = "cube", const ColorRGB& rgb = Color::White);

        /**
         * @brief 添加立方体
         */
        void addCube(const Box& box, const QString& id = "cube");

        /**
         * @brief 添加 3D 文本标签 (带引线和背景框)
         * @param pos 3D坐标点
         * @param text 显示的文本
         * @param id 标签的ID
         * @param r, g, b 文本颜色 (0-1.0)
         */
        void add3DLabel(const PointXYZRGBN& pos, const QString& text, const QString& id,
                        double scale = 0.03, double r = 1.0, double g = 1.0, double b = 0.0);

        /**
         * @brief 添加 3D 号码牌标签（带背景色，始终可见不被点云遮挡）
         * @param pos 3D坐标点
         * @param text 显示的文本
         * @param id 标签的ID
         * @param scale 字体缩放
         * @param textR/textG/textB 文字颜色 (0-1.0)
         * @param bgR/bgG/bgB 背景色 (0-1.0)
         * @param bgOpacity 背景不透明度 (0-1.0)
         */
        void add3DBadge(const PointXYZRGBN& pos, const QString& text, const QString& id,
                        int font_size = 24,
                        double textR = 1.0, double textG = 0.0, double textB = 0.0,
                        double bgR = 1.0, double bgG = 1.0, double bgB = 1.0,
                        double bgOpacity = 0.8);

        void remove3DBadge(const QString& id);
        void removeAll3DBadges();

        /**
         * @brief 添加多边形网格（PolygonMesh）到视图
         * @param mesh 多边形网格
         * @param id 网格标识符
         * @param viewport 视口（默认 0 = 全部）
         */
        void addPolygonMesh(const pcl::PolygonMesh::Ptr& mesh, const QString& id, int viewport = 0);

        /**
         * @brief 从 PolygonMesh 添加线框到视图
         * @param mesh 多边形网格
         * @param id 线框标识符
         * @param viewport 视口（默认 0 = 全部）
         */
        void addPolylineFromPolygonMesh(const pcl::PolygonMesh::Ptr& mesh, const QString& id, int viewport = 0);

        ////////////////////////////////////////////////////////
        // 2D->3D(display to world)

        /**
         * @brief 屏幕2D坐标映射为3D坐标  将二维显示坐标转换成三维世界坐标
         */
        PointXYZRGBN displayToWorld(const PointXY& xy);

        /**
         * @brief 添加相对屏幕的2D多边形
         */
        void addPolygon2D(const std::vector<PointXY>& points,
                          const QString& id = "polyline", const ColorRGB& rgb = Color::White);


        // point pick

        /**
         * @brief 单点选择
         * @param p 屏幕2D坐标点
         * @return int 选中点云的点索引
         * 主要是实现鼠标点击操作选点的功能
         */
        PickResult singlePick(const PointXY& p, const QString& target_cloud_id = "");

        /**
         * @brief 多边形选取,确定哪些点位于指定的多边形区域内或外
         * @param points 屏幕2D多边形顶点
         * @param cloud 选取的点云
         * @param in_out 选择是否反向
         * @return std::vector<int> 选中点云的点索引集合
         */
        Cloud::Ptr areaPick(const std::vector<PointXY>& points, const Cloud::Ptr& cloud, bool in_out = false);


        ///////////////////////////////////////////////////////
        // remove
        /**
         * @brief 移除点云
         */
        void removePointCloud(const QString& id);

        /**
         * @brief 移除模型
         */
        void removeShape(const QString& id);

        /**
         * @brief 移除多边形网格
         * @param id 网格标识符
         * @param viewport 视口（默认 0 = 全部）
         */
        void removePolygonMesh(const QString& id, int viewport = 0);

        /**
         * @brief 移除对应关系
         */
        void removeCorrespondences(const QString& id);

        /**
         * @brief 移除所有点云
         */
        void removeAllPointClouds();

        /**
         * @brief 移除所有模型
         */
        void removeAllShapes();

        /**
         * @brief 添加坐标系
         */
        void addCoordinateSystem(const Coord& coord);

        /**
         * @brief 移除指定坐标系
         */
        void removeCoordinateSystem(const QString& id);

        /**
         * @brief 移除所有坐标系
         */
        void removeAllCoordinateSystems();

        ///////////////////////////////////////////////////////////
        // properties
        /**
         * @brief 设置点云颜色 (RGB)
         */
        void setPointCloudColor(const Cloud::Ptr& cloud, const ColorRGB& rgb = Color::White);

        /**
         * @brief 设置点云颜色 (RGB)
         */
        void setPointCloudColor(const QString& id, const ColorRGB& rgb = Color::White);

        /**
         * @brief 设置点云颜色（维度） 根据指定的坐标轴为点云设置颜色
         * 如果选择 X 坐标作为颜色依据，那么点云中每个点的颜色会根据其 X 坐标的值改变。
         * 例如，在 X 坐标为负值的点可以显示为蓝色，而正值的点显示为红色
         */
        void setPointCloudColor(const Cloud::Ptr& cloud, const QString& axis);

        /**
         * @brief 重置点云颜色，将点云颜色重置成其默认的RGB颜色
         */
        void resetPointCloudColor(const Cloud::Ptr& cloud);

        /**
         * @brief 设置点云大小
         */
        void setPointCloudSize(const QString& id, float size);

        /**
         * @brief 设置点云透明度
         */
        void setPointCloudOpacity(const QString& id, float value);

        /**
         * @brief 设置背景颜色
         */
        void setBackgroundColor(const ColorRGB& rgb = Color::White);

        /**
         * @brief 重置背景颜色
         */
        void resetBackgroundColor();

        /**
         * @brief 设置模型颜色
         */
        void setShapeColor(const QString& shapeid, const ColorRGB& rgb = Color::White);

        /**
         * @brief 设置模型点大小
         */
        void setShapeSize(const QString& shapeid, float size);

        /**
         * @brief 设置模型透明度
         */
        void setShapeOpacity(const QString& shapeid, float value);

        /**
         * @brief 设置模型线宽
         */
        void setShapeLineWidth(const QString& shapeid, float value);

        /**
         * @brief 设置模型字体大小
         */
        void setShapeFontSize(const QString& shapeid, float value);

        /**
         * @brief 设置模型表示类型
         * @param type 0-点，1-线，2-面
         */
        void setShapeRepersentation(const QString& shapeid, int type);

        /**
         * @brief 设置点云可见性
         * @param id 点云ID
         * @param visible 是否可见
         */
        void setPointCloudVisibility(const QString& id, bool visible);

        ///////////////////////////////////////////////////////////
        // camera
        /**
         * @brief 重置相机参数
         */
         void resetCamera()
        {
             m_viewer->resetCamera();
             m_render->ResetCameraClippingRange();
             m_viewer->getRenderWindow()->Render();
        }

        /// 获取当前相机参数
        CameraParams getCameraParams() const;

        /// 获取当前观察位姿 (Affine3f)
        Eigen::Affine3f getViewerPose() const;

        /// 恢复相机参数
        void setCameraParams(const CameraParams& params);

        /// 获取当前视图选项
        ViewOptions getViewOptions() const;

        /// 恢复视图选项
        void setViewOptions(const ViewOptions& opts);

        ///////////////////////////////////////////////////////////////
        // display
        /**
         * @brief 显示视图器信息
         * @param level 信息位置1-10
         */
        void showInfo(const QString& text, int level, const ColorRGB& rgb = Color::White);

        /**
         * @brief 清除视图器信息
         */
        void clearInfo();

        /**
         * @brief 显示点云ID
         */
        void showCloudId(const QString& id);

        /**
         * @brief 设置是否显示点云ID
         */
        void setShowId(const bool& enable);

        /**
         * @brief 设置是否显示帧率
         */
        void setShowFPS(const bool& enable)
        {
            m_show_fps = enable;
            m_viewer->setShowFPS(enable);
        }

        /**
         * @brief 设置是否显示坐标系小部件
         */
        void setShowAxes(const bool& enable);

        void syncViewCubeOrientation();

        ///////////////////////////////////////////////////////////
        // other
        /**
         * @brief 检查具有给定ID的点云、模型或坐标是否已添加到视图中
         */
        bool contains(const QString& id)
        {
            if (m_OctreeRenders.contains(id)) {
                return true;
            }
            // 调用pcl::visualization::PCLVisualizer::Ptr类的contains方法，返回一个bool值，表示是否包含这个id
            return m_viewer->contains(id.toStdString());
        }

        /**
         * @brief 设置是否开启交互
         */
        void setInteractorEnable(const bool& enable);

        ////////////////////////////////////////////////////////
        // viewport
        /**
         * @brief 设置为俯视图(+z)
         */
        void setTopView();

        /**
        * @brief 设置底视图(-Z)
        */
        void setBottomView();

        /**
         * @brief 设置为正视图(front)
         */
        void setFrontView();

        /**
        * @brief 设置后视图(Back)
        */
        void setBackView();

        /**
         * @brief 设置为左视图(Left)
         */
        void setLeftSideView();

        /**
         * @brief 设置右视图(Right)
         */
        void setRightSideView();

        /**
         * @brief 聚集到指定的包围盒
         *  @param min_pt 最小点 (x, y, z)
         *  @param max_pt 最大点 (x, y, z)
         */
        void zoomToBounds(const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt);

        /**
         * @brief 是否启用自动渲染
         * @param enable true=自动渲染，false=手动渲染
         */
        void setAutoRender(bool enable) { m_auto_render = enable; }

        /**
         * @brief 手动刷新渲染窗口
         */
        void refresh();

        /**
         * @brief 使指定点云的渲染缓存失效并强制重绘
         * @param cloud_id 点云 ID（与 m_OctreeRenders 的 key 对应）
         */
        void invalidateCloudRender(const QString& cloud_id);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        void mousePressEvent(QMouseEvent* event) override;
        void resizeEvent(QResizeEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event);
        void mouseMoveEvent(QMouseEvent* event);

    signals:
        void viewerPose(Eigen::Affine3f);
        void sizeChanged(const QSize& size);
        void posChanged(const QPoint& pos);
        void mouseLeftPressed(const PointXY& pt);
        void mouseLeftReleased(const PointXY& pt);
        void mouseRightPressed(const PointXY& pt);
        void mouseRightReleased(const PointXY& pt);
        void mouseMoved(const PointXY& pt);

    private:
        void setView(const Eigen::Vector3f& direction, const Eigen::Vector3f& up);

        // 渲染回调
        static void OnRenderEvent(vtkObject* caller, unsigned long eventId, void* clientData, void* callData);

        // VTK交互回调
        static void OnInteractionEvent(vtkObject* caller, unsigned long eventId, void* clientData, void* callData);

        void updateRenderers();

        // 处理交互状态切换
        void onInteraction(bool is_interacting);

    private:
        struct InfoData{
            QString text;
            ColorRGB rgb;
        };
        // Key = Level (int), Value = InfoData
        QMap<int, InfoData> m_active_infos;

    private:
        Q_DISABLE_COPY(CloudView);

        bool m_show_id;
        bool m_show_fps = true;
        int m_info_level;
        QString m_last_id;
        QString m_current_id; // 用于记录当前显示的ID字符串

        pcl::visualization::PCLVisualizer::Ptr m_viewer;
        vtkSmartPointer<vtkRenderer> m_render;
        vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderwindow;
        ViewCube* m_view_cube = nullptr;

        // 维护一个正在显示的Cloud::Ptr列表，方便进行预览模式切换
        std::vector<Cloud::Ptr> m_visible_clouds;
        bool m_auto_render = true; //默认开启自动渲染

        QMap<QString, std::shared_ptr<OctreeRenderer>> m_OctreeRenders;
        QSet<QString> m_coord_ids;
        QMap<QString, vtkSmartPointer<vtkBillboardTextActor3D>> m_badge_actors;
        unsigned long m_observer_tag = 0; // 回调 ID
        unsigned long m_interaction_tag = 0; //保存交互回调的 ID

    };
} // namespace ct

#endif //POINTWORKS_CLOUDVIEW_H
