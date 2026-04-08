#ifndef CT_ALGORITHM_NORMALS_H
#define CT_ALGORITHM_NORMALS_H

#include "core/cloud.h"

#include <functional>
#include <atomic>
#include <string>

namespace ct
{
    struct NormalsResult {
        Cloud::Ptr cloud;         // 带法线的点云
        float time_ms = 0;
        std::string error_msg;    // 非空表示执行失败
    };

    class Normals
    {
    public:
        /**
         * @brief 法线估计
         * @param cloud 输入点云
         * @param k_search K近邻搜索数量（0 表示不使用）
         * @param radius_search 半径搜索（0 表示不使用）
         * @param vpx, vpy, vpz 视点坐标
         * @param cancel 取消标志
         * @param on_progress 进度回调 (0~100)
         */
        static NormalsResult estimate(const Cloud::Ptr& cloud,
                                      int k_search, double radius_search,
                                      float vpx, float vpy, float vpz,
                                      bool reverse = false,
                                      std::atomic<bool>* cancel = nullptr,
                                      std::function<void(int)> on_progress = nullptr);
    };

} // namespace ct

#endif // CT_ALGORITHM_NORMALS_H
