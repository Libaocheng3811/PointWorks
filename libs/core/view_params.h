#ifndef POINTWORKS_VIEW_PARAMS_H
#define POINTWORKS_VIEW_PARAMS_H

#include "exports.h"

#include <Eigen/Dense>

namespace ct
{

/// 相机参数（纯数据结构）
struct CameraParams
{
    double position[3] = {0, 0, 0};
    double focal_point[3] = {0, 0, 0};
    double view_up[3] = {0, 0, 1};
    double clip_near = 0.01;
    double clip_far = 100000.0;
};

/// 视图选项（纯数据结构）
struct ViewOptions
{
    bool show_fps = true;
    bool show_axes = true;
    bool show_id = true;

    // 背景色
    bool use_gradient_bg = true;
    double bg_color[3] = {0.0, 0.05, 0.08};
    double bg_color2[3] = {0.05, 0.4, 0.6};
};

} // namespace ct

#endif // POINTWORKS_VIEW_PARAMS_H
