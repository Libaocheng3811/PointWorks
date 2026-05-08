// python_bindings.cpp — pybind11 嵌入模块入口
// 具体绑定实现按功能拆分到 bindings/ 目录下各 bind_*.cpp 文件

// Qt 的 <QObject> 定义了 slots 宏，与 Python 的 object.h 冲突
#undef slots
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

#include "bindings/bind_core.h"
#include "bindings/bind_view.h"
#include "bindings/bind_appearance.h"
#include "bindings/bind_overlay.h"
#include "bindings/bind_cloud_mgmt.h"
#include "bindings/bind_progress.h"
#include "bindings/bind_filters.h"
#include "bindings/bind_csf_veg.h"
#include "bindings/bind_distance.h"
#include "bindings/bind_features.h"
#include "bindings/bind_registration.h"
#include "bindings/bind_segmentation.h"
#include "bindings/bind_surface.h"
#include "bindings/bind_normals.h"
#include "bindings/bind_keypoints.h"
#include "bindings/bind_transform.h"

namespace py = pybind11;

PYBIND11_EMBEDDED_MODULE(pw, m)
{
    registerCoreBindings(m);
    registerViewBindings(m);
    registerAppearanceBindings(m);
    registerOverlayBindings(m);
    registerCloudMgmtBindings(m);
    registerProgressBindings(m);
    registerFilterBindings(m);
    registerCsfVegBindings(m);
    registerDistanceBindings(m);
    registerFeatureBindings(m);
    registerRegistrationBindings(m);
    registerSegmentationBindings(m);
    registerSurfaceBindings(m);
    registerNormalBindings(m);
    registerKeypointBindings(m);
    registerTransformBindings(m);
}
