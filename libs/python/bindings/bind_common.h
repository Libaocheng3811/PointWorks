#pragma once

// 所有 bind_*.cpp 的公共头文件
// 提供 pybind11 include 和 PythonManager/Bridge 访问

#include "python_manager.h"
#include "python_bridge.h"
#include "core/cloud.h"
#include "core/octree.h"

// Qt 的 <QObject> 定义了 slots 宏，与 Python 的 object.h 冲突
#undef slots
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;
