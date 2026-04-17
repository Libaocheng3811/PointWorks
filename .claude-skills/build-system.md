# 构建系统与 CMake 配置

## 环境要求

CMake 3.28+, C++17, Qt5.15.2, VTK 9.1.0, PCL 1.12.1, OpenMP, Python 3.9 (EXACT), pybind11

## 构建命令

```bash
# 配置（在项目根目录）
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# 构建
cmake --build build --config Release
```

**输出目录**:
- `build/bin/` - 可执行文件 (.exe) 和动态库 (.dll)
- `build/lib/` - 静态库 (.lib)

**Debug 后缀**: Debug 版本库自动添加 `d` 后缀（如 `ct_cored.lib`）

## 编译器标志 (MSVC)

- `/MP` - 多处理器编译（根据 CPU 核心数自动设置）
- `/fp:precise` - 精确浮点运算
- `/bigobj` - 支持大对象文件
- `/wd4996` - 禁用 CRT 安全警告
- `/source-charset:utf-8` - 源文件 UTF-8 编码

## CMake 配置选项

```cmake
# Qt 路径（如需要手动指定）
set(CMAKE_PREFIX_PATH "D:/Qt5.15/5.15.2/msvc2019_64")

# OpenMP 支持
option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)

# Python 3.9（硬编码路径，需根据本地环境修改）
set(MY_NATIVE_PYTHON_DIR "F:/Program Files/Python39")
set(Python3_ROOT_DIR ${MY_NATIVE_PYTHON_DIR})
```

## 库结构

项目包含 7 个 CMake 库目标 + 1 个可执行文件目标：

### ct_core (libs/core/CMakeLists.txt) — SHARED
```cmake
add_library(ct_core SHARED ...)
target_link_libraries(ct_core
    PUBLIC Qt5::Widgets ${PCL_LIBRARIES}
    PRIVATE LASlib OpenMP::OpenMP_CXX)
target_compile_definitions(ct_core PRIVATE CT_LIBRARY)
```

### ct_viz (libs/viz/CMakeLists.txt) — SHARED
```cmake
add_library(ct_viz SHARED ...)
target_link_libraries(ct_viz
    PRIVATE ct_core ct_io Qt5::Widgets ${VTK_LIBRARIES} ${PCL_LIBRARIES})
```

### ct_io (libs/io/CMakeLists.txt) — SHARED
```cmake
add_library(ct_io SHARED ...)
target_link_libraries(ct_io
    PUBLIC ct_core Qt5::Widgets
    PRIVATE LASlib E57Format)
target_compile_definitions(ct_io PRIVATE CT_BUILDING_CT_IO)
```

> ct_io 使用 `file(GLOB *.cpp)` 自动收集源文件，新增 .cpp 无需手动注册。

### ct_algorithm (libs/algorithm/CMakeLists.txt) — STATIC
```cmake
add_library(ct_algorithm STATIC ...)
target_link_libraries(ct_algorithm
    PRIVATE ct_core CSF_Lib ${PCL_LIBRARIES} ${VTK_LIBRARIES} OpenMP::OpenMP_CXX)
```

### ct_ui_dialog (libs/ui/CMakeLists.txt) — STATIC
```cmake
add_library(ct_ui_dialog STATIC ...)
target_link_libraries(ct_ui_dialog PRIVATE ct_core Qt5::Widgets)
```

### ct_ui_base (libs/ui/CMakeLists.txt) — STATIC
```cmake
add_library(ct_ui_base STATIC ...)
target_link_libraries(ct_ui_base
    PUBLIC ct_ui_dialog ct_core ct_viz ct_io Qt5::Widgets)
```

### ct_python (libs/python/CMakeLists.txt) — OBJECT
```cmake
add_library(ct_python OBJECT ...)
target_link_libraries(ct_python
    PRIVATE Qt5::Widgets pybind11::embed Python3::Python ct_core ct_algorithm)
```

### pointworks (src/CMakeLists.txt) — 可执行文件
```cmake
add_executable(pointworks WIN32 ...)
target_link_libraries(pointworks PRIVATE
    ct_core ct_viz ct_io ct_algorithm ct_ui_base ct_python
    Qt5::Widgets Qt5::Concurrent ${VTK_LIBRARIES} ${PCL_LIBRARIES}
    pybind11::embed Python3::Python)
```

## 第三方库配置

### LAStools
```cmake
set(LASZIP_BUILD_STATIC ON CACHE BOOL "Build static laszip" FORCE)
set(LASLIB_BUILD_STATIC ON CACHE BOOL "Build static laslib" FORCE)
add_subdirectory(3rdparty/LAStools)
target_link_libraries(ct_core PRIVATE LASlib)
```

### CSF
```cmake
# 3rdparty/CSF/CMakeLists.txt:
file(GLOB CSF_Srcs "src/*.cpp")
file(GLOB CSF_Hdrs "src/*.h")
add_library(CSF_Lib STATIC ${CSF_Srcs} ${CSF_Hdrs})
target_link_libraries(CSF_Lib PRIVATE OpenMP::OpenMP_CXX)

# 链接
target_link_libraries(ct_algorithm PRIVATE CSF_Lib)
```

### pybind11
```cmake
add_subdirectory(3rdparty/pybind11)
# ct_python 通过 pybind11::embed 链接
```

## 文件格式支持

| 格式 | 扩展名 | 读取 | 写入 | 说明 |
|------|--------|------|------|------|
| LAS | .las | √ | √ | ASPRS LAS 格式（LASlib） |
| LAZ | .laz | √ | √ | 压缩 LAS 格式（LASlib） |
| E57 | .e57 | √ | √ | 工业扫描格式（E57Format） |
| PLY | .ply | √ | √ | Stanford PLY 格式（点云+网格） |
| PCD | .pcd | √ | √ | PCL 点云格式 |
| TXT | .txt/.xyz/.asc | √ | √ | 文本格式（支持字段映射和交互式配置） |
| OBJ | .obj | √ | √ | Wavefront OBJ 格式（支持纹理检测） |
| STL | .stl | √ | √ | STL 网格格式 |
| VTK | .vtk | √ | √ | VTK PolyData 格式 |
| IFS | .ifs | √ | - | IFS 网格格式 |

## 预处理器定义

| 宏 | 值 | 说明 |
|----|----|------|
| `ROOT_PATH` | 项目根目录 | 项目源码路径 |
| `DATA_PATH` | data/ | 数据文件路径 |
| `PYTHON_HOME` | Python 安装目录 | 嵌入式 Python 解释器路径 |
| `CT_LIBRARY` | - | 标记库导出（用于 Windows DLL） |
