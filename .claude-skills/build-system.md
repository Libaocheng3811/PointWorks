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

**Debug 后缀**: Debug 版本库自动添加 `d` 后缀（如 `pw_cored.lib`）

## 编译器标志 (MSVC)

- `/MP` - 多处理器编译
- `/fp:precise` - 精确浮点运算
- `/bigobj` - 支持大对象文件
- `/wd4996` - 禁用 CRT 安全警告
- `/source-charset:utf-8` - 源文件 UTF-8 编码

## 库结构

项目包含 7 个 CMake 库目标 + 1 个可执行文件目标：

### pw_core (libs/core/CMakeLists.txt) — SHARED
```cmake
add_library(pw_core SHARED ...)
target_link_libraries(pw_core
    PUBLIC ${PCL_LIBRARIES}
    PRIVATE LASlib OpenMP::OpenMP_CXX)
```

### pw_viz (libs/viz/CMakeLists.txt) — SHARED
```cmake
add_library(pw_viz SHARED ...)
target_link_libraries(pw_viz
    PUBLIC pw_core pw_io Qt5::Widgets ${VTK_LIBRARIES} ${PCL_LIBRARIES})
```

> `pw_viz` 仍 PUBLIC 依赖 `pw_io`（因为部分渲染功能需要文件路径解析）。

### pw_io (libs/io/CMakeLists.txt) — SHARED
```cmake
add_library(pw_io SHARED ...)
target_link_libraries(pw_io
    PUBLIC pw_core Qt5::Widgets
    PRIVATE LASlib E57Format)
```

> `textured_mesh.h` 已从 `libs/io/` 移至 `libs/core/`。

### pw_algorithm (libs/algorithm/CMakeLists.txt) — STATIC
```cmake
add_library(pw_algorithm STATIC ...)
target_link_libraries(pw_algorithm
    PUBLIC pw_core
    PRIVATE ${PCL_LIBRARIES} CSF_Lib OpenMP::OpenMP_CXX)
```

> **零 VTK 依赖**。VTK 渲染预处理已移至 `libs/viz/surface_viz_helper.cpp`。

### pw_ui_dialog (libs/ui/CMakeLists.txt) — STATIC
```cmake
add_library(pw_ui_dialog STATIC ...)
target_link_libraries(pw_ui_dialog PUBLIC pw_core Qt5::Widgets)
```

### pw_ui_base (libs/ui/CMakeLists.txt) — STATIC
```cmake
add_library(pw_ui_base STATIC ...)
target_link_libraries(pw_ui_base
    PUBLIC pw_ui_dialog pw_core pw_viz
    PRIVATE pw_io Qt5::Widgets Qt5::Charts)
```

> **`pw_io` 为 PRIVATE**：仅 `cloud_io_controller.cpp` 使用。所有 `pw_ui_base` 公共头文件不含 `io/` 路径引用。

### pw_python (libs/python/CMakeLists.txt) — OBJECT
```cmake
add_library(pw_python OBJECT ...)
target_link_libraries(pw_python
    PRIVATE Qt5::Core pybind11::embed Python3::Python pw_core pw_algorithm)
```

### pointworks (src/CMakeLists.txt) — 可执行文件
```cmake
add_executable(pointworks WIN32 ...)
target_link_libraries(pointworks PRIVATE
    pw_core pw_viz pw_io pw_algorithm pw_ui_base pw_python
    Qt5::Widgets Qt5::Concurrent ${VTK_LIBRARIES} ${PCL_LIBRARIES}
    pybind11::embed Python3::Python)
```

## 第三方库配置

### LAStools
```cmake
set(LASZIP_BUILD_STATIC ON CACHE BOOL "" FORCE)
set(LASLIB_BUILD_STATIC ON CACHE BOOL "" FORCE)
add_subdirectory(3rdparty/LAStools)
target_link_libraries(pw_core PRIVATE LASlib)
```

### CSF
```cmake
add_library(CSF_Lib STATIC ...)
target_link_libraries(pw_algorithm PRIVATE CSF_Lib)
```

### pybind11
```cmake
add_subdirectory(3rdparty/pybind11)
```

### E57Format + Xerces-C（预编译）
```cmake
# E57 格式依赖 Xerces-C XML 解析库，两者均以预编译静态库方式引入
add_library(E57Format STATIC IMPORTED)
set_target_properties(E57Format PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/3rdparty/libE57/lib/E57Format.lib
    INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/3rdparty/libE57/include
)
add_library(xerces-c STATIC IMPORTED)
set_target_properties(xerces-c PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/3rdparty/Xerces-C/lib/xerces-c_3.lib
    INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/3rdparty/Xerces-C/include
)
# pw_io 链接 E57Format 和 xerces-c
target_link_libraries(pw_io PRIVATE E57Format xerces-c)
```

## 文件格式支持

| 格式 | 扩展名 | 读取 | 写入 | 说明 |
|------|--------|------|------|------|
| LAS | .las | Y | Y | ASPRS LAS 格式（LASlib） |
| LAZ | .laz | Y | Y | 压缩 LAS 格式（LASlib） |
| E57 | .e57 | Y | Y | 工业扫描格式（E57Format） |
| PLY | .ply | Y | Y | Stanford PLY 格式（点云+网格） |
| PCD | .pcd | Y | Y | PCL 点云格式 |
| TXT | .txt/.xyz/.asc | Y | Y | 文本格式（支持字段映射和交互式配置） |
| OBJ | .obj | Y | Y | Wavefront OBJ 格式（支持纹理检测） |
| STL | .stl | Y | Y | STL 网格格式 |
| VTK | .vtk | Y | Y | VTK PolyData 格式 |
| IFS | .ifs | Y | - | IFS 网格格式 |

## 预处理器定义

| 宏 | 值 | 说明 |
|----|----|------|
| `ROOT_PATH` | 项目根目录 | 项目源码路径 |
| `DATA_PATH` | data/ | 数据文件路径 |
| `PYTHON_HOME` | Python 安装目录 | 嵌入式 Python 解释器路径 |
| `PW_BUILDING_PW_CORE` | - | 标记 pw_core 库构建 |
| `PW_BUILDING_PW_IO` | - | 标记 pw_io 库构建 |
| `PW_BUILDING_PW_VIZ` | - | 标记 pw_viz 库构建 |
