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

## CMake 库配置

### ct_core (core/CMakeLists.txt)
```cmake
add_library(ct_core SHARED ${LibSrcs} ${LibHdrs})
target_link_libraries(ct_core
    PUBLIC Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES}
    PRIVATE LASlib OpenMP::OpenMP_CXX)
target_compile_definitions(ct_core PRIVATE CT_LIBRARY)
```

### ct_modules (modules/CMakeLists.txt)
```cmake
add_library(ct_modules SHARED ${LibSrcs} ${LibHdrs})
target_link_libraries(ct_modules
    PRIVATE ct_core CSF_Lib Qt5::Widgets ${PCL_LIBRARIES})
```

### ct_widget & ct_common_ui (widgets/CMakeLists.txt)
```cmake
add_library(ct_common_ui STATIC ${UI_SOURCES} ${UI_HEADERS} ${UI_FORMS})
add_library(ct_widget STATIC ${Widget_Srcs} ${Widget_Hdrs})
target_link_libraries(ct_widget PUBLIC ct_common_ui ct_core)
```

### cloudtool (cloudtool/CMakeLists.txt)
```cmake
add_executable(cloudtool WIN32 ${Srcs} ${Hdrs} ${QRCs} ${IRCs})
target_link_libraries(cloudtool
    PRIVATE ct_core ct_widget ct_modules ct_python
            Qt5::Widgets ${VTK_LIBRARIES} pybind11::embed Python3::Python)
```

## 第三方库配置

### LAStools
```cmake
# 调用式配置
set(LASZIP_BUILD_STATIC ON CACHE BOOL "Build static laszip" FORCE)
set(LASLIB_BUILD_STATIC ON CACHE BOOL "Build static laslib" FORCE)
add_subdirectory(3rdparty/LAStools)

# 链接
target_link_libraries(ct_core PRIVATE LASlib)
```

### CSF
```cmake
# 托管式配置（自定义 CMakeLists）
# 3rdparty/CSF/CMakeLists.txt:
file(GLOB CSF_Srcs "src/*.cpp")
file(GLOB CSF_Hdrs "src/*.h")
add_library(CSF_Lib STATIC ${CSF_Srcs} ${CSF_Hdrs})
target_link_libraries(CSF_Lib PRIVATE OpenMP::OpenMP_CXX)

# 主程序链接
target_link_libraries(ct_modules PRIVATE CSF_Lib)
```

### pybind11
```cmake
# git submodule 方式引入
add_subdirectory(3rdparty/pybind11)
# ct_python 通过 pybind11::embed 链接
```

## 文件格式支持

| 格式 | 扩展名 | 读取 | 写入 | 说明 |
|------|--------|------|------|------|
| LAS | .las | √ | √ | ASPRS LAS 格式 |
| LAZ | .laz | √ | √ | 压缩 LAS 格式 |
| PLY | .ply | √ | √ | Stanford PLY 格式 |
| PCD | .pcd | √ | √ | PCL 点云格式 |
| TXT | .txt | √ | √ | 文本格式（支持字段映射） |
| OBJ | .obj | √ | - | Wavefront OBJ 格式 |

## 预处理器定义

| 宏 | 值 | 说明 |
|----|----|------|
| `ROOT_PATH` | 项目根目录 | 项目源码路径 |
| `DATA_PATH` | data/ | 数据文件路径 |
| `PYTHON_HOME` | Python 安装目录 | 嵌入式 Python 解释器路径 |
| `CT_LIBRARY` | - | 标记库导出（用于 Windows DLL） |
