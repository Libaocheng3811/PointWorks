#ifndef POINTWORKS_EXPORTS_H
#define POINTWORKS_EXPORTS_H

#define BOOST_ALLOW_DEPRECATED_HEADERS

// 每个 DLL 使用独立的宏，避免交叉编译时符号导出/导入混乱
// PW_BUILDING_PW_CORE  → 构建 pw_core.dll 时定义
// PW_BUILDING_PW_VIZ   → 构建 pw_viz.dll 时定义
// PW_BUILDING_PW_IO    → 构建 pw_io.dll 时定义

#if defined(_WIN32) || defined(_WIN64)
    #ifdef PW_BUILDING_PW_CORE
        #define PW_CORE_EXPORT __declspec(dllexport)
    #else
        #define PW_CORE_EXPORT __declspec(dllimport)
    #endif

    #ifdef PW_BUILDING_PW_VIZ
        #define PW_VIZ_EXPORT __declspec(dllexport)
    #else
        #define PW_VIZ_EXPORT __declspec(dllimport)
    #endif

    #ifdef PW_BUILDING_PW_IO
        #define PW_IO_EXPORT __declspec(dllexport)
    #else
        #define PW_IO_EXPORT __declspec(dllimport)
    #endif
#elif defined(__GNUC__)
    #define PW_CORE_EXPORT __attribute__((visibility("default")))
    #define PW_VIZ_EXPORT  __attribute__((visibility("default")))
    #define PW_IO_EXPORT   __attribute__((visibility("default")))
#else
    #define PW_CORE_EXPORT
    #define PW_VIZ_EXPORT
    #define PW_IO_EXPORT
#endif

// 向后兼容：未迁移的代码仍可使用 PW_EXPORT（等同于 PW_CORE_EXPORT）
#ifndef PW_EXPORT
    #define PW_EXPORT PW_CORE_EXPORT
#endif

#endif //POINTWORKS_EXPORTS_H
