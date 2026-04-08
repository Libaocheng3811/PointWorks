#ifndef POINTWORKS_EXPORTS_H
#define POINTWORKS_EXPORTS_H

#define BOOST_ALLOW_DEPRECATED_HEADERS

// 每个 DLL 使用独立的宏，避免交叉编译时符号导出/导入混乱
// CT_BUILDING_CT_CORE  → 构建 ct_core.dll 时定义
// CT_BUILDING_CT_VIZ   → 构建 ct_viz.dll 时定义
// CT_BUILDING_CT_IO    → 构建 ct_io.dll 时定义

#if defined(_WIN32) || defined(_WIN64)
    #ifdef CT_BUILDING_CT_CORE
        #define CT_CORE_EXPORT __declspec(dllexport)
    #else
        #define CT_CORE_EXPORT __declspec(dllimport)
    #endif

    #ifdef CT_BUILDING_CT_VIZ
        #define CT_VIZ_EXPORT __declspec(dllexport)
    #else
        #define CT_VIZ_EXPORT __declspec(dllimport)
    #endif

    #ifdef CT_BUILDING_CT_IO
        #define CT_IO_EXPORT __declspec(dllexport)
    #else
        #define CT_IO_EXPORT __declspec(dllimport)
    #endif
#elif defined(__GNUC__)
    #define CT_CORE_EXPORT __attribute__((visibility("default")))
    #define CT_VIZ_EXPORT  __attribute__((visibility("default")))
    #define CT_IO_EXPORT   __attribute__((visibility("default")))
#else
    #define CT_CORE_EXPORT
    #define CT_VIZ_EXPORT
    #define CT_IO_EXPORT
#endif

// 向后兼容：未迁移的代码仍可使用 CT_EXPORT（等同于 CT_CORE_EXPORT）
#ifndef CT_EXPORT
    #define CT_EXPORT CT_CORE_EXPORT
#endif

#endif //POINTWORKS_EXPORTS_H
