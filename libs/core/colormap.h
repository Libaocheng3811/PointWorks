#ifndef POINTWORKS_COLORMAP_H
#define POINTWORKS_COLORMAP_H

#include "exports.h"

#include <string>
#include <vector>

namespace pw {

enum class ColormapType {
    JET = 0,
    TURBO,
    HOT,
    COOL,
    BROWN_YELLOW,
    BLUE_RED,
    GREEN_RED,
    GREYSCALE,
    BWR,
    RDYLGN,
    COUNT
};

inline std::vector<std::string> colormapNames()
{
    return {
        "jet",
        "turbo",
        "hot",
        "cool",
        "brown-yellow",
        "blue-red",
        "green-red",
        "greyscale",
        "bwr",
        "rdylgn"
    };
}

inline std::string colormapName(ColormapType type)
{
    const char* names[] = {
        "jet", "turbo", "hot", "cool",
        "brown-yellow", "blue-red", "green-red", "greyscale",
        "bwr", "rdylgn"
    };
    int idx = static_cast<int>(type);
    if (idx >= 0 && idx < static_cast<int>(ColormapType::COUNT))
        return names[idx];
    return "jet";
}

inline ColormapType colormapFromName(const std::string& name)
{
    auto names = colormapNames();
    for (size_t i = 0; i < names.size(); ++i) {
        if (names[i] == name) return static_cast<ColormapType>(i);
    }
    return ColormapType::JET;
}

inline bool isDivergingColormap(ColormapType type)
{
    return type == ColormapType::BWR || type == ColormapType::RDYLGN;
}

// LUT data: 1024 entries, each float packs RGB as (R<<16)|(G<<8)|B
using ColormapLUT = std::vector<float>;

PW_CORE_EXPORT ColormapLUT buildColormapLUT(ColormapType type);

inline const ColormapLUT& getColormapLUT(ColormapType type)
{
    static ColormapLUT lut_cache[static_cast<int>(ColormapType::COUNT)];
    static bool initialized = false;

    if (!initialized) {
        for (int i = 0; i < static_cast<int>(ColormapType::COUNT); ++i)
            lut_cache[i] = buildColormapLUT(static_cast<ColormapType>(i));
        initialized = true;
    }

    int idx = static_cast<int>(type);
    return lut_cache[idx >= 0 && idx < static_cast<int>(ColormapType::COUNT) ? idx : 0];
}

}  // namespace pw

#endif  // POINTWORKS_COLORMAP_H
