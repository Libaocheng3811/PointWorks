#ifndef POINTWORKS_COLORMAP_H
#define POINTWORKS_COLORMAP_H

#include "exports.h"

#include <string>
#include <vector>
#include <QString>
#include <QStringList>

namespace ct {

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

inline QStringList colormapNames()
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

inline QString colormapName(ColormapType type)
{
    const char* names[] = {
        "jet", "turbo", "hot", "cool",
        "brown-yellow", "blue-red", "green-red", "greyscale",
        "bwr", "rdylgn"
    };
    int idx = static_cast<int>(type);
    if (idx >= 0 && idx < static_cast<int>(ColormapType::COUNT))
        return QString(names[idx]);
    return "jet";
}

inline ColormapType colormapFromName(const QString& name)
{
    auto names = colormapNames();
    int idx = names.indexOf(name);
    if (idx >= 0) return static_cast<ColormapType>(idx);
    return ColormapType::JET;
}

inline bool isDivergingColormap(ColormapType type)
{
    return type == ColormapType::BWR || type == ColormapType::RDYLGN;
}

// LUT data: 1024 entries, each float packs RGB as (R<<16)|(G<<8)|B
using ColormapLUT = std::vector<float>;

CT_CORE_EXPORT ColormapLUT buildColormapLUT(ColormapType type);

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

}  // namespace ct

#endif  // POINTWORKS_COLORMAP_H
