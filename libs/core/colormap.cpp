#include "colormap.h"
#include <cmath>
#include <cstdint>

namespace ct {

// Helper: pack R,G,B (0-255) into a float (bit-reinterpret)
static float packRGB(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t packed = (static_cast<uint32_t>(r) << 16) |
                      (static_cast<uint32_t>(g) << 8) |
                      static_cast<uint32_t>(b);
    float f;
    std::memcpy(&f, &packed, sizeof(f));
    return f;
}

// Helper: lerp
static float lerp(float a, float b, float t) { return a + (b - a) * t; }

// Helper: clamp 0-255
static uint8_t clamp8(float v) {
    int i = static_cast<int>(v + 0.5f);
    if (i < 0) return 0;
    if (i > 255) return 255;
    return static_cast<uint8_t>(i);
}

ColormapLUT buildColormapLUT(ColormapType type)
{
    ColormapLUT lut(1024);

    for (int i = 0; i < 1024; ++i) {
        float t = static_cast<float>(i) / 1023.0f;
        uint8_t r = 0, g = 0, b = 0;

        switch (type) {
        case ColormapType::JET:
            if (t < 0.25f)      { r = 0;   g = clamp8(255 * 4 * t);       b = 255; }
            else if (t < 0.5f)  { r = 0;   g = 255;                        b = clamp8(255 * (1 - 4 * (t - 0.25f))); }
            else if (t < 0.75f) { r = clamp8(255 * 4 * (t - 0.5f)); g = 255; b = 0; }
            else                 { r = 255; g = clamp8(255 * (1 - 4 * (t - 0.75f))); b = 0; }
            break;

        case ColormapType::TURBO: {
            // Approximate turbo colormap (simplified)
            float r_f, g_f, b_f;
            if (t < 0.25f) {
                r_f = 0.133f + t * 1.068f;
                g_f = 0.0f + t * 0.4f;
                b_f = 0.533f - t * 0.667f;
            } else if (t < 0.5f) {
                float s = (t - 0.25f) * 4;
                r_f = 0.4f + s * 0.6f;
                g_f = 0.1f + s * 0.9f;
                b_f = 0.367f - s * 0.367f;
            } else if (t < 0.75f) {
                float s = (t - 0.5f) * 4;
                r_f = 1.0f;
                g_f = 1.0f - s * 0.2f;
                b_f = 0.0f + s * 0.1f;
            } else {
                float s = (t - 0.75f) * 4;
                r_f = 1.0f - s * 0.4f;
                g_f = 0.8f - s * 0.6f;
                b_f = 0.1f + s * 0.4f;
            }
            r = clamp8(r_f * 255);
            g = clamp8(g_f * 255);
            b = clamp8(b_f * 255);
            break;
        }

        case ColormapType::HOT:
            // Black → Red → Yellow → White
            if (t < 0.375f) {
                r = clamp8(255 * t / 0.375f);
                g = 0; b = 0;
            } else if (t < 0.75f) {
                r = 255;
                g = clamp8(255 * (t - 0.375f) / 0.375f);
                b = 0;
            } else {
                r = 255;
                g = 255;
                b = clamp8(255 * (t - 0.75f) / 0.25f);
            }
            break;

        case ColormapType::COOL:
            // Cyan → Magenta
            r = clamp8(255 * t);
            g = clamp8(255 * (1 - t));
            b = 255;
            break;

        case ColormapType::BROWN_YELLOW: {
            // Dark brown → Yellow
            r = clamp8(lerp(101, 255, t));
            g = clamp8(lerp(67, 255, t));
            b = clamp8(lerp(33, 0, t));
            break;
        }

        case ColormapType::BLUE_RED:
            // Blue → White → Red (diverging)
            if (t < 0.5f) {
                float s = t * 2;
                r = clamp8(lerp(0, 255, s));
                g = clamp8(lerp(0, 255, s));
                b = 255;
            } else {
                float s = (t - 0.5f) * 2;
                r = 255;
                g = clamp8(lerp(255, 0, s));
                b = clamp8(lerp(255, 0, s));
            }
            break;

        case ColormapType::GREEN_RED:
            // Green → Yellow → Red
            if (t < 0.5f) {
                float s = t * 2;
                r = clamp8(255 * s);
                g = 255;
                b = 0;
            } else {
                float s = (t - 0.5f) * 2;
                r = 255;
                g = clamp8(255 * (1 - s));
                b = 0;
            }
            break;

        case ColormapType::GREYSCALE: {
            uint8_t v = clamp8(255 * t);
            r = v; g = v; b = v;
            break;
        }

        case ColormapType::BWR:
            // Blue → White → Red (diverging)
            if (t < 0.5f) {
                float s = t * 2;
                r = clamp8(255 * s);
                g = clamp8(255 * s);
                b = 255;
            } else {
                float s = (t - 0.5f) * 2;
                r = 255;
                g = clamp8(255 * (1 - s));
                b = clamp8(255 * (1 - s));
            }
            break;

        case ColormapType::RDYLGN:
            // Red → Yellow → Green (diverging)
            if (t < 0.5f) {
                float s = t * 2;
                r = 255;
                g = clamp8(255 * s);
                b = 0;
            } else {
                float s = (t - 0.5f) * 2;
                r = clamp8(255 * (1 - s));
                g = 255;
                b = 0;
            }
            break;

        default:
            break;
        }

        lut[i] = packRGB(r, g, b);
    }

    return lut;
}

}  // namespace ct
