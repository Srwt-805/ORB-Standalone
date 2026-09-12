#include "GaussianFilter.h"
#include "Border.h"
#include <vector>
#include <algorithm>
#include <cstring>

namespace ORB_Standalone
{

void GaussianFilter::Filter7x7(
    const uint8_t* src, int srcWidth, int srcHeight, int srcStride,
    uint8_t* dst, int dstStride)
{
    if (!src || !dst || srcWidth <= 0 || srcHeight <= 0)
        return;

    // Temporary intermediate buffer for horizontal pass (int32 or int16)
    // intermediate buffer size: srcHeight rows x srcWidth columns
    std::vector<int32_t> intermediate(srcHeight * srcWidth);

    // Pass 1: Horizontal 1D filtering
    for (int y = 0; y < srcHeight; ++y)
    {
        const uint8_t* srcRow = src + y * srcStride;
        int32_t* dstRow = intermediate.data() + y * srcWidth;

        for (int x = 0; x < srcWidth; ++x)
        {
            // Reflect boundary for x +/- 1, 2, 3
            int x_m3 = (x >= 3) ? (x - 3) : BorderReflect101(x - 3, srcWidth);
            int x_m2 = (x >= 2) ? (x - 2) : BorderReflect101(x - 2, srcWidth);
            int x_m1 = (x >= 1) ? (x - 1) : BorderReflect101(x - 1, srcWidth);
            int x_p1 = (x + 1 < srcWidth) ? (x + 1) : BorderReflect101(x + 1, srcWidth);
            int x_p2 = (x + 2 < srcWidth) ? (x + 2) : BorderReflect101(x + 2, srcWidth);
            int x_p3 = (x + 3 < srcWidth) ? (x + 3) : BorderReflect101(x + 3, srcWidth);

            int val = (K0 * (static_cast<int>(srcRow[x_m3]) + static_cast<int>(srcRow[x_p3])) +
                       K1 * (static_cast<int>(srcRow[x_m2]) + static_cast<int>(srcRow[x_p2])) +
                       K2 * (static_cast<int>(srcRow[x_m1]) + static_cast<int>(srcRow[x_p1])) +
                       K3 * static_cast<int>(srcRow[x]) + 32768) >> 16;

            dstRow[x] = val;
        }
    }

    // Pass 2: Vertical 1D filtering on intermediate buffer
    for (int y = 0; y < srcHeight; ++y)
    {
        int y_m3 = (y >= 3) ? (y - 3) : BorderReflect101(y - 3, srcHeight);
        int y_m2 = (y >= 2) ? (y - 2) : BorderReflect101(y - 2, srcHeight);
        int y_m1 = (y >= 1) ? (y - 1) : BorderReflect101(y - 1, srcHeight);
        int y_p1 = (y + 1 < srcHeight) ? (y + 1) : BorderReflect101(y + 1, srcHeight);
        int y_p2 = (y + 2 < srcHeight) ? (y + 2) : BorderReflect101(y + 2, srcHeight);
        int y_p3 = (y + 3 < srcHeight) ? (y + 3) : BorderReflect101(y + 3, srcHeight);

        const int32_t* row_m3 = intermediate.data() + y_m3 * srcWidth;
        const int32_t* row_m2 = intermediate.data() + y_m2 * srcWidth;
        const int32_t* row_m1 = intermediate.data() + y_m1 * srcWidth;
        const int32_t* row_c  = intermediate.data() + y * srcWidth;
        const int32_t* row_p1 = intermediate.data() + y_p1 * srcWidth;
        const int32_t* row_p2 = intermediate.data() + y_p2 * srcWidth;
        const int32_t* row_p3 = intermediate.data() + y_p3 * srcWidth;

        uint8_t* outRow = dst + y * dstStride;

        for (int x = 0; x < srcWidth; ++x)
        {
            int val = (K0 * (row_m3[x] + row_p3[x]) +
                       K1 * (row_m2[x] + row_p2[x]) +
                       K2 * (row_m1[x] + row_p1[x]) +
                       K3 * row_c[x] + 32768) >> 16;

            outRow[x] = static_cast<uint8_t>(std::max(0, std::min(255, val)));
        }
    }
}

void GaussianFilter::Filter7x7(const NativeImage& src, NativeImage& dst)
{
    if (dst.width != src.width || dst.height != src.height || dst.empty())
    {
        dst.Allocate(src.width, src.height);
    }

    Filter7x7(src.data, src.width, src.height, src.stride,
              dst.data, dst.stride);
}

} // namespace ORB_Standalone
