#include "Border.h"
#include <cstring>
#include <algorithm>

namespace ORB_Standalone
{

void CopyMakeBorderReflect101(
    const uint8_t* src, int srcWidth, int srcHeight, int srcStride,
    uint8_t* dst, int dstStride,
    int borderTop, int borderBottom, int borderLeft, int borderRight)
{
    if (!src || !dst || srcWidth <= 0 || srcHeight <= 0)
        return;

    const int dstWidth = srcWidth + borderLeft + borderRight;

    // Step 1: Copy interior source rows and pad left/right columns for each active row
    for (int y = 0; y < srcHeight; ++y)
    {
        const uint8_t* srcRow = src + y * srcStride;
        uint8_t* dstRow = dst + (y + borderTop) * dstStride;

        // Copy active interior pixels
        std::memcpy(dstRow + borderLeft, srcRow, srcWidth);

        // Pad left columns
        for (int x = 0; x < borderLeft; ++x)
        {
            int sx = BorderReflect101(x - borderLeft, srcWidth);
            dstRow[x] = srcRow[sx];
        }

        // Pad right columns
        for (int x = 0; x < borderRight; ++x)
        {
            int sx = BorderReflect101(srcWidth + x, srcWidth);
            dstRow[borderLeft + srcWidth + x] = srcRow[sx];
        }
    }

    // Step 2: Pad top rows by copying corresponding reflected rows from dst
    for (int y = 0; y < borderTop; ++y)
    {
        int sy = BorderReflect101(y - borderTop, srcHeight) + borderTop;
        std::memcpy(dst + y * dstStride, dst + sy * dstStride, dstWidth);
    }

    // Step 3: Pad bottom rows by copying corresponding reflected rows from dst
    for (int y = 0; y < borderBottom; ++y)
    {
        int sy = BorderReflect101(srcHeight + y, srcHeight) + borderTop;
        std::memcpy(dst + (borderTop + srcHeight + y) * dstStride, dst + sy * dstStride, dstWidth);
    }
}

void CopyMakeBorderReflect101(
    const NativeImage& src, NativeImage& dst,
    int borderTop, int borderBottom, int borderLeft, int borderRight)
{
    const int dstWidth = src.width + borderLeft + borderRight;
    const int dstHeight = src.height + borderTop + borderBottom;

    if (dst.width != dstWidth || dst.height != dstHeight || dst.empty())
    {
        dst.Allocate(dstWidth, dstHeight);
    }

    CopyMakeBorderReflect101(
        src.data, src.width, src.height, src.stride,
        dst.data, dst.stride,
        borderTop, borderBottom, borderLeft, borderRight);
}

} // namespace ORB_Standalone
