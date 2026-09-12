#ifndef BORDER_H
#define BORDER_H

#include "ORBTypes.h"
#include <cstdint>

namespace ORB_Standalone
{

/**
 * @brief Maps an out-of-bounds coordinate into range [0, length-1] using standard reflect-101 border symmetry.
 * Example for length 10: -1 -> 1, -2 -> 2, 10 -> 8, 11 -> 7.
 */
inline int BorderReflect101(int p, int length)
{
    if (length <= 1) return 0;
    while (p < 0 || p >= length)
    {
        if (p < 0)
            p = -p;
        else
            p = 2 * length - 2 - p;
    }
    return p;
}

/**
 * @brief Adds reflected border padding (BORDER_REFLECT_101) to a grayscale image.
 *
 * @param src Pointer to source image top-left pixel
 * @param srcWidth Width of source image
 * @param srcHeight Height of source image
 * @param srcStride Row stride of source image
 * @param dst Pointer to destination buffer (must have size (srcWidth + left + right) * dstStride)
 * @param dstStride Row stride of destination buffer
 * @param borderTop Number of rows to pad on top
 * @param borderBottom Number of rows to pad on bottom
 * @param borderLeft Number of columns to pad on left
 * @param borderRight Number of columns to pad on right
 */
void CopyMakeBorderReflect101(
    const uint8_t* src, int srcWidth, int srcHeight, int srcStride,
    uint8_t* dst, int dstStride,
    int borderTop, int borderBottom, int borderLeft, int borderRight);

/**
 * @brief Overload operating directly on NativeImage structures.
 */
void CopyMakeBorderReflect101(
    const NativeImage& src, NativeImage& dst,
    int borderTop, int borderBottom, int borderLeft, int borderRight);

} // namespace ORB_Standalone

#endif // BORDER_H
