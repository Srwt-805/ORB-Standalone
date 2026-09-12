#ifndef GAUSSIAN_FILTER_H
#define GAUSSIAN_FILTER_H

#include "ORBTypes.h"
#include <cstdint>

namespace ORB_Standalone
{

/**
 * @brief Separable Gaussian Blur filter (7x7, sigma=2.0, reflect-101 border)
 *
 * Implements efficient 2-pass separable convolution:
 * Pass 1: Horizontal 1D filter (7 taps, symmetric, 16-bit fixed point)
 * Pass 2: Vertical 1D filter (7 taps, symmetric, 16-bit fixed point)
 */
class GaussianFilter
{
public:
    // Fixed-point 16-bit normalized symmetric kernel weights (sum = 65536)
    static const int K0 = 4598;   // k[-3] = k[3]
    static const int K1 = 8590;   // k[-2] = k[2]
    static const int K2 = 12499;  // k[-1] = k[1]
    static const int K3 = 14162;  // k[0] (center)

    /**
     * @brief Filters an image with 7x7 Gaussian blur (sigma=2.0, BORDER_REFLECT_101).
     *
     * @param src Pointer to input image pixels
     * @param srcWidth Width of image
     * @param srcHeight Height of image
     * @param srcStride Row stride of input
     * @param dst Pointer to output buffer
     * @param dstStride Row stride of output
     */
    static void Filter7x7(
        const uint8_t* src, int srcWidth, int srcHeight, int srcStride,
        uint8_t* dst, int dstStride);

    static void Filter7x7(const NativeImage& src, NativeImage& dst);
};

} // namespace ORB_Standalone

#endif // GAUSSIAN_FILTER_H
