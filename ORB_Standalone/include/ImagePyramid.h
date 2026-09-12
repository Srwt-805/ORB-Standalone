#ifndef IMAGE_PYRAMID_H
#define IMAGE_PYRAMID_H

#include "ORBTypes.h"
#include "Border.h"
#include <vector>
#include <cmath>

namespace ORB_Standalone
{

/**
 * @brief Multi-scale Image Pyramid for ORB feature extraction.
 *
 * Implements the standard scale-space pyramid for ORB:
 *  - Scale decimation ratio (default: 1.2)
 *  - Resampling: Fixed-point bilinear interpolation
 *  - Border margin: EDGE_THRESHOLD = 19 with reflect-101 border padding
 *  - Contiguous buffers with zero-copy inner region sub-images
 */
class ImagePyramid
{
public:
    explicit ImagePyramid(int nLevels = 8, float scaleFactor = 1.2f, int edgeThreshold = 19);
    ~ImagePyramid() = default;

    /**
     * @brief Builds all pyramid octaves from an input grayscale image.
     */
    void Compute(const NativeImage& image);

    // Accessors
    int GetNumLevels() const { return mnLevels; }
    float GetScaleFactor() const { return mScaleFactor; }
    int GetEdgeThreshold() const { return mEdgeThreshold; }

    const NativePyramidLevel& GetLevel(int level) const { return mvLevels[level]; }
    const std::vector<NativePyramidLevel>& GetLevels() const { return mvLevels; }

    const std::vector<float>& GetScaleFactors() const { return mvScaleFactor; }
    const std::vector<float>& GetInverseScaleFactors() const { return mvInvScaleFactor; }
    const std::vector<float>& GetScaleSigmaSquares() const { return mvLevelSigma2; }
    const std::vector<float>& GetInverseScaleSigmaSquares() const { return mvInvLevelSigma2; }

    /**
     * @brief Native bilinear image resizing from src to dst dimensions.
     * Fixed-point 11-bit precision with coordinate mapping:
     *   sx = (dx + 0.5) * (src_w / dst_w) - 0.5
     *   sy = (dy + 0.5) * (src_h / dst_h) - 0.5
     */
    static void ResizeBilinear(
        const uint8_t* src, int srcWidth, int srcHeight, int srcStride,
        uint8_t* dst, int dstWidth, int dstHeight, int dstStride);

    static void ResizeBilinear(const NativeImage& src, NativeImage& dst, int dstWidth, int dstHeight);

private:
    void InitializeScales();

    int mnLevels;
    float mScaleFactor;
    int mEdgeThreshold;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    std::vector<NativePyramidLevel> mvLevels;
};

} // namespace ORB_Standalone

#endif // IMAGE_PYRAMID_H
