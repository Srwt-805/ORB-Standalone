#include "ImagePyramid.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace ORB_Standalone
{

ImagePyramid::ImagePyramid(int nLevels, float scaleFactor, int edgeThreshold)
    : mnLevels(nLevels), mScaleFactor(scaleFactor), mEdgeThreshold(edgeThreshold)
{
    InitializeScales();
}

void ImagePyramid::InitializeScales()
{
    mvScaleFactor.resize(mnLevels);
    mvLevelSigma2.resize(mnLevels);
    mvScaleFactor[0] = 1.0f;
    mvLevelSigma2[0] = 1.0f;
    for (int i = 1; i < mnLevels; ++i)
    {
        mvScaleFactor[i] = mvScaleFactor[i - 1] * mScaleFactor;
        mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(mnLevels);
    mvInvLevelSigma2.resize(mnLevels);
    for (int i = 0; i < mnLevels; ++i)
    {
        mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
        mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
    }

    mvLevels.resize(mnLevels);
}

void ImagePyramid::ResizeBilinear(
    const uint8_t* src, int srcWidth, int srcHeight, int srcStride,
    uint8_t* dst, int dstWidth, int dstHeight, int dstStride)
{
    if (!src || !dst || srcWidth <= 0 || srcHeight <= 0 || dstWidth <= 0 || dstHeight <= 0)
        return;

    const double scale_x = static_cast<double>(srcWidth) / static_cast<double>(dstWidth);
    const double scale_y = static_cast<double>(srcHeight) / static_cast<double>(dstHeight);

    // Precompute horizontal mapping and weights
    std::vector<int> x_ofs(dstWidth);
    std::vector<short> x_w(dstWidth * 2);

    for (int x = 0; x < dstWidth; ++x)
    {
        float fx = static_cast<float>((x + 0.5) * scale_x - 0.5);
        int sx = static_cast<int>(std::floor(fx));
        fx -= static_cast<float>(sx);

        if (sx < 0)
        {
            fx = 0.0f;
            sx = 0;
        }
        if (sx >= srcWidth - 1)
        {
            fx = 0.0f;
            sx = srcWidth - 1;
        }

        x_ofs[x] = sx;
        short iw0 = static_cast<short>(std::round((1.0f - fx) * 2048.0f));
        x_w[x * 2] = iw0;
        x_w[x * 2 + 1] = static_cast<short>(2048 - iw0);
    }

    // Precompute vertical mapping and weights
    std::vector<int> y_ofs(dstHeight);
    std::vector<short> y_w(dstHeight * 2);

    for (int y = 0; y < dstHeight; ++y)
    {
        float fy = static_cast<float>((y + 0.5) * scale_y - 0.5);
        int sy = static_cast<int>(std::floor(fy));
        fy -= static_cast<float>(sy);

        if (sy < 0)
        {
            fy = 0.0f;
            sy = 0;
        }
        if (sy >= srcHeight - 1)
        {
            fy = 0.0f;
            sy = srcHeight - 1;
        }

        y_ofs[y] = sy;
        short iw0 = static_cast<short>(std::round((1.0f - fy) * 2048.0f));
        y_w[y * 2] = iw0;
        y_w[y * 2 + 1] = static_cast<short>(2048 - iw0);
    }

    // Bilinear interpolation with fixed-point arithmetic
    for (int y = 0; y < dstHeight; ++y)
    {
        int sy0 = y_ofs[y];
        int sy1 = std::min(sy0 + 1, srcHeight - 1);
        short yw0 = y_w[y * 2];
        short yw1 = y_w[y * 2 + 1];

        const uint8_t* row0 = src + sy0 * srcStride;
        const uint8_t* row1 = src + sy1 * srcStride;
        uint8_t* dstRow = dst + y * dstStride;

        for (int x = 0; x < dstWidth; ++x)
        {
            int sx0 = x_ofs[x];
            int sx1 = std::min(sx0 + 1, srcWidth - 1);
            short xw0 = x_w[x * 2];
            short xw1 = x_w[x * 2 + 1];

            int S0 = static_cast<int>(row0[sx0]) * xw0 + static_cast<int>(row0[sx1]) * xw1;
            int S1 = static_cast<int>(row1[sx0]) * xw0 + static_cast<int>(row1[sx1]) * xw1;

            int t0 = ((S0 >> 4) * static_cast<int>(yw0)) >> 16;
            int t1 = ((S1 >> 4) * static_cast<int>(yw1)) >> 16;
            int val = (t0 + t1 + 2) >> 2;

            dstRow[x] = static_cast<uint8_t>(std::max(0, std::min(255, val)));
        }
    }
}

void ImagePyramid::ResizeBilinear(const NativeImage& src, NativeImage& dst, int dstWidth, int dstHeight)
{
    if (dst.width != dstWidth || dst.height != dstHeight || dst.empty())
    {
        dst.Allocate(dstWidth, dstHeight);
    }

    ResizeBilinear(src.data, src.width, src.height, src.stride,
                   dst.data, dst.width, dst.height, dst.stride);
}

void ImagePyramid::Compute(const NativeImage& image)
{
    const int edgeThresh = mEdgeThreshold;

    for (int level = 0; level < mnLevels; ++level)
    {
        float scale = mvInvScaleFactor[level];
        int lvlWidth = static_cast<int>(std::round(static_cast<float>(image.width) * scale));
        int lvlHeight = static_cast<int>(std::round(static_cast<float>(image.height) * scale));

        NativePyramidLevel& lvl = mvLevels[level];
        lvl.width = lvlWidth;
        lvl.height = lvlHeight;
        lvl.scale = mvScaleFactor[level];
        lvl.invScale = mvInvScaleFactor[level];
        lvl.sigma2 = mvLevelSigma2[level];
        lvl.invSigma2 = mvInvLevelSigma2[level];

        const int fullWidth = lvlWidth + edgeThresh * 2;
        const int fullHeight = lvlHeight + edgeThresh * 2;

        // Ensure full padded buffer is allocated
        if (lvl.fullImage.width != fullWidth || lvl.fullImage.height != fullHeight || lvl.fullImage.empty())
        {
            lvl.fullImage.Allocate(fullWidth, fullHeight);
        }

        // innerImage is a sub-image pointing to the active unpadded ROI
        lvl.innerImage = lvl.fullImage.SubImage(edgeThresh, edgeThresh, lvlWidth, lvlHeight);

        if (level == 0)
        {
            // Level 0: Pad original image directly into fullImage
            CopyMakeBorderReflect101(
                image.data, image.width, image.height, image.stride,
                lvl.fullImage.data, lvl.fullImage.stride,
                edgeThresh, edgeThresh, edgeThresh, edgeThresh);
        }
        else
        {
            // Downscale from previous octave's innerImage to current octave's innerImage
            ResizeBilinear(
                mvLevels[level - 1].innerImage.data,
                mvLevels[level - 1].innerImage.width,
                mvLevels[level - 1].innerImage.height,
                mvLevels[level - 1].innerImage.stride,
                lvl.innerImage.data,
                lvlWidth,
                lvlHeight,
                lvl.innerImage.stride);

            // Reflect border margin around current innerImage into fullImage
            // Note: innerImage is already placed at offset (edgeThresh, edgeThresh) inside fullImage,
            // so we can directly copy border into fullImage
            CopyMakeBorderReflect101(
                lvl.innerImage.data, lvl.innerImage.width, lvl.innerImage.height, lvl.innerImage.stride,
                lvl.fullImage.data, lvl.fullImage.stride,
                edgeThresh, edgeThresh, edgeThresh, edgeThresh);
        }
    }
}

} // namespace ORB_Standalone
