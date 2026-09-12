#include "FastDetector.h"
#include <algorithm>
#include <cstring>
#include <vector>

namespace ORB_Standalone
{

static const int CIRCLE_DX[16] = {
     0,  1,  2,  3,  3,  3,  2,  1,
     0, -1, -2, -3, -3, -3, -2, -1
};

static const int CIRCLE_DY[16] = {
    -3, -3, -2, -1,  0,  1,  2,  3,
     3,  3,  2,  1,  0, -1, -2, -3
};

void FastDetector::ComputeScoreMap(
    const uint8_t* data, int width, int height, int stride,
    int16_t* scores, int threshold)
{
    if (!data || !scores || width < 7 || height < 7)
        return;

    std::memset(scores, 0, width * height * sizeof(int16_t));

    // Precompute 1D byte offsets for 16 Bresenham circle positions
    int circleOfs[16];
    for (int k = 0; k < 16; ++k)
    {
        circleOfs[k] = CIRCLE_DY[k] * stride + CIRCLE_DX[k];
    }

    const int ofs0 = circleOfs[0];
    const int ofs4 = circleOfs[4];
    const int ofs8 = circleOfs[8];
    const int ofs12 = circleOfs[12];

    for (int y = 3; y < height - 3; ++y)
    {
        const uint8_t* ptr = data + y * stride;
        int16_t* scoreRow = scores + y * width;

        for (int x = 3; x < width - 3; ++x)
        {
            const uint8_t* center = ptr + x;
            const int c = static_cast<int>(*center);

            // Early rejection test: an arc of 9 must contain at least one adjacent pair of cardinal points
            const int p0 = static_cast<int>(center[ofs0]);
            const int p4 = static_cast<int>(center[ofs4]);
            const int p8 = static_cast<int>(center[ofs8]);
            const int p12 = static_cast<int>(center[ofs12]);

            const bool b0 = (p0 - c >= threshold);
            const bool b4 = (p4 - c >= threshold);
            const bool b8 = (p8 - c >= threshold);
            const bool b12 = (p12 - c >= threshold);
            const bool has_b = (b0 && b4) || (b4 && b8) || (b8 && b12) || (b12 && b0);

            const bool d0 = (c - p0 >= threshold);
            const bool d4 = (c - p4 >= threshold);
            const bool d8 = (c - p8 >= threshold);
            const bool d12 = (c - p12 >= threshold);
            const bool has_d = (d0 && d4) || (d4 && d8) || (d8 && d12) || (d12 && d0);

            if (!has_b && !has_d)
                continue;

            // Load 16 circle pixels and compute signed differences
            int diffs[16];
            for (int k = 0; k < 16; ++k)
            {
                diffs[k] = static_cast<int>(center[circleOfs[k]]) - c;
            }

            // Find maximum threshold across all 16 contiguous arcs of length 9
            int max_b = -9999;
            int max_d = -9999;

            for (int i = 0; i < 16; ++i)
            {
                int min_b = diffs[i];
                int min_d = -diffs[i];
                for (int j = 1; j < 9; ++j)
                {
                    int d = diffs[(i + j) & 15];
                    if (d < min_b) min_b = d;
                    if (-d < min_d) min_d = -d;
                }
                if (min_b > max_b) max_b = min_b;
                if (min_d > max_d) max_d = min_d;
            }

            int score = std::max(max_b, max_d) - 1;
            if (score >= threshold)
            {
                scoreRow[x] = static_cast<int16_t>(score);
            }
        }
    }
}

void FastDetector::Detect(
    const uint8_t* data, int width, int height, int stride,
    std::vector<NativeKeyPoint>& keypoints,
    int threshold,
    bool nonmaxSuppression)
{
    keypoints.clear();
    if (!data || width < 7 || height < 7)
        return;

    std::vector<int16_t> scores(width * height, 0);
    ComputeScoreMap(data, width, height, stride, scores.data(), threshold);

    if (nonmaxSuppression)
    {
        for (int y = 3; y < height - 3; ++y)
        {
            const int16_t* prevRow = scores.data() + (y - 1) * width;
            const int16_t* currRow = scores.data() + y * width;
            const int16_t* nextRow = scores.data() + (y + 1) * width;

            for (int x = 3; x < width - 3; ++x)
            {
                int16_t s = currRow[x];
                if (s >= threshold)
                {
                    if (s > currRow[x - 1] && s > currRow[x + 1] &&
                        s > prevRow[x - 1] && s > prevRow[x] && s > prevRow[x + 1] &&
                        s > nextRow[x - 1] && s > nextRow[x] && s > nextRow[x + 1])
                    {
                        keypoints.emplace_back(
                            static_cast<float>(x),
                            static_cast<float>(y),
                            7.0f,
                            -1.0f,
                            static_cast<float>(s),
                            0);
                    }
                }
            }
        }
    }
    else
    {
        for (int y = 3; y < height - 3; ++y)
        {
            const int16_t* currRow = scores.data() + y * width;
            for (int x = 3; x < width - 3; ++x)
            {
                int16_t s = currRow[x];
                if (s >= threshold)
                {
                    keypoints.emplace_back(
                        static_cast<float>(x),
                        static_cast<float>(y),
                        7.0f,
                        -1.0f,
                        static_cast<float>(s),
                        0);
                }
            }
        }
    }
}

void FastDetector::Detect(
    const NativeImage& image,
    std::vector<NativeKeyPoint>& keypoints,
    int threshold,
    bool nonmaxSuppression)
{
    Detect(image.data, image.width, image.height, image.stride,
           keypoints, threshold, nonmaxSuppression);
}

} // namespace ORB_Standalone
