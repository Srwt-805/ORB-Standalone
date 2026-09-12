#include "BriefDescriptor.h"
#include "OrbPattern.h"
#include <cmath>
#include <cstring>
#include <algorithm>

#if defined(__SSE2__) || defined(__x86_64__) || defined(_M_X64)
#include <emmintrin.h>
inline int NativeRound(float value)
{
    return _mm_cvtss_si32(_mm_set_ss(value));
}
#else
inline int NativeRound(float value)
{
    return static_cast<int>(std::lrint(value));
}
#endif

namespace ORB_Standalone
{

static const float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

void BriefDescriptor::ComputeDescriptor(
    const NativeKeyPoint& kpt,
    const NativeImage& smoothedImage,
    uint8_t* desc)
{
    float angle = kpt.angle * DEG_TO_RAD;
    float a = std::cos(angle);
    float b = std::sin(angle);

    const int ix = NativeRound(kpt.x);
    const int iy = NativeRound(kpt.y);

    const uint8_t* center = smoothedImage.data + iy * smoothedImage.stride + ix;
    const int step = smoothedImage.stride;

    const int* pattern = bit_pattern_31_;

    #define GET_VALUE(idx) \
        center[NativeRound(pattern[(idx)*2] * b + pattern[(idx)*2 + 1] * a) * step + \
               NativeRound(pattern[(idx)*2] * a - pattern[(idx)*2 + 1] * b)]

    for (int i = 0; i < 32; ++i, pattern += 32)
    {
        int t0, t1, val;
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = (t0 < t1);
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = static_cast<uint8_t>(val);
    }

    #undef GET_VALUE
}

void BriefDescriptor::ComputeDescriptor(
    const NativeKeyPoint& kpt,
    const NativeImage& smoothedImage,
    NativeDescriptor& desc)
{
    ComputeDescriptor(kpt, smoothedImage, desc.ptr());
}

void BriefDescriptor::ComputeDescriptors(
    const std::vector<NativeKeyPoint>& keypoints,
    const NativeImage& smoothedImage,
    std::vector<NativeDescriptor>& descriptors)
{
    descriptors.resize(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i)
    {
        ComputeDescriptor(keypoints[i], smoothedImage, descriptors[i].ptr());
    }
}

} // namespace ORB_Standalone
