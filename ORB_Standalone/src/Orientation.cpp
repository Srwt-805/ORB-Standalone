#include "Orientation.h"
#include "Atan2.h"
#include <cmath>
#include <algorithm>

namespace ORB_Standalone
{

void Orientation::InitializeUmaxTable(int* umax, int halfPatch)
{
    int v, v0;
    int vmax = static_cast<int>(std::floor(halfPatch * std::sqrt(2.0f) / 2.0f + 1.0f));
    int vmin = static_cast<int>(std::ceil(halfPatch * std::sqrt(2.0f) / 2.0f));
    const double hp2 = static_cast<double>(halfPatch * halfPatch);

    for (v = 0; v <= vmax; ++v)
    {
        umax[v] = static_cast<int>(std::round(std::sqrt(hp2 - v * v)));
    }

    for (v = halfPatch, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}

float Orientation::IC_Angle(const NativeImage& image, float ptX, float ptY, const int* u_max)
{
    int m_01 = 0, m_10 = 0;
    const int halfPatch = HALF_PATCH_SIZE;

    const int ix = static_cast<int>(std::round(ptX));
    const int iy = static_cast<int>(std::round(ptY));

    const uint8_t* center = image.data + iy * image.stride + ix;
    const int step = image.stride;

    // Center horizontal line v = 0
    for (int u = -halfPatch; u <= halfPatch; ++u)
    {
        m_10 += u * center[u];
    }

    // Go line by line symmetrically in circular patch for v in [1, halfPatch]
    for (int v = 1; v <= halfPatch; ++v)
    {
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v * step];
            int val_minus = center[u - v * step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return FastAtan2(static_cast<float>(m_01), static_cast<float>(m_10));
}

void Orientation::ComputeOrientation(
    const NativeImage& image,
    std::vector<NativeKeyPoint>& keypoints,
    const int* umax)
{
    for (auto& kp : keypoints)
    {
        kp.angle = IC_Angle(image, kp.x, kp.y, umax);
    }
}

} // namespace ORB_Standalone
