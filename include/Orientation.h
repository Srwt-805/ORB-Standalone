#ifndef ORIENTATION_H
#define ORIENTATION_H

#include "ORBTypes.h"
#include <vector>

namespace ORB_Standalone
{

/**
 * @brief Intensity Centroid Orientation for ORB features (Rosin's algorithm)
 *
 * Computes the orientation angle from patch moments m01, m10 over a circular patch of radius 15.
 */
class Orientation
{
public:
    static const int HALF_PATCH_SIZE = 15;
    static const int PATCH_SIZE = 31;

    /**
     * @brief Computes intensity-centroid orientation for a point (ptX, ptY) on a NativeImage.
     */
    static float IC_Angle(const NativeImage& image, float ptX, float ptY, const int* umax);

    /**
     * @brief Computes orientation for a batch of keypoints on a NativeImage.
     */
    static void ComputeOrientation(
        const NativeImage& image,
        std::vector<NativeKeyPoint>& keypoints,
        const int* umax);

    /**
     * @brief Precomputes the 8-way symmetric umax table.
     */
    static void InitializeUmaxTable(int* umax, int halfPatch = 15);
};

} // namespace ORB_Standalone

#endif // ORIENTATION_H
