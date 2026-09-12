#ifndef FAST_DETECTOR_H
#define FAST_DETECTOR_H

#include "ORBTypes.h"
#include <vector>
#include <cstdint>

namespace ORB_Standalone
{

/**
 * @brief FAST-9 Corner Detector (Features from Accelerated Segment Test)
 *
 * Implements the 9-arc contiguous Bresenham circle detector, corner response scoring,
 * and 3x3 non-maximum suppression.
 */
class FastDetector
{
public:
    FastDetector() = default;
    ~FastDetector() = default;

    /**
     * @brief Detects FAST-9 corners on a NativeImage.
     *
     * @param image Input native image buffer or ROI sub-image
     * @param keypoints Output list of detected keypoints
     * @param threshold Contrast threshold (e.g. 20 initial, 7 fallback)
     * @param nonmaxSuppression Whether to apply 3x3 non-maximum suppression
     */
    static void Detect(
        const NativeImage& image,
        std::vector<NativeKeyPoint>& keypoints,
        int threshold,
        bool nonmaxSuppression = true);

    /**
     * @brief Detects FAST-9 corners on raw image buffer pointer.
     */
    static void Detect(
        const uint8_t* data, int width, int height, int stride,
        std::vector<NativeKeyPoint>& keypoints,
        int threshold,
        bool nonmaxSuppression = true);

    /**
     * @brief Computes the dense corner response score map for a buffer.
     * Scores are strictly >= threshold for valid corners, or 0 otherwise.
     */
    static void ComputeScoreMap(
        const uint8_t* data, int width, int height, int stride,
        int16_t* scores, int threshold);
};

} // namespace ORB_Standalone

#endif // FAST_DETECTOR_H
