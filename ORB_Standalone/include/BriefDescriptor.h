#ifndef BRIEF_DESCRIPTOR_H
#define BRIEF_DESCRIPTOR_H

#include "ORBTypes.h"
#include <vector>
#include <cstdint>

namespace ORB_Standalone
{

/**
 * @brief Steered BRIEF-256 Descriptor Extractor
 *
 * Computes 256 binary intensity comparison tests on rotated bit_pattern_31_
 * over a pre-smoothed 31x31 circular neighborhood.
 */
class BriefDescriptor
{
public:
    static const int NUM_TESTS = 256;
    static const int DESCRIPTOR_BYTES = 32;
    static const int NUM_POINTS = 512;

    /**
     * @brief Computes a 256-bit steered BRIEF descriptor for a single keypoint.
     *
     * @param kpt Keypoint containing coordinate (x, y) and orientation angle in degrees
     * @param smoothedImage Pre-smoothed pyramid level image (including edge margin)
     * @param desc Destination buffer of 32 bytes
     */
    static void ComputeDescriptor(
        const NativeKeyPoint& kpt,
        const NativeImage& smoothedImage,
        uint8_t* desc);

    static void ComputeDescriptor(
        const NativeKeyPoint& kpt,
        const NativeImage& smoothedImage,
        NativeDescriptor& desc);

    /**
     * @brief Computes descriptors for a batch of keypoints on a smoothed octave image.
     */
    static void ComputeDescriptors(
        const std::vector<NativeKeyPoint>& keypoints,
        const NativeImage& smoothedImage,
        std::vector<NativeDescriptor>& descriptors);
};

} // namespace ORB_Standalone

#endif // BRIEF_DESCRIPTOR_H
