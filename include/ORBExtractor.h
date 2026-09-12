#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H

#include <vector>
#include <string>
#include <memory>

#include "ORBTypes.h"
#include "Border.h"
#include "ImagePyramid.h"
#include "FastDetector.h"
#include "FeatureDistribution.h"
#include "Orientation.h"
#include "Atan2.h"
#include "GaussianFilter.h"
#include "BriefDescriptor.h"

namespace ORB_Standalone
{

/**
 * @brief Standalone ORB Feature Extractor (Oriented FAST and Rotated BRIEF)
 *
 * Implements the standard 7-stage ORB extraction pipeline using pure native C++:
 *  1. Multi-scale scale-space pyramid construction (ImagePyramid)
 *  2. Reflect-101 border margin padding (Border)
 *  3. Cell-based FAST-9 corner detection with adaptive fallback thresholding (FastDetector)
 *  4. Deterministic quadtree spatial distribution (FeatureDistribution)
 *  5. Circular patch intensity centroid orientation estimation (Orientation, Atan2)
 *  6. Separable Gaussian pre-filtering on octave images (GaussianFilter)
 *  7. Steered BRIEF-256 binary descriptor extraction (BriefDescriptor)
 */
class ORBExtractor
{
public:
    explicit ORBExtractor(const ORBSettings& settings = ORBSettings());
    ORBExtractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);
    ~ORBExtractor() = default;

    /**
     * @brief Primary feature extraction interface
     *
     * @param image Input contiguous grayscale native image buffer
     * @param keypoints Output list of native keypoints (x, y, octave, scale, response, angle, size)
     * @param descriptors Output list of 256-bit (32-byte) binary descriptors
     * @param report Optional detailed per-stage timing and metrics report
     */
    void Extract(const NativeImage& image,
                 std::vector<NativeKeyPoint>& keypoints,
                 std::vector<NativeDescriptor>& descriptors,
                 ExtractionReport* report = nullptr);

    /**
     * @brief Convenience extraction interface returning unified ORBFeature structures
     *
     * @param image Input contiguous grayscale native image buffer
     * @param features Output list of unified features combining keypoint metadata and descriptor
     * @param report Optional detailed per-stage timing and metrics report
     */
    void Extract(const NativeImage& image,
                 std::vector<ORBFeature>& features,
                 ExtractionReport* report = nullptr);

    // Configuration & Reporting
    const ORBSettings& GetSettings() const { return mSettings; }
    const ExtractionReport& GetLastReport() const { return mLastReport; }

    // Pyramid Scale Properties
    int GetLevels() const { return mSettings.nLevels; }
    float GetScaleFactor() const { return mSettings.scaleFactor; }
    const std::vector<float>& GetScaleFactors() const { return mvScaleFactor; }
    const std::vector<float>& GetInverseScaleFactors() const { return mvInvScaleFactor; }
    const std::vector<float>& GetScaleSigmaSquares() const { return mvLevelSigma2; }
    const std::vector<float>& GetInverseScaleSigmaSquares() const { return mvInvLevelSigma2; }

    const ImagePyramid& GetPyramid() const { return mNativePyramid; }

private:
    void InitializeTables();

    ORBSettings mSettings;
    ExtractionReport mLastReport;

    std::vector<int> mnFeaturesPerLevel;
    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    ImagePyramid mNativePyramid;
};

} // namespace ORB_Standalone

#endif // ORB_EXTRACTOR_H
