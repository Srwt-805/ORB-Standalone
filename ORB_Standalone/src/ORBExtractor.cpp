/**
 * @file ORBExtractor.cpp
 * @brief Standalone Implementation of the ORB Feature Extractor
 *
 * All algorithmic pipeline stages (Pyramid, Border, FAST, Distribution, Orientation, Blur, BRIEF)
 * are implemented in pure native C++ with zero OpenCV dependencies.
 */

#include "ORBExtractor.h"

#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstring>

namespace ORB_Standalone
{

ORBExtractor::ORBExtractor(const ORBSettings& settings)
    : mSettings(settings), mNativePyramid(settings.nLevels, settings.scaleFactor, settings.edgeThreshold)
{
    InitializeTables();
}

ORBExtractor::ORBExtractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST)
    : mNativePyramid(nlevels, scaleFactor, 19)
{
    mSettings.nFeatures = nfeatures;
    mSettings.scaleFactor = scaleFactor;
    mSettings.nLevels = nlevels;
    mSettings.iniThFAST = iniThFAST;
    mSettings.minThFAST = minThFAST;
    mSettings.edgeThreshold = 19;
    mSettings.patchSize = 31;
    mSettings.halfPatchSize = 15;
    mSettings.fastCellSize = 30;
    InitializeTables();
}

void ORBExtractor::InitializeTables()
{
    const int nlevels = mSettings.nLevels;
    const float scaleFactor = mSettings.scaleFactor;
    const int nfeatures = mSettings.nFeatures;

    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0] = 1.0f;
    mvLevelSigma2[0] = 1.0f;
    for (int i = 1; i < nlevels; ++i)
    {
        mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;
        mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for (int i = 0; i < nlevels; ++i)
    {
        mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
        mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
    }

    // Feature budget allocation across pyramid levels (exponential decimation)
    mnFeaturesPerLevel.resize(nlevels);
    float factor = 1.0f / scaleFactor;
    float nDesiredFeaturesPerScale = nfeatures * (1.0f - factor) / (1.0f - static_cast<float>(std::pow(factor, nlevels)));

    int sumFeatures = 0;
    for (int level = 0; level < nlevels - 1; ++level)
    {
        mnFeaturesPerLevel[level] = static_cast<int>(std::round(nDesiredFeaturesPerScale));
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[nlevels - 1] = std::max(nfeatures - sumFeatures, 0);

    // Initialize circular patch row bounds (umax) for intensity centroid orientation
    const int halfPatch = mSettings.halfPatchSize;
    umax.resize(halfPatch + 1);
    Orientation::InitializeUmaxTable(umax.data(), halfPatch);
}

void ORBExtractor::Extract(const NativeImage& image,
                           std::vector<NativeKeyPoint>& keypoints,
                           std::vector<NativeDescriptor>& descriptors,
                           ExtractionReport* report)
{
    keypoints.clear();
    descriptors.clear();
    if (image.empty()) return;

    mLastReport = ExtractionReport();
    mLastReport.inputWidth = image.width;
    mLastReport.inputHeight = image.height;
    mLastReport.nLevels = mSettings.nLevels;
    mLastReport.levelStats.resize(mSettings.nLevels);

    auto tTotalStart = std::chrono::high_resolution_clock::now();

    // Stage 1: Scale space pyramid construction
    auto tPyrStart = std::chrono::high_resolution_clock::now();
    mNativePyramid.Compute(image);
    auto tPyrEnd = std::chrono::high_resolution_clock::now();
    mLastReport.timePyramidMs = std::chrono::duration<double, std::milli>(tPyrEnd - tPyrStart).count();

    // Stage 2: Cell-based FAST-9 detection & quadtree distribution
    std::vector<std::vector<NativeKeyPoint>> allKeypoints(mSettings.nLevels);
    const float W = static_cast<float>(mSettings.fastCellSize);
    const int edgeThresh = mSettings.edgeThreshold;

    double totalFastMs = 0.0;
    double totalDistMs = 0.0;

    for (int level = 0; level < mSettings.nLevels; ++level)
    {
        LevelStatistics& stat = mLastReport.levelStats[level];
        stat.level = level;
        const auto& lvl = mNativePyramid.GetLevel(level);
        stat.width = lvl.width;
        stat.height = lvl.height;
        stat.scaleFactor = mvScaleFactor[level];
        stat.invScaleFactor = mvInvScaleFactor[level];

        const NativeImage& innerImg = lvl.innerImage;
        const int minBorderX = edgeThresh - 3;
        const int minBorderY = minBorderX;
        const int maxBorderX = lvl.width - edgeThresh + 3;
        const int maxBorderY = lvl.height - edgeThresh + 3;

        std::vector<NativeKeyPoint> vCandidates;
        vCandidates.reserve(mSettings.nFeatures * 10);

        const float width = static_cast<float>(maxBorderX - minBorderX);
        const float height = static_cast<float>(maxBorderY - minBorderY);

        const int nCols = std::max(1, static_cast<int>(width / W));
        const int nRows = std::max(1, static_cast<int>(height / W));
        const int wCell = static_cast<int>(std::ceil(width / nCols));
        const int hCell = static_cast<int>(std::ceil(height / nRows));

        auto tFastStart = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < nRows; ++i)
        {
            const int iniY = minBorderY + i * hCell;
            int maxY = iniY + hCell + 6;
            if (iniY >= maxBorderY - 3) continue;
            if (maxY > maxBorderY) maxY = maxBorderY;

            for (int j = 0; j < nCols; ++j)
            {
                const int iniX = minBorderX + j * wCell;
                int maxX = iniX + wCell + 6;
                if (iniX >= maxBorderX - 6) continue;
                if (maxX > maxBorderX) maxX = maxBorderX;

                NativeImage cellROI = innerImg.SubImage(iniX, iniY, maxX - iniX, maxY - iniY);
                std::vector<NativeKeyPoint> vCell;

                // Primary FAST-9 detection
                FastDetector::Detect(cellROI, vCell, mSettings.iniThFAST, true);

                // Fallback FAST-9 detection if cell is empty
                if (vCell.empty())
                {
                    FastDetector::Detect(cellROI, vCell, mSettings.minThFAST, true);
                }

                if (!vCell.empty())
                {
                    for (auto& kp : vCell)
                    {
                        kp.x += j * wCell;
                        kp.y += i * hCell;
                        vCandidates.push_back(kp);
                    }
                }
            }
        }
        auto tFastEnd = std::chrono::high_resolution_clock::now();
        totalFastMs += std::chrono::duration<double, std::milli>(tFastEnd - tFastStart).count();

        stat.fastCandidatesCount = vCandidates.size();
        mLastReport.totalFastCandidates += stat.fastCandidatesCount;

        // Distribute via deterministic quadtree partitioning
        auto tDistStart = std::chrono::high_resolution_clock::now();
        allKeypoints[level] = FeatureDistribution::DistributeOctTree(
            vCandidates, minBorderX, maxBorderX, minBorderY, maxBorderY,
            mnFeaturesPerLevel[level], level);
        auto tDistEnd = std::chrono::high_resolution_clock::now();
        totalDistMs += std::chrono::duration<double, std::milli>(tDistEnd - tDistStart).count();

        const float scaledPatchSize = static_cast<float>(mSettings.patchSize) * mvScaleFactor[level];
        for (auto& kp : allKeypoints[level])
        {
            kp.x += minBorderX;
            kp.y += minBorderY;
            kp.octave = level;
            kp.size = scaledPatchSize;
        }

        stat.selectedKeypointsCount = allKeypoints[level].size();
        mLastReport.totalSelectedKeypoints += stat.selectedKeypointsCount;
    }

    mLastReport.timeFastMs = totalFastMs;
    mLastReport.timeDistributionMs = totalDistMs;

    // Stage 3: Intensity Centroid orientation
    auto tOriStart = std::chrono::high_resolution_clock::now();
    for (int level = 0; level < mSettings.nLevels; ++level)
    {
        const auto& lvl = mNativePyramid.GetLevel(level);
        Orientation::ComputeOrientation(lvl.innerImage, allKeypoints[level], umax.data());
    }
    auto tOriEnd = std::chrono::high_resolution_clock::now();
    mLastReport.timeOrientationMs = std::chrono::duration<double, std::milli>(tOriEnd - tOriStart).count();

    // Stage 4: Gaussian pre-smoothing & Steered BRIEF descriptor extraction
    auto tDescStart = std::chrono::high_resolution_clock::now();
    for (int level = 0; level < mSettings.nLevels; ++level)
    {
        std::vector<NativeKeyPoint>& kpts = allKeypoints[level];
        size_t nkp = kpts.size();
        LevelStatistics& stat = mLastReport.levelStats[level];
        if (nkp == 0) continue;

        const auto& lvl = mNativePyramid.GetLevel(level);
        NativeImage blurredOctave(lvl.width, lvl.height);
        GaussianFilter::Filter7x7(lvl.innerImage, blurredOctave);

        std::vector<NativeDescriptor> lvlDesc;
        BriefDescriptor::ComputeDescriptors(kpts, blurredOctave, lvlDesc);
        stat.descriptorCount = lvlDesc.size();
        mLastReport.totalDescriptors += stat.descriptorCount;

        float scale = mvScaleFactor[level];
        for (size_t i = 0; i < nkp; ++i)
        {
            NativeKeyPoint outKp = kpts[i];
            if (level != 0)
            {
                outKp.x *= scale;
                outKp.y *= scale;
            }
            keypoints.push_back(outKp);
            descriptors.push_back(lvlDesc[i]);
        }
    }
    auto tDescEnd = std::chrono::high_resolution_clock::now();
    mLastReport.timeDescriptorMs = std::chrono::duration<double, std::milli>(tDescEnd - tDescStart).count();

    auto tTotalEnd = std::chrono::high_resolution_clock::now();
    mLastReport.timeTotalMs = std::chrono::duration<double, std::milli>(tTotalEnd - tTotalStart).count();
    mLastReport.fps = (mLastReport.timeTotalMs > 0.0) ? (1000.0 / mLastReport.timeTotalMs) : 0.0;

    if (report)
    {
        *report = mLastReport;
    }
}

void ORBExtractor::Extract(const NativeImage& image,
                           std::vector<ORBFeature>& features,
                           ExtractionReport* report)
{
    features.clear();
    std::vector<NativeKeyPoint> keypoints;
    std::vector<NativeDescriptor> descriptors;

    Extract(image, keypoints, descriptors, report);

    features.reserve(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i)
    {
        ORBFeature feat;
        feat.x = keypoints[i].x;
        feat.y = keypoints[i].y;
        feat.octave = keypoints[i].octave;
        feat.scale = mvScaleFactor[keypoints[i].octave];
        feat.angle = keypoints[i].angle;
        feat.response = keypoints[i].response;
        feat.size = keypoints[i].size;
        std::memcpy(feat.descriptor, descriptors[i].ptr(), 32);
        features.push_back(feat);
    }
}

} // namespace ORB_Standalone
