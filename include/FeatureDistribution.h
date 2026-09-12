#ifndef FEATURE_DISTRIBUTION_H
#define FEATURE_DISTRIBUTION_H

#include "ORBTypes.h"
#include <vector>
#include <list>

namespace ORB_Standalone
{

/**
 * @brief Node for quadtree spatial keypoint partitioning (OctTree distribution)
 */
class NativeExtractorNode
{
public:
    NativeExtractorNode() : bNoMore(false) {}

    // Divide node into 4 quadrant sub-nodes
    void DivideNode(NativeExtractorNode &n1, NativeExtractorNode &n2, NativeExtractorNode &n3, NativeExtractorNode &n4);

    std::vector<NativeKeyPoint> vKeys;
    int UL_x = 0, UL_y = 0;
    int UR_x = 0, UR_y = 0;
    int BL_x = 0, BL_y = 0;
    int BR_x = 0, BR_y = 0;
    std::list<NativeExtractorNode>::iterator lit;
    bool bNoMore;
};

/**
 * @brief Deterministic Quadtree Spatial Keypoint Distribution
 *
 * Distributes FAST corner candidates across image cells to ensure uniform feature density.
 * Uses deterministic spatial tie-breaking (size, UL.y, UL.x) instead of pointer addresses.
 */
class FeatureDistribution
{
public:
    /**
     * @brief Distributes keypoints across the image region using quadtree partitioning.
     *
     * @param vToDistributeKeys Candidate keypoints detected by FAST
     * @param minX Minimum X boundary
     * @param maxX Maximum X boundary
     * @param minY Minimum Y boundary
     * @param maxY Maximum Y boundary
     * @param N Desired feature count for this level
     * @param level Pyramid level
     * @return Retained keypoints after spatial distribution (at most N)
     */
    static std::vector<NativeKeyPoint> DistributeOctTree(
        const std::vector<NativeKeyPoint>& vToDistributeKeys,
        int minX, int maxX, int minY, int maxY,
        int N, int level);
};

} // namespace ORB_Standalone

#endif // FEATURE_DISTRIBUTION_H
