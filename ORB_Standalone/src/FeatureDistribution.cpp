#include "FeatureDistribution.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace ORB_Standalone
{

void NativeExtractorNode::DivideNode(NativeExtractorNode &n1, NativeExtractorNode &n2, NativeExtractorNode &n3, NativeExtractorNode &n4)
{
    const int halfX = static_cast<int>(std::ceil(static_cast<float>(UR_x - UL_x) / 2.0f));
    const int halfY = static_cast<int>(std::ceil(static_cast<float>(BR_y - UL_y) / 2.0f));

    // Quadrant 1 (Top-Left)
    n1.UL_x = UL_x;         n1.UL_y = UL_y;
    n1.UR_x = UL_x + halfX; n1.UR_y = UL_y;
    n1.BL_x = UL_x;         n1.BL_y = UL_y + halfY;
    n1.BR_x = UL_x + halfX; n1.BR_y = UL_y + halfY;
    n1.vKeys.reserve(vKeys.size());

    // Quadrant 2 (Top-Right)
    n2.UL_x = n1.UR_x;      n2.UL_y = UL_y;
    n2.UR_x = UR_x;         n2.UR_y = UR_y;
    n2.BL_x = n1.BR_x;      n2.BL_y = n1.BR_y;
    n2.BR_x = UR_x;         n2.BR_y = UL_y + halfY;
    n2.vKeys.reserve(vKeys.size());

    // Quadrant 3 (Bottom-Left)
    n3.UL_x = n1.BL_x;      n3.UL_y = n1.BL_y;
    n3.UR_x = n1.BR_x;      n3.UR_y = n1.BR_y;
    n3.BL_x = BL_x;         n3.BL_y = BL_y;
    n3.BR_x = n1.BR_x;      n3.BR_y = BL_y;
    n3.vKeys.reserve(vKeys.size());

    // Quadrant 4 (Bottom-Right)
    n4.UL_x = n3.UR_x;      n4.UL_y = n3.UR_y;
    n4.UR_x = n2.BR_x;      n4.UR_y = n2.BR_y;
    n4.BL_x = n3.BR_x;      n4.BL_y = n3.BR_y;
    n4.BR_x = BR_x;         n4.BR_y = BR_y;
    n4.vKeys.reserve(vKeys.size());

    // Associate keypoints to child quadrants
    for (size_t i = 0; i < vKeys.size(); ++i)
    {
        const NativeKeyPoint &kp = vKeys[i];
        if (kp.x < n1.UR_x)
        {
            if (kp.y < n1.BR_y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if (kp.y < n1.BR_y)
        {
            n2.vKeys.push_back(kp);
        }
        else
        {
            n4.vKeys.push_back(kp);
        }
    }

    if (n1.vKeys.size() == 1) n1.bNoMore = true;
    if (n2.vKeys.size() == 1) n2.bNoMore = true;
    if (n3.vKeys.size() == 1) n3.bNoMore = true;
    if (n4.vKeys.size() == 1) n4.bNoMore = true;
}

std::vector<NativeKeyPoint> FeatureDistribution::DistributeOctTree(
    const std::vector<NativeKeyPoint>& vToDistributeKeys,
    int minX, int maxX, int minY, int maxY,
    int N, int level)
{
    (void)level;
    if (vToDistributeKeys.empty() || N <= 0)
        return std::vector<NativeKeyPoint>();

    const int nIni = std::max(1, static_cast<int>(std::round(static_cast<float>(maxX - minX) / (maxY - minY))));
    const float hX = static_cast<float>(maxX - minX) / nIni;

    std::list<NativeExtractorNode> lNodes;
    std::vector<NativeExtractorNode*> vpIniNodes(nIni);

    for (int i = 0; i < nIni; ++i)
    {
        NativeExtractorNode ni;
        ni.UL_x = static_cast<int>(hX * static_cast<float>(i));
        ni.UL_y = 0;
        ni.UR_x = static_cast<int>(hX * static_cast<float>(i + 1));
        ni.UR_y = 0;
        ni.BL_x = ni.UL_x;
        ni.BL_y = maxY - minY;
        ni.BR_x = ni.UR_x;
        ni.BR_y = maxY - minY;
        ni.vKeys.reserve(vToDistributeKeys.size());

        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    // Associate points to initial root nodes
    for (size_t i = 0; i < vToDistributeKeys.size(); ++i)
    {
        const NativeKeyPoint &kp = vToDistributeKeys[i];
        int idx = static_cast<int>(kp.x / hX);
        idx = std::max(0, std::min(idx, nIni - 1));
        vpIniNodes[idx]->vKeys.push_back(kp);
    }

    auto lit = lNodes.begin();
    while (lit != lNodes.end())
    {
        if (lit->vKeys.size() == 1)
        {
            lit->bNoMore = true;
            lit++;
        }
        else if (lit->vKeys.empty())
        {
            lit = lNodes.erase(lit);
        }
        else
        {
            lit++;
        }
    }

    struct NativeExtractorNodeSortKey
    {
        int size;
        int y;
        int x;
        NativeExtractorNode* pNode;

        bool operator<(const NativeExtractorNodeSortKey& other) const
        {
            if (size != other.size)
                return size < other.size;
            if (y != other.y)
                return y < other.y;
            return x < other.x;
        }
    };

    bool bFinish = false;
    std::vector<NativeExtractorNodeSortKey> vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size() * 4);

    while (!bFinish)
    {
        int prevSize = static_cast<int>(lNodes.size());
        lit = lNodes.begin();
        int nToExpand = 0;
        vSizeAndPointerToNode.clear();

        while (lit != lNodes.end())
        {
            if (lit->bNoMore)
            {
                lit++;
                continue;
            }
            else
            {
                NativeExtractorNode n1, n2, n3, n4;
                lit->DivideNode(n1, n2, n3, n4);

                if (!n1.vKeys.empty())
                {
                    lNodes.push_front(n1);
                    if (n1.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back({static_cast<int>(n1.vKeys.size()), n1.UL_y, n1.UL_x, &lNodes.front()});
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if (!n2.vKeys.empty())
                {
                    lNodes.push_front(n2);
                    if (n2.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back({static_cast<int>(n2.vKeys.size()), n2.UL_y, n2.UL_x, &lNodes.front()});
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if (!n3.vKeys.empty())
                {
                    lNodes.push_front(n3);
                    if (n3.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back({static_cast<int>(n3.vKeys.size()), n3.UL_y, n3.UL_x, &lNodes.front()});
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if (!n4.vKeys.empty())
                {
                    lNodes.push_front(n4);
                    if (n4.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back({static_cast<int>(n4.vKeys.size()), n4.UL_y, n4.UL_x, &lNodes.front()});
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit = lNodes.erase(lit);
                continue;
            }
        }

        if (static_cast<int>(lNodes.size()) >= N || static_cast<int>(lNodes.size()) == prevSize)
        {
            bFinish = true;
        }
        else if ((static_cast<int>(lNodes.size()) + nToExpand * 3) > N)
        {
            while (!bFinish)
            {
                prevSize = static_cast<int>(lNodes.size());
                std::vector<NativeExtractorNodeSortKey> vPrev = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                std::sort(vPrev.begin(), vPrev.end());
                for (int j = static_cast<int>(vPrev.size()) - 1; j >= 0; --j)
                {
                    NativeExtractorNode n1, n2, n3, n4;
                    vPrev[j].pNode->DivideNode(n1, n2, n3, n4);

                    if (!n1.vKeys.empty())
                    {
                        lNodes.push_front(n1);
                        if (n1.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back({static_cast<int>(n1.vKeys.size()), n1.UL_y, n1.UL_x, &lNodes.front()});
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (!n2.vKeys.empty())
                    {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back({static_cast<int>(n2.vKeys.size()), n2.UL_y, n2.UL_x, &lNodes.front()});
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (!n3.vKeys.empty())
                    {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back({static_cast<int>(n3.vKeys.size()), n3.UL_y, n3.UL_x, &lNodes.front()});
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (!n4.vKeys.empty())
                    {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back({static_cast<int>(n4.vKeys.size()), n4.UL_y, n4.UL_x, &lNodes.front()});
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrev[j].pNode->lit);

                    if (static_cast<int>(lNodes.size()) >= N)
                        break;
                }

                if (static_cast<int>(lNodes.size()) >= N || static_cast<int>(lNodes.size()) == prevSize)
                    bFinish = true;
            }
        }
    }

    // Retain single highest FAST response keypoint in each leaf node
    std::vector<NativeKeyPoint> vResultKeys;
    vResultKeys.reserve(lNodes.size());
    for (auto nodeIt = lNodes.begin(); nodeIt != lNodes.end(); ++nodeIt)
    {
        std::vector<NativeKeyPoint> &vNodeKeys = nodeIt->vKeys;
        if (vNodeKeys.empty()) continue;

        NativeKeyPoint* pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;

        for (size_t k = 1; k < vNodeKeys.size(); ++k)
        {
            if (vNodeKeys[k].response > maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}

} // namespace ORB_Standalone
