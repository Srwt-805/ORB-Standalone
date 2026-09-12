#ifndef ORB_TYPES_H
#define ORB_TYPES_H

#include <vector>
#include <string>
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace ORB_Standalone
{

/**
 * @brief Contiguous 8-bit grayscale image buffer with explicit dimensions and stride.
 * Designed for predictable memory access and zero-copy sub-image operations.
 */
struct NativeImage
{
    uint8_t* data = nullptr;
    int width = 0;
    int height = 0;
    int stride = 0;
    bool ownsData = false;

    NativeImage() = default;

    NativeImage(int w, int h, int s = 0)
        : width(w), height(h), stride(s > 0 ? s : w), ownsData(true)
    {
        if (width > 0 && height > 0)
        {
            data = new uint8_t[height * stride];
            std::memset(data, 0, height * stride);
        }
    }

    NativeImage(uint8_t* ptr, int w, int h, int s = 0)
        : data(ptr), width(w), height(h), stride(s > 0 ? s : w), ownsData(false)
    {
    }

    ~NativeImage()
    {
        Release();
    }

    NativeImage(const NativeImage& other)
        : width(other.width), height(other.height), stride(other.stride), ownsData(false)
    {
        if (other.ownsData && other.data && width > 0 && height > 0)
        {
            ownsData = true;
            data = new uint8_t[height * stride];
            std::memcpy(data, other.data, height * stride);
        }
        else
        {
            data = other.data;
        }
    }

    NativeImage& operator=(const NativeImage& other)
    {
        if (this != &other)
        {
            Release();
            width = other.width;
            height = other.height;
            stride = other.stride;
            if (other.ownsData && other.data && width > 0 && height > 0)
            {
                ownsData = true;
                data = new uint8_t[height * stride];
                std::memcpy(data, other.data, height * stride);
            }
            else
            {
                data = other.data;
                ownsData = false;
            }
        }
        return *this;
    }

    NativeImage(NativeImage&& other) noexcept
        : data(other.data), width(other.width), height(other.height), stride(other.stride), ownsData(other.ownsData)
    {
        other.data = nullptr;
        other.width = 0;
        other.height = 0;
        other.stride = 0;
        other.ownsData = false;
    }

    NativeImage& operator=(NativeImage&& other) noexcept
    {
        if (this != &other)
        {
            Release();
            data = other.data;
            width = other.width;
            height = other.height;
            stride = other.stride;
            ownsData = other.ownsData;
            other.data = nullptr;
            other.width = 0;
            other.height = 0;
            other.stride = 0;
            other.ownsData = false;
        }
        return *this;
    }

    void Allocate(int w, int h, int s = 0)
    {
        Release();
        width = w;
        height = h;
        stride = (s > 0) ? s : w;
        ownsData = true;
        data = new uint8_t[height * stride];
        std::memset(data, 0, height * stride);
    }

    void Release()
    {
        if (ownsData && data)
        {
            delete[] data;
        }
        data = nullptr;
        width = 0;
        height = 0;
        stride = 0;
        ownsData = false;
    }

    inline bool empty() const { return data == nullptr || width <= 0 || height <= 0; }

    inline uint8_t* RowPtr(int y) { return data + y * stride; }
    inline const uint8_t* RowPtr(int y) const { return data + y * stride; }

    inline uint8_t& at(int y, int x) { return data[y * stride + x]; }
    inline const uint8_t& at(int y, int x) const { return data[y * stride + x]; }

    NativeImage SubImage(int x, int y, int w, int h) const
    {
        NativeImage sub;
        sub.data = data + y * stride + x;
        sub.width = w;
        sub.height = h;
        sub.stride = stride;
        sub.ownsData = false;
        return sub;
    }
};

/**
 * @brief Lightweight 2D keypoint structure
 */
struct NativeKeyPoint
{
    float x = 0.0f;
    float y = 0.0f;
    float response = 0.0f;
    float angle = -1.0f;
    float size = 0.0f;
    int octave = 0;

    NativeKeyPoint() = default;
    NativeKeyPoint(float _x, float _y, float _size = 0.0f, float _angle = -1.0f, float _response = 0.0f, int _octave = 0)
        : x(_x), y(_y), response(_response), angle(_angle), size(_size), octave(_octave) {}
};

/**
 * @brief Fixed-size 256-bit (32-byte) binary descriptor
 */
struct NativeDescriptor
{
    uint8_t data[32] = {0};

    inline uint8_t& operator[](size_t idx) { return data[idx]; }
    inline const uint8_t& operator[](size_t idx) const { return data[idx]; }
    inline uint8_t* ptr() { return data; }
    inline const uint8_t* ptr() const { return data; }
};

/**
 * @brief Scale level descriptor for the image pyramid
 */
struct NativePyramidLevel
{
    NativeImage fullImage;   // Includes edgeThreshold border padding (width + 2*pad, height + 2*pad)
    NativeImage innerImage;  // Sub-image viewing the active region without padding
    int width = 0;
    int height = 0;
    float scale = 1.0f;
    float invScale = 1.0f;
    float sigma2 = 1.0f;
    float invSigma2 = 1.0f;
};

/**
 * @brief Extracted feature keypoint with full metadata and 256-bit descriptor
 */
struct ORBFeature
{
    float x;                 // Coordinate X (relative to level 0)
    float y;                 // Coordinate Y (relative to level 0)
    int octave;              // Pyramid level (0 .. nlevels-1)
    float scale;             // Scale factor at this octave (scaleFactor^octave)
    float angle;             // Orientation angle in degrees [0, 360)
    float response;          // FAST corner response score
    float size;              // Diameter of the meaningful keypoint neighborhood
    uint8_t descriptor[32];  // 256-bit steered BRIEF descriptor (32 bytes)
};

/**
 * @brief Configuration parameters for the ORB feature extractor
 */
struct ORBSettings
{
    int nFeatures = 1000;          // Number of desired features
    float scaleFactor = 1.2f;      // Scale pyramid decimation ratio
    int nLevels = 8;               // Number of pyramid levels
    int iniThFAST = 20;            // Initial FAST corner detection threshold
    int minThFAST = 7;             // Fallback FAST threshold if cell has 0 points
    int patchSize = 31;            // Patch size for descriptor & orientation
    int halfPatchSize = 15;        // Half patch size (radius for orientation)
    int edgeThreshold = 19;        // Border safety margin for keypoint extraction
    int fastCellSize = 30;         // Grid cell size in pixels for FAST detection (W=30)
};

/**
 * @brief Statistics per scale level for extraction verification
 */
struct LevelStatistics
{
    int level = 0;
    int width = 0;
    int height = 0;
    float scaleFactor = 1.0f;
    float invScaleFactor = 1.0f;
    size_t fastCandidatesCount = 0;
    size_t selectedKeypointsCount = 0;
    size_t descriptorCount = 0;
};

/**
 * @brief Complete summary report of an extraction run
 */
struct ExtractionReport
{
    std::string imageName;
    int inputWidth = 0;
    int inputHeight = 0;
    int nLevels = 0;
    size_t totalFastCandidates = 0;
    size_t totalSelectedKeypoints = 0;
    size_t totalDescriptors = 0;
    // Stage Timing in milliseconds
    double timePyramidMs = 0.0;
    double timeFastMs = 0.0;
    double timeDistributionMs = 0.0;
    double timeOrientationMs = 0.0;
    double timeDescriptorMs = 0.0;
    double timeTotalMs = 0.0;
    double fps = 0.0;

    std::vector<LevelStatistics> levelStats;
};

} // namespace ORB_Standalone

#endif // ORB_TYPES_H
