# ORB-Standalone

A lightweight, modular, dependency-free C++ implementation of **ORB (Oriented FAST and Rotated BRIEF)** for feature detection and binary description.

ORB-Standalone provides a self-contained ORB feature-extraction core designed for **visual odometry, VIO, SLAM, visual tracking, image matching, robotics, embedded vision, and hardware-oriented computer-vision systems**.

The computational core is implemented using native C++ data structures and **does not depend on OpenCV or any other computer-vision framework**.

The library accepts an 8-bit grayscale image buffer and produces oriented keypoints with 256-bit binary descriptors.

An optional webcam example uses OpenCV only as an external application layer for camera capture and visualization. **OpenCV is not required by the ORB core.**

---

## Why ORB-Standalone?

Computer-vision frameworks such as OpenCV provide extremely powerful and highly optimized functionality, but they are designed primarily as general-purpose software frameworks.

For embedded, real-time, FPGA, DSP, microcontroller, or custom-vision applications, a complete computer-vision framework can introduce dependencies and software infrastructure that are unnecessary when only a specific algorithm is required.

ORB-Standalone takes a different approach:

```text
Application / Camera / Sensor
             │
             ▼
      8-bit Grayscale Buffer
             │
             ▼
      ┌──────────────────┐
      │  ORB-Standalone  │
      │                  │
      │ Image Pyramid    │
      │ FAST             │
      │ NMS              │
      │ Distribution     │
      │ Orientation      │
      │ BRIEF            │
      └────────┬─────────┘
               │
               ▼
      Keypoints + Descriptors
```

The ORB algorithm is isolated from:

* camera drivers
* GUI frameworks
* operating-system-specific APIs
* image containers such as `cv::Mat`
* computer-vision frameworks
* visualization systems
* application-specific code

This makes the computational core easier to integrate, port, optimize, and eventually map to dedicated hardware.

---

# Overview

ORB combines several stages to produce scale-aware and rotation-aware binary image features:

* FAST-based keypoint detection
* Multi-scale image pyramids
* Non-maximum suppression
* Spatial feature distribution
* Intensity-centroid orientation estimation
* Gaussian image smoothing
* Steered BRIEF descriptors

The resulting features consist of:

```text
Keypoint
    ├── x
    ├── y
    ├── octave
    ├── angle
    ├── response
    └── scale

Descriptor
    └── 256 bits / 32 bytes
```

The binary descriptors can be compared efficiently using **Hamming distance**.

---

# ORB Pipeline

```text
                 Grayscale Image
                        │
                        ▼
                Image Pyramid
                        │
                        ▼
                 FAST Detection
                        │
                        ▼
                 FAST Scoring
                        │
                        ▼
                 Non-Maximum
                   Suppression
                        │
                        ▼
             Spatial Distribution
                        │
                        ▼
             Orientation Estimation
                        │
                        ▼
                Image Smoothing
                        │
                        ▼
               Steered BRIEF-256
                        │
                        ▼
          Keypoints + Descriptors
```

The implementation is modular, so each stage can be independently tested, optimized, replaced, or mapped to another execution platform.

---

# Architecture

ORB-Standalone separates **image acquisition** from **feature extraction**.

```text
 Camera / Sensor / Image / Video
                │
                ▼
          Image Acquisition
                │
                ▼
       8-bit Grayscale Buffer
                │
                ▼
        ┌─────────────────┐
        │  ORB-Standalone │
        │                 │
        │ Image Pyramid   │
        │ FAST Detection  │
        │ NMS             │
        │ Distribution    │
        │ Orientation     │
        │ BRIEF           │
        └────────┬────────┘
                 │
                 ▼
        Keypoints + Descriptors
                 │
        ┌────────┼─────────┐
        ▼        ▼         ▼
       VO       VIO       SLAM
```

## Design Rule

The ORB computational core does **not** access:

* cameras
* camera drivers
* GUI systems
* display APIs
* filesystem APIs
* operating-system-specific capture interfaces
* OpenCV

The application provides an image buffer to the extractor.

This allows the same ORB implementation to operate with different image sources without modifying the feature-extraction algorithm.

---

# Main Characteristics

* Pure C++ computational core
* **Zero OpenCV dependency in the core**
* No external computer-vision framework required
* 256-bit / 32-byte binary descriptors
* Multi-scale feature extraction
* Rotation-aware descriptors
* Deterministic extraction
* Modular architecture
* Native image-buffer interface
* Configurable extraction parameters
* Spatially distributed features
* Suitable for desktop and embedded Linux
* Designed for RTOS and bare-metal porting
* Suitable for FPGA/HLS-oriented hardware partitioning
* Optional OpenCV webcam demonstration
* Easy integration with custom cameras and image sources

---

# Multi-Scale Feature Extraction

The extractor constructs an image pyramid so that features can be detected at multiple image scales.

Default configuration:

| Parameter              |  Default |
| ---------------------- | -------: |
| Number of features     |     1000 |
| Scale factor           |      1.2 |
| Pyramid levels         |        8 |
| Initial FAST threshold |       20 |
| Minimum FAST threshold |        7 |
| Descriptor size        | 256 bits |
| Descriptor storage     | 32 bytes |

These parameters can be configured through `ORBSettings`.

---

# FAST Keypoint Detection

ORB-Standalone uses the FAST corner detector as its primary keypoint detector.

The detector evaluates the standard FAST circular pixel pattern:

```text
             0  1  2
          15       3
        14           4
       13     C       5
        12           6
          11       7
             10 9 8
```

The implementation uses the FAST-9 criterion on the 16-pixel circle.

An adaptive threshold mechanism allows the detector to use:

```text
Initial threshold
        │
        ▼
      FAST
        │
        ├── sufficient features ──► accept
        │
        └── insufficient features
                    │
                    ▼
             lower threshold
                    │
                    ▼
              FAST fallback
```

This helps maintain feature availability in low-contrast image regions.

---

# FAST Scoring and NMS

Detected FAST candidates are scored and filtered using non-maximum suppression.

The implementation uses FAST-based response scoring rather than requiring a separate floating-point Harris response calculation.

This keeps the detector:

* computationally lightweight
* integer/fixed-point friendly
* suitable for embedded systems
* suitable for hardware acceleration

After scoring, non-maximum suppression removes weaker neighboring responses.

---

# Spatial Feature Distribution

Simply selecting the strongest image features can result in features clustering in a small number of highly textured regions.

ORB-Standalone therefore performs spatial feature distribution using hierarchical subdivision.

Conceptually:

```text
Initial image region
        │
        ▼
    ┌───────┐
    │       │
    └───────┘
        │
        ▼
   subdivide regions
        │
        ▼
 ┌───┬───┬───┬───┐
 │   │   │   │   │
 ├───┼───┼───┼───┤
 │   │   │   │   │
 └───┴───┴───┴───┘
        │
        ▼
 Select strongest feature
 from terminal regions
```

The purpose is to obtain a more spatially distributed feature set.

This is particularly useful for visual tracking and visual odometry, where features concentrated in one portion of the image can reduce geometric robustness.

---

# Orientation Estimation

Each selected keypoint is assigned an orientation using the **intensity-centroid method**.

The local image moments are used to estimate the direction of the intensity centroid relative to the keypoint.

Conceptually:

```text
        Local image patch
               │
               ▼
       Intensity moments
               │
          ┌────┴────┐
          ▼         ▼
         m10       m01
          │         │
          └────┬────┘
               ▼
        atan2(m01,m10)
               │
               ▼
        Keypoint angle
```

The orientation is then used to rotate the BRIEF sampling pattern.

---

# Gaussian Smoothing

Before descriptor generation, the implementation applies a separable fixed-point Gaussian-style smoothing filter.

The filter is implemented using integer/fixed-point arithmetic rather than requiring floating-point image-processing frameworks.

This provides:

* deterministic behavior
* reduced computational complexity
* simple memory access
* suitability for embedded implementations
* suitability for hardware pipelines

---

# Steered BRIEF Descriptor

Each keypoint receives a **256-bit binary descriptor**.

The descriptor is generated from intensity comparisons between pairs of pixels in a local image patch.

Conceptually:

```text
             Keypoint
                 │
                 ▼
          Local image patch
                 │
                 ▼
       Rotate sampling pattern
                 │
                 ▼
       256 intensity comparisons
                 │
                 ▼
          256 binary bits
                 │
                 ▼
             32 bytes
```

The descriptor is rotation-aware because the BRIEF sampling pattern is rotated according to the keypoint orientation.

Descriptors can be compared using Hamming distance:

```text
Descriptor A
     XOR
Descriptor B
     │
     ▼
 Population Count
     │
     ▼
Hamming Distance
```

A smaller Hamming distance indicates greater descriptor similarity.

---

# OpenCV Independence

## Computational Core

The ORB computational core is **completely independent of OpenCV**.

The following components are implemented using native C++:

```text
ORBTypes
Border
ImagePyramid
FastDetector
FeatureDistribution
Atan2
Orientation
GaussianFilter
BriefDescriptor
ORBExtractor
OrbPattern
```

The core contains:

* no OpenCV headers
* no `cv::Mat`
* no OpenCV feature detector
* no OpenCV descriptor generator
* no OpenCV image pyramid
* no OpenCV filtering
* no OpenCV matcher

The core does not link against OpenCV.

The required interface is simply an 8-bit grayscale image buffer.

---

# Why Avoid OpenCV in the Core?

OpenCV is an excellent general-purpose computer-vision framework and is highly optimized for desktop and server-class processors.

However, a complete computer-vision framework is not always desirable when the target is a constrained or specialized platform.

ORB-Standalone is intended for cases where the application needs **the ORB algorithm itself rather than an entire computer-vision framework**.

Advantages of the dependency-free architecture include:

### Smaller software dependency footprint

The computational core requires only standard C++ functionality rather than a large external computer-vision framework.

### Easier platform integration

The extractor communicates through a simple image-buffer interface rather than an operating-system-specific image abstraction.

### Easier embedded deployment

The algorithm can be integrated into embedded software without requiring OpenCV.

### Hardware-oriented design

The major processing stages have explicit data paths and well-defined computational operations, making them easier to analyze for FPGA, DSP, accelerator, or HLS implementations.

### Algorithm ownership

Every major ORB processing stage is directly implemented in the project rather than hidden behind a general-purpose framework API.

---

# Portability and Bare-Metal Design

ORB-Standalone is designed with portability as a primary architectural goal.

The ORB core does not inherently require:

* Linux
* Windows
* macOS
* a graphical desktop
* camera drivers
* USB
* networking
* filesystem access
* OpenCV
* ROS
* a display
* an operating-system camera API

The fundamental interface is:

```text
Input:
    8-bit grayscale image buffer

Output:
    Keypoints
    +
    256-bit descriptors
```

Therefore, the algorithmic core can be adapted to environments such as:

```text
Desktop CPU
     │
     ├── Embedded Linux
     │
     ├── ARM processor
     │
     ├── RTOS
     │
     ├── DSP
     │
     ├── Microcontroller
     │
     └── FPGA / hardware accelerator
```

## Bare-Metal Consideration

The core is **designed to be portable to bare-metal and RTOS environments**, because it does not require an operating-system API or OpenCV.

However, the current reference build uses standard C++ facilities and is primarily validated on conventional desktop/Linux systems.

A specific bare-metal target may require:

* replacement or configuration of dynamic memory allocation
* suitable C++ runtime support
* platform-specific startup code
* custom memory management
* hardware-specific image acquisition
* appropriate integer/math implementations

Therefore, bare-metal support is an **architectural portability target**, not a claim that every microcontroller can compile the current CMake project unchanged.

---

# Native Image Interface

The ORB extractor accepts an 8-bit single-channel grayscale image through a lightweight native image view.

Example:

```cpp
NativeImage image(
    pixels.data(),
    width,
    height,
    stride
);
```

The parameters are:

| Parameter | Description                              |
| --------- | ---------------------------------------- |
| `data`    | Pointer to grayscale image pixels        |
| `width`   | Image width in pixels                    |
| `height`  | Image height in pixels                   |
| `stride`  | Number of bytes between consecutive rows |

The image buffer is supplied by the application.

The ORB extractor does not require the image to be copied into an OpenCV container.

---

# Using the ORB Library

Minimal example:

```cpp
#include <iostream>
#include <vector>

#include "ORBExtractor.h"
#include "ORBTypes.h"

int main()
{
    using namespace ORB_Standalone;

    ORBSettings settings;

    settings.nFeatures = 1000;
    settings.scaleFactor = 1.2f;
    settings.nLevels = 8;
    settings.iniThFAST = 20;
    settings.minThFAST = 7;

    ORBExtractor extractor(settings);

    const int width = 640;
    const int height = 480;

    std::vector<uint8_t> pixels(
        width * height,
        128
    );

    NativeImage image(
        pixels.data(),
        width,
        height,
        width
    );

    std::vector<NativeKeyPoint> keypoints;
    std::vector<NativeDescriptor> descriptors;

    extractor.Extract(
        image,
        keypoints,
        descriptors
    );

    std::cout
        << "Detected "
        << keypoints.size()
        << " features\n";

    return 0;
}
```

---

# Camera and Image Sources

The ORB core does not acquire images itself.

The application is responsible for:

1. acquiring an image
2. converting it to 8-bit grayscale if necessary
3. providing the image buffer to `NativeImage`
4. calling `ORBExtractor`

The architecture therefore looks like:

```text
Camera / Sensor / File
        │
        ▼
   Acquire Frame
        │
        ▼
Convert to Grayscale
        │
        ▼
  8-bit Image Buffer
        │
        ▼
    NativeImage
        │
        ▼
   ORBExtractor
        │
        ▼
Keypoints + Descriptors
```

This makes camera-specific code independent from the ORB algorithm.

---

# Supported Integration Approaches

## USB Camera

A USB camera can be accessed using:

* V4L2
* OpenCV
* another camera API

The captured image can then be converted to grayscale and passed to ORB-Standalone.

## Raspberry Pi / CSI Camera

Camera frameworks such as `libcamera` can provide image data to an application.

The grayscale/Y-plane buffer can then be passed to the ORB extractor.

## Embedded Camera

A custom embedded camera pipeline can directly provide an 8-bit grayscale buffer.

No OpenCV layer is required.

## ROS / ROS 2

A ROS node can receive an image message, convert it to the required format, and pass the image buffer to ORB-Standalone.

ROS is not required by the ORB core.

## Image and Video Files

Any image/video decoding library can be used externally.

The decoded grayscale frame is then passed to ORB-Standalone.

## Custom Sensors

Any system capable of producing an 8-bit grayscale image buffer can be connected to the extractor.

---

# Webcam Demo

The repository contains an optional OpenCV-based webcam demonstration:

```text
examples/webcam_demo.cpp
```

The demo uses OpenCV only for:

* camera capture
* grayscale conversion
* visualization
* GUI/window handling
* annotated output

**This application is not part of the ORB computational core.**

The ORB library itself remains OpenCV-independent.

Build:

```bash
mkdir -p build
cd build

cmake ..

cmake --build . -j$(nproc)
```

If OpenCV is available, the webcam demonstration is built.

Run:

```bash
./webcam_demo
```

Select a camera:

```bash
./webcam_demo 1
```

Specify camera and resolution:

```bash
./webcam_demo 0 1280 720
```

The demonstration displays:

* detected ORB keypoints
* keypoint orientation
* pyramid level
* feature count
* image resolution
* ORB processing time
* processing FPS

## Keyboard Controls

| Key   | Action                                |
| ----- | ------------------------------------- |
| `ESC` | Exit                                  |
| `Q`   | Exit                                  |
| `S`   | Save annotated frame and feature data |
| `D`   | Toggle information overlay            |

---

# Building

## Core Requirements

The core library requires:

* C++14-compatible compiler
* CMake 3.10 or newer

The core does **not** require OpenCV.

Build:

```bash
mkdir -p build
cd build

cmake ..

cmake --build . -j$(nproc)
```

The static library is generated as:

```text
build/liborb_standalone_lib.a
```

---

# Optional Webcam Requirements

The optional webcam demonstration requires:

* OpenCV 3.x or 4.x
* supported camera device

If OpenCV is not available, the computational core can still be built and used independently.

---

# Output

The extractor produces:

```cpp
std::vector<NativeKeyPoint>
std::vector<NativeDescriptor>
```

A `NativeKeyPoint` contains:

```text
x
y
octave
angle
response
size
```

Coordinates are returned in the original image coordinate system.

A `NativeDescriptor` contains:

```text
32 bytes
=
256 bits
```

These descriptors are suitable for Hamming-distance matching.

---

# Determinism

ORB-Standalone is designed to produce deterministic results.

Given:

```text
same image
+
same configuration
+
same extractor state
```

the extraction process produces repeatable:

* keypoint locations
* orientations
* feature responses
* pyramid levels
* descriptor contents

This property is useful for:

* testing
* embedded implementations
* hardware verification
* regression testing
* FPGA/software comparison

---

# Validation

The implementation has been validated through unit-level, numerical, robustness, and integration testing.

Validation includes:

* image pyramid construction
* border handling
* FAST detection
* FAST scoring
* non-maximum suppression
* spatial feature distribution
* orientation estimation
* Gaussian filtering
* BRIEF descriptor generation
* keypoint coordinate scaling
* descriptor consistency
* deterministic repeated extraction
* different image resolutions
* low-contrast images
* noisy images
* brightness changes
* contrast changes
* image transformations
* frame-to-frame feature matching
* memory safety
* repeated extraction
* OpenCV independence

Reference comparisons have also been performed against OpenCV ORB/FAST behavior for selected image and descriptor tests.

---

# Performance

ORB-Standalone prioritizes:

* algorithmic transparency
* portability
* modularity
* deterministic behavior
* hardware-oriented implementation
* minimal external dependencies

It is **not intended to outperform highly optimized desktop OpenCV implementations on a general-purpose CPU**.

For example, desktop OpenCV can use architecture-specific SIMD instructions and multithreading that are not used by the current scalar ORB-Standalone implementation.

A representative 640×480 comparison is:

| Implementation      | Approx. Runtime |
| ------------------- | --------------: |
| OpenCV ORB          |         ~5.1 ms |
| ORB-SLAM3 extractor |       ~18–25 ms |
| ORB-Standalone      |        ~25.2 ms |

The current implementation therefore trades desktop CPU performance for a simpler, dependency-free computational architecture.

This trade-off is intentional.

The software implementation provides a clean baseline for later optimization and hardware acceleration.

---

# FPGA and Hardware Acceleration

ORB-Standalone is structured so that computationally intensive stages can be analyzed and accelerated independently.

A potential hardware architecture is:

```text
                  Image Stream
                       │
                       ▼
              ┌────────────────┐
              │ Image Pyramid  │
              └───────┬────────┘
                      │
                      ▼
              ┌────────────────┐
              │ FAST Detector  │
              │                │
              │ Parallel       │
              │ Comparators    │
              └───────┬────────┘
                      │
                      ▼
              ┌────────────────┐
              │      NMS       │
              └───────┬────────┘
                      │
                      ▼
              Feature Candidates
                      │
                      ▼
                 CPU / ARM
                      │
          ┌───────────┼───────────┐
          ▼           ▼           ▼
     Distribution  Orientation  BRIEF
```

Potential hardware suitability:

| Stage                | Hardware Suitability |
| -------------------- | -------------------- |
| Image Pyramid        | High                 |
| FAST Detection       | Very High            |
| NMS                  | Very High            |
| Gaussian Filtering   | Very High            |
| Orientation          | Moderate–High        |
| BRIEF                | Moderate–High        |
| Feature Distribution | Low–Moderate         |

FAST is particularly attractive for hardware acceleration because its local comparisons expose substantial parallelism.

The software implementation therefore provides a useful algorithmic reference before hardware implementation.

---

# Embedded Deployment Philosophy

The project follows a separation between:

```text
Algorithm
    │
    ▼
ORB-Standalone
    │
    ├── Desktop application
    ├── Embedded Linux
    ├── RTOS application
    ├── Custom camera pipeline
    └── Hardware accelerator
```

The image acquisition layer can change without changing the ORB algorithm.

For example:

```text
V4L2
   │
   ├──────────────┐
libcamera         │
   │              │
Custom camera     │
   │              ▼
             NativeImage
                 │
                 ▼
            ORBExtractor
```

This separation is important when the same vision algorithm must eventually run on different hardware platforms.

---

# Applications

ORB-Standalone can serve as a feature-extraction component for:

* Visual Odometry
* Visual-Inertial Odometry
* Visual SLAM
* Feature tracking
* Image matching
* Image alignment
* Image stitching
* Robotics
* Embedded computer vision
* Autonomous systems
* FPGA-based vision
* Hardware accelerators
* Academic research
* Custom computer-vision pipelines

The repository provides the **ORB feature-extraction layer**.

Higher-level systems such as:

* camera tracking
* pose estimation
* visual odometry
* keyframe management
* mapping
* loop closure
* bundle adjustment
* SLAM

can be built around the output of the extractor.

---

# Project Structure

```text
ORB-Standalone/
│
├── include/
│   ├── Atan2.h
│   ├── Border.h
│   ├── BriefDescriptor.h
│   ├── FastDetector.h
│   ├── FeatureDistribution.h
│   ├── GaussianFilter.h
│   ├── ImagePyramid.h
│   ├── ORBExtractor.h
│   ├── ORBTypes.h
│   ├── OrbPattern.h
│   └── Orientation.h
│
├── src/
│   ├── Atan2.cpp
│   ├── Border.cpp
│   ├── BriefDescriptor.cpp
│   ├── FastDetector.cpp
│   ├── FeatureDistribution.cpp
│   ├── GaussianFilter.cpp
│   ├── ImagePyramid.cpp
│   └── Orientation.cpp
│
├── examples/
│   └── webcam_demo.cpp
│
├── CMakeLists.txt
├── README.md
├── LICENSE
└── .gitignore
```

---

# Design Goals

## Modularity

Each major stage of ORB is implemented as an independent module.

This makes the implementation easier to:

* test
* debug
* optimize
* replace
* port
* accelerate

## Portability

The core uses a native image-buffer interface and avoids operating-system-specific image acquisition.

The architecture is suitable for desktop, embedded, RTOS, and future bare-metal integration.

## Independence

The ORB algorithm does not depend on OpenCV.

Camera acquisition, visualization, and application logic remain outside the computational core.

## Hardware Readiness

The algorithm is divided into explicit processing stages with well-defined data flow, making computational bottlenecks easier to identify for FPGA, DSP, ASIC, or HLS acceleration.

## Practicality

The extractor can be integrated into custom visual-odometry, tracking, robotics, and SLAM systems without requiring a complete computer-vision framework.

---

# Quick Start

Clone the repository:

```bash
git clone <repository-url>

cd ORB-Standalone
```

Build the core:

```bash
mkdir build
cd build

cmake ..

cmake --build . -j$(nproc)
```

The core library can then be linked into an application.

If OpenCV is installed, the optional webcam demonstration can be run:

```bash
./webcam_demo
```

For custom integration:

```cpp
#include "ORBExtractor.h"
#include "ORBTypes.h"
```

Provide an 8-bit grayscale image buffer to `ORBExtractor`.

No OpenCV image container is required.

---

# Project Status

ORB-Standalone currently provides a validated, standalone ORB feature-extraction implementation.

The ORB core is intentionally separated from higher-level vision algorithms.

Current scope:

```text
Image
  │
  ▼
ORB Feature Extraction
  │
  ├── Keypoints
  └── Descriptors
```

Future systems can build on this interface:

```text
ORB
 │
 ├── Feature Matching
 │
 ├── Visual Odometry
 │
 ├── Visual Tracking
 │
 └── Visual SLAM
```

The separation allows the ORB implementation to remain independently testable and reusable.

---

# License

This project is licensed under the **MIT License**.

See [`LICENSE`](LICENSE) for the complete license text.

---

# Summary

ORB-Standalone is a **lightweight, modular, OpenCV-independent implementation of ORB**.

Its primary design objective is not to compete with highly optimized desktop computer-vision frameworks in raw CPU performance.

Instead, it provides:

```text
        Self-contained ORB
                │
        ┌───────┼────────┐
        ▼       ▼        ▼
      Desktop Embedded  FPGA
        │       │        │
        ▼       ▼        ▼
       CPU     ARM    Hardware
```

The core requires only an image buffer and produces standard ORB-style keypoints and 256-bit binary descriptors.

By separating image acquisition from feature extraction, the same computational core can be integrated into different camera, embedded, robotics, and hardware environments.

**The goal is simple: implement the algorithm once, control the complete pipeline, and make it portable to wherever the vision system needs to run.**
