# ORB-Standalone

A lightweight, modular C++ implementation of **ORB (Oriented FAST and Rotated BRIEF)** for feature detection and binary description.

ORB-Standalone provides a self-contained ORB feature extraction core that can be used with **visual odometry, VIO, SLAM, tracking, image matching, robotics, embedded vision, or academic projects**.

The computational core uses native C++ data structures and does not depend on OpenCV or any other computer-vision framework. An optional webcam example uses OpenCV only for camera capture and visualization.

---

## Overview

ORB combines:

* FAST-based keypoint detection
* Multi-scale image pyramids
* Spatial feature distribution
* Intensity-centroid orientation estimation
* Rotated BRIEF descriptors

The output is a set of oriented keypoints and **256-bit binary descriptors** suitable for fast feature matching using Hamming distance.

### Main characteristics

* Pure C++ computational core
* No OpenCV dependency in the core library
* 256-bit / 32-byte binary descriptors
* Multi-scale feature extraction
* Rotation-aware descriptors
* Deterministic feature extraction
* Modular implementation
* Native image-buffer interface
* Suitable for desktop and embedded Linux
* Optional OpenCV webcam demonstration
* Easy integration with custom cameras and image sources

---

## ORB Pipeline

```text
                Grayscale Image
                       │
                       ▼
              Image Pyramid
                       │
                       ▼
              FAST Keypoints
                       │
                       ▼
          Corner Response / NMS
                       │
                       ▼
          Spatial Feature Distribution
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
        Keypoints + 256-bit Descriptors
```

---

## Architecture

The library separates image acquisition from feature extraction.

```text
 Camera / Image / Video Source
              │
              ▼
       Grayscale Frame Buffer
              │
              ▼
        ┌─────────────────┐
        │  ORB-Standalone │
        │                 │
        │ Image Pyramid   │
        │ FAST Detection  │
        │ Distribution    │
        │ Orientation     │
        │ BRIEF           │
        └────────┬────────┘
                 │
                 ▼
       Keypoints + Descriptors
                 │
       ┌─────────┼──────────┐
       ▼         ▼          ▼
      VO        VIO        SLAM
                 │
            Tracking /
          Image Matching
```

### Design rule

The ORB core **does not access cameras, camera drivers, GUI frameworks, or operating-system-specific capture APIs**.

Your application is responsible for obtaining an image and providing it to the ORB extractor as an 8-bit grayscale image buffer.

This makes the same ORB library usable with different input sources without modifying the extraction algorithm.

---

## Features

### Multi-Scale Detection

The extractor builds an image pyramid to detect features at different image scales.

Default configuration:

| Parameter              |  Default |
| ---------------------- | -------: |
| Features               |     1000 |
| Scale factor           |      1.2 |
| Pyramid levels         |        8 |
| FAST threshold         |       20 |
| Minimum FAST threshold |        7 |
| Descriptor size        | 256 bits |
| Descriptor storage     | 32 bytes |

These parameters can be configured through `ORBSettings`.

### Keypoints

Each detected keypoint contains:

* `x`, `y` — image coordinates
* `octave` — pyramid level
* `angle` — orientation in degrees
* `response` — corner response
* `size` — feature scale

Coordinates are returned in the original image coordinate system.

### Descriptors

Each keypoint receives a:

```text
256-bit binary descriptor
32 bytes
```

Binary descriptors can be compared efficiently using Hamming distance.

---

## OpenCV Independence

The **computational ORB core is OpenCV-independent**.

The following components are implemented using standard C++ and native data structures:

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
```

The core library does not include OpenCV headers or link against OpenCV.

OpenCV is used only by the optional `webcam_demo` application for:

* camera capture
* grayscale conversion
* visualization
* GUI/window handling
* screenshot output

Therefore, the ORB library can be built and used on a system without OpenCV.

---

# Building

## Requirements

### Core library

* C++14-compatible compiler
* CMake 3.10 or newer

### Optional webcam demo

* OpenCV 3.x or 4.x
* A working camera supported by OpenCV

---

## Build the Library

From the repository root:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j$(nproc)
```

The core library will be generated as:

```text
build/liborb_standalone_lib.a
```

If OpenCV is available, the webcam demonstration will also be built:

```text
build/webcam_demo
```

If OpenCV is not installed, the core library still builds normally and the webcam demo is skipped.

---

# Using the ORB Library

The ORB extractor accepts an 8-bit grayscale image stored in a normal memory buffer.

A minimal example:

```cpp
#include <iostream>
#include <vector>

#include "ORBExtractor.h"
#include "ORBTypes.h"

int main()
{
    using namespace ORB_Standalone;

    // Configure ORB.
    ORBSettings settings;
    settings.nFeatures = 1000;
    settings.scaleFactor = 1.2f;
    settings.nLevels = 8;
    settings.iniThFAST = 20;
    settings.minThFAST = 7;

    ORBExtractor extractor(settings);

    // Example 640x480 grayscale image.
    const int width = 640;
    const int height = 480;

    std::vector<uint8_t> pixels(width * height, 128);

    // NativeImage does not copy the image.
    NativeImage image(
        pixels.data(),
        width,
        height,
        width
    );

    // Extract features.
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

### Important

`NativeImage` is a lightweight view over an existing image buffer.

The constructor arguments are:

```cpp
NativeImage(
    data,
    width,
    height,
    stride
);
```

where:

* `data` = pointer to grayscale pixel data
* `width` = image width in pixels
* `height` = image height in pixels
* `stride` = number of bytes between consecutive image rows

The image must contain **8-bit single-channel grayscale pixels**.

The ORB core does not require the image to be copied into an OpenCV `cv::Mat`.

---

# Using Camera or Other Image Sources

Because the ORB core only requires a grayscale image buffer, it can be connected to almost any image source.

Typical integration looks like:

```text
Camera / Sensor / File
        │
        ▼
  Obtain image frame
        │
        ▼
 Convert to 8-bit grayscale
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

Examples include:

### USB Camera

Use OpenCV, V4L2, or another camera API to acquire the frame, convert it to grayscale, and pass the buffer to `NativeImage`.

### Raspberry Pi / CSI Camera

A camera application using `libcamera` or another capture interface can provide the grayscale/Y-plane buffer directly to the ORB extractor.

### ROS / ROS 2

A camera node can convert an incoming image message into the required 8-bit grayscale format and pass its buffer to `NativeImage`.

### Image or Video Files

Decode the image/video using your preferred library, convert the frame to grayscale, and pass the resulting buffer to ORB.

### Custom Camera or Sensor

Any system capable of providing an 8-bit grayscale image buffer can be connected without changing the ORB implementation.

This separation allows camera-specific code to remain outside the ORB library.

---

# Webcam Demo

The repository includes a small OpenCV-based webcam application:

```text
examples/webcam_demo.cpp
```

Build it with OpenCV installed:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j$(nproc)
```

Run the default camera:

```bash
./webcam_demo
```

Or select a camera index:

```bash
./webcam_demo 1
```

You can also specify resolution:

```bash
./webcam_demo 0 1280 720
```

The demo displays:

* live camera feed
* detected ORB keypoints
* keypoint orientation
* pyramid-level visualization
* feature count
* image resolution
* ORB processing time
* processing FPS

### Keyboard Controls

| Key   | Action                                            |
| ----- | ------------------------------------------------- |
| `ESC` | Exit                                              |
| `Q`   | Exit                                              |
| `S`   | Save the current annotated frame and feature data |
| `D`   | Toggle the information overlay                    |

The webcam application is only an example of how to connect a camera to the ORB library. It is **not required** when using the core library.

---

# Output

The extractor provides:

```cpp
std::vector<NativeKeyPoint>
std::vector<NativeDescriptor>
```

A `NativeKeyPoint` contains the location and properties of a detected feature.

A `NativeDescriptor` contains:

```text
32 bytes = 256 bits
```

The descriptors can be matched using Hamming distance.

For example, conceptually:

```text
descriptor A
     XOR
descriptor B
     │
     ▼
 count set bits
     │
     ▼
Hamming distance
```

This makes ORB descriptors suitable for fast feature matching.

---

# Applications

ORB-Standalone can be used as a feature-extraction component in:

* Visual Odometry (VO)
* Visual-Inertial Odometry (VIO)
* SLAM systems
* Visual tracking
* Feature matching
* Image alignment
* Image stitching
* Robotics
* Embedded computer vision
* Autonomous systems
* Academic and research projects
* Custom computer-vision pipelines

The repository provides the **feature extraction layer**. Higher-level systems such as odometry, mapping, pose estimation, or tracking can be built around its output.

---

# Project Structure

```text
ORB-Standalone/
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
│   ├── ORBExtractor.cpp
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

# Validation

The implementation has been independently tested at both individual pipeline stages and end-to-end extraction.

Validation covers:

* image pyramid construction
* border handling
* FAST detection
* feature distribution
* orientation estimation
* Gaussian filtering
* BRIEF descriptor generation
* keypoint coordinates
* descriptor consistency
* deterministic repeated extraction
* multiple image resolutions
* different image contrast conditions
* real webcam frames

The computational core has also been checked to ensure that it does not depend on OpenCV.

---

# Design Goals

ORB-Standalone is designed around four principles:

**Modularity**
Each major stage of the ORB pipeline is implemented separately.

**Portability**
The core uses standard C++ and a simple image-buffer interface, making it suitable for desktop and embedded systems.

**Independence**
Camera acquisition and visualization remain outside the ORB library.

**Practicality**
The library can be directly integrated into existing VO, VIO, SLAM, robotics, or academic projects without adopting a large computer-vision framework.

---

# License

This project is licensed under the **MIT License**.

See [`LICENSE`](LICENSE) for the complete license text.

---

## Quick Start

For the shortest path to trying ORB-Standalone:

```bash
git clone <repository-url>
cd ORB-Standalone

mkdir build
cd build

cmake ..
cmake --build . -j$(nproc)
```

If OpenCV is installed:

```bash
./webcam_demo
```

For integration into your own application, include:

```cpp
#include "ORBExtractor.h"
#include "ORBTypes.h"
```

Then provide an 8-bit grayscale image buffer to `ORBExtractor`.

That's all that is required to use the ORB feature extractor.
